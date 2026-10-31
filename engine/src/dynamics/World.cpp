#include "kinema/dynamics/World.h"

#include <algorithm>
#include <cassert>
#include <limits>

#include "kinema/core/Timer.h"

namespace kinema {

World::World(const WorldSettings& settings)
    : m_settings(settings), m_broadPhase(CreateBroadPhase(settings.broadPhase)) {}

World::~World() = default;

// =============================================================================
// 物体
// =============================================================================
Body* World::CreateBody(const BodyDef& def, const Shape& shape) {
    const BodyId id = m_nextBodyId++;
    auto body = std::make_unique<Body>(id, def, shape.Clone());
    Body* raw = body.get();
    m_bodies.push_back(std::move(body));
    m_bodyMap.emplace(id, raw);
    return raw;
}

void World::DestroyBody(BodyId id) {
    Body* body = FindBody(id);
    if (body == nullptr) return;

    // 1) 删除连接到该物体的关节
    std::vector<JointId> jointsToRemove;
    for (const auto& j : m_joints) {
        if (j->GetBodyA() == body || j->GetBodyB() == body) jointsToRemove.push_back(j->GetId());
    }
    for (JointId jid : jointsToRemove) DestroyJoint(jid);

    // 2) 删除相关接触，并唤醒与之接触的物体（否则它们会悬空不动）
    for (auto it = m_contacts.begin(); it != m_contacts.end();) {
        Contact& c = it->second;
        if (c.m_bodyA == body || c.m_bodyB == body) {
            Body* other = c.m_bodyA == body ? c.m_bodyB : c.m_bodyA;
            other->SetAwake(true);
            it = m_contacts.erase(it);
        } else {
            ++it;
        }
    }

    // 3) 从容器中移除
    m_bodyMap.erase(id);
    m_bodies.erase(std::remove_if(m_bodies.begin(), m_bodies.end(),
                                  [body](const std::unique_ptr<Body>& b) { return b.get() == body; }),
                   m_bodies.end());
}

Body* World::FindBody(BodyId id) const {
    const auto it = m_bodyMap.find(id);
    return it != m_bodyMap.end() ? it->second : nullptr;
}

// =============================================================================
// 关节
// =============================================================================
template <typename TJoint, typename TDef>
TJoint* World::AddJoint(const TDef& def) {
    auto joint = std::make_unique<TJoint>(m_nextJointId++, def);
    TJoint* raw = joint.get();
    m_joints.push_back(std::move(joint));
    if (raw->GetBodyA()) raw->GetBodyA()->SetAwake(true);
    if (raw->GetBodyB()) raw->GetBodyB()->SetAwake(true);
    RebuildJointPairs();
    return raw;
}

DistanceJoint* World::CreateJoint(const DistanceJointDef& def) { return AddJoint<DistanceJoint>(def); }
RevoluteJoint* World::CreateJoint(const RevoluteJointDef& def) { return AddJoint<RevoluteJoint>(def); }
MouseJoint* World::CreateJoint(const MouseJointDef& def) { return AddJoint<MouseJoint>(def); }

void World::DestroyJoint(JointId id) {
    const auto it = std::find_if(m_joints.begin(), m_joints.end(),
                                 [id](const std::unique_ptr<Joint>& j) { return j->GetId() == id; });
    if (it == m_joints.end()) return;
    if ((*it)->GetBodyA()) (*it)->GetBodyA()->SetAwake(true);
    if ((*it)->GetBodyB()) (*it)->GetBodyB()->SetAwake(true);
    m_joints.erase(it);
    RebuildJointPairs();
}

Joint* World::FindJoint(JointId id) const {
    for (const auto& j : m_joints) {
        if (j->GetId() == id) return j.get();
    }
    return nullptr;
}

void World::RebuildJointPairs() {
    m_jointPairs.clear();
    for (const auto& j : m_joints) {
        if (j->GetCollideConnected() || !j->GetBodyA() || !j->GetBodyB()) continue;
        ++m_jointPairs[PairKey(j->GetBodyA()->GetId(), j->GetBodyB()->GetId())];
    }
    // 关节可能让原本碰撞的一对物体不再碰撞，清掉旧接触
    for (auto it = m_contacts.begin(); it != m_contacts.end();) {
        if (m_jointPairs.count(it->first)) {
            it = m_contacts.erase(it);
        } else {
            ++it;
        }
    }
}

void World::ForEachContact(const std::function<void(const Contact&)>& fn) const {
    for (const auto& [key, c] : m_contacts) {
        if (c.IsTouching()) fn(c);
    }
}

void World::Clear() {
    m_contacts.clear();
    m_joints.clear();
    m_jointPairs.clear();
    m_bodies.clear();
    m_bodyMap.clear();
    m_stats = {};
}

void World::SetSettings(const WorldSettings& settings) {
    const bool bpChanged = settings.broadPhase != m_settings.broadPhase;
    const bool sleepDisabled = m_settings.allowSleep && !settings.allowSleep;
    m_settings = settings;
    if (bpChanged) m_broadPhase = CreateBroadPhase(settings.broadPhase);
    if (sleepDisabled) WakeAll();
}

void World::SetGravity(Vec2 g) {
    m_settings.gravity = g;
    WakeAll();
}

void World::WakeAll() {
    for (auto& b : m_bodies) b->SetAwake(true);
}

float World::ComputeKineticEnergy() const {
    float e = 0.0f;
    for (const auto& b : m_bodies) e += b->GetKineticEnergy();
    return e;
}

// =============================================================================
// Step
// =============================================================================
void World::Step(float dt) {
    if (dt <= 0.0f) return;

    Timer total;
    ++m_stepIndex;

    // 本步中每个物体的求解器索引 == 它在 m_bodies 中的下标
    for (size_t i = 0; i < m_bodies.size(); ++i) m_bodies[i]->m_solverIndex = static_cast<int>(i);

    // ---- 1 & 2: 宽阶段 + 窄阶段 ----
    UpdatePairs();

    // ---- 3 & 4: 积分速度 + 求解约束 + 积分位置 ----
    Solve(dt);

    // ---- 5: 休眠 ----
    Timer sleepTimer;
    UpdateSleep(dt);
    m_stats.sleepMs = static_cast<float>(sleepTimer.ElapsedMs());

    RemoveOutOfBounds();

    // ---- 统计 ----
    m_stats.bodyCount = static_cast<int>(m_bodies.size());
    m_stats.awakeBodyCount = static_cast<int>(
        std::count_if(m_bodies.begin(), m_bodies.end(), [](const auto& b) { return b->IsAwake(); }));
    m_stats.jointCount = static_cast<int>(m_joints.size());
    m_stats.totalMs = static_cast<float>(total.ElapsedMs());
    ++m_stats.stepCount;
}

void World::UpdatePairs() {
    // ---------------- 宽阶段 ----------------
    Timer bpTimer;
    m_proxies.clear();
    m_proxies.reserve(m_bodies.size());
    for (size_t i = 0; i < m_bodies.size(); ++i) {
        m_proxies.push_back({m_bodies[i]->GetAABB(), static_cast<uint32_t>(i)});
    }
    m_broadPhase->ComputePairs(m_proxies, m_pairs);
    m_stats.pairCount = static_cast<int>(m_pairs.size());
    m_stats.broadPhaseMs = static_cast<float>(bpTimer.ElapsedMs());

    // ---------------- 窄阶段 ----------------
    Timer npTimer;
    for (const auto& [ia, ib] : m_pairs) {
        Body* a = m_bodies[ia].get();
        Body* b = m_bodies[ib].get();

        // 至少一方是动态物体，且至少一方醒着
        if (!a->IsDynamic() && !b->IsDynamic()) continue;
        if (!a->IsAwake() && !b->IsAwake()) continue;
        if (!a->ShouldCollide(*b)) continue;

        const uint64_t key = PairKey(a->GetId(), b->GetId());
        if (m_jointPairs.count(key)) continue;

        auto it = m_contacts.find(key);
        if (it == m_contacts.end()) {
            // 保证 A 的形状类型 >= B，碰撞函数只需处理 3 种组合
            if (a->GetShapeType() < b->GetShapeType()) std::swap(a, b);
            it = m_contacts.emplace(key, Contact(a, b)).first;
        }

        Contact& c = it->second;
        c.m_lastSeenStep = m_stepIndex;
        c.Update(m_settings.warmStarting);

        // 醒着的物体碰到睡着的物体 -> 把它叫醒
        if (c.IsTouching()) {
            Body* ca = c.m_bodyA;
            Body* cb = c.m_bodyB;
            if (ca->IsAwake() && !cb->IsAwake()) cb->SetAwake(true);
            if (cb->IsAwake() && !ca->IsAwake()) ca->SetAwake(true);
        }
    }

    // 清理本步未被宽阶段报告的接触（两个都睡着的接触保留，醒来后可继续热启动）
    for (auto it = m_contacts.begin(); it != m_contacts.end();) {
        const Contact& c = it->second;
        const bool anyAwake = c.m_bodyA->IsAwake() || c.m_bodyB->IsAwake();
        if (c.m_lastSeenStep != m_stepIndex && anyAwake) {
            it = m_contacts.erase(it);
        } else {
            ++it;
        }
    }

    // 关节连接的物体同样需要一起醒来
    for (const auto& j : m_joints) {
        Body* a = j->GetBodyA();
        Body* b = j->GetBodyB();
        if (!a || !b) continue;
        if (a->IsAwake() && !b->IsAwake()) b->SetAwake(true);
        if (b->IsAwake() && !a->IsAwake()) a->SetAwake(true);
    }

    m_stats.narrowPhaseMs = static_cast<float>(npTimer.ElapsedMs());
}

void World::Solve(float dt) {
    // ---------------- 积分速度 ----------------
    Timer intTimer;
    const size_t n = m_bodies.size();
    m_solverBodies.resize(n);

    for (size_t i = 0; i < n; ++i) {
        Body* b = m_bodies[i].get();
        SolverBody& sb = m_solverBodies[i];
        sb.p = b->m_position;
        sb.q = b->m_xf.q;

        if (!b->IsAwake()) {
            // 静态或睡眠物体：对求解器而言质量无穷大、速度为零
            sb.v = {};
            sb.w = 0.0f;
            sb.invMass = 0.0f;
            sb.invI = 0.0f;
            continue;
        }

        sb.invMass = b->m_invMass;
        sb.invI = b->m_invInertia;
        Vec2 v = b->m_linearVelocity;
        float w = b->m_angularVelocity;

        if (b->IsDynamic()) {
            // v += dt * (g + F/m)，ω += dt * τ/I
            v += dt * (b->m_gravityScale * m_settings.gravity + b->m_invMass * b->m_force);
            w += dt * b->m_invInertia * b->m_torque;

            // 阻尼：dv/dt = -c v 的隐式解 v' = v / (1 + c dt)，无条件稳定
            v *= 1.0f / (1.0f + dt * b->m_linearDamping);
            w *= 1.0f / (1.0f + dt * b->m_angularDamping);
        }
        sb.v = v;
        sb.w = w;
    }
    float integrateMs = static_cast<float>(intTimer.ElapsedMs());

    // ---------------- 求解约束 ----------------
    Timer solveTimer;

    m_activeContacts.clear();
    int pointCount = 0;
    for (auto& [key, c] : m_contacts) {
        if (!c.IsTouching()) continue;
        if (!c.m_bodyA->IsAwake() && !c.m_bodyB->IsAwake()) continue;
        m_activeContacts.push_back(&c);
        pointCount += c.m_manifold.pointCount;
    }
    m_stats.contactCount = static_cast<int>(m_activeContacts.size());
    m_stats.pointCount = pointCount;

    m_activeJoints.clear();
    for (auto& j : m_joints) {
        const bool awakeA = j->GetBodyA() && j->GetBodyA()->IsAwake();
        const bool awakeB = j->GetBodyB() && j->GetBodyB()->IsAwake();
        if (awakeA || awakeB) m_activeJoints.push_back(j.get());
    }

    StepContext ctx;
    ctx.dt = dt;
    ctx.invDt = 1.0f / dt;
    ctx.warmStarting = m_settings.warmStarting;
    ctx.baumgarte = m_settings.baumgarte;
    ctx.linearSlop = m_settings.linearSlop;
    ctx.restitutionThreshold = m_settings.restitutionThreshold;
    ctx.bodies = &m_solverBodies;

    ContactSolver contactSolver(ctx, m_activeContacts);
    contactSolver.PrepareAndWarmStart();
    for (Joint* j : m_activeJoints) j->PrepareAndWarmStart(ctx);

    for (int it = 0; it < m_settings.velocityIterations; ++it) {
        for (Joint* j : m_activeJoints) j->SolveVelocity(ctx);
        contactSolver.SolveVelocity();
    }
    m_stats.solverMs = static_cast<float>(solveTimer.ElapsedMs());

    // ---------------- 积分位置 & 写回 ----------------
    intTimer.Reset();
    for (size_t i = 0; i < n; ++i) {
        Body* b = m_bodies[i].get();
        if (!b->IsAwake()) continue;

        Vec2 v = m_solverBodies[i].v;
        float w = m_solverBodies[i].w;

        // 限制单步最大位移/旋转，防止数值不稳定时物体"飞出宇宙"
        const Vec2 translation = dt * v;
        const float maxT = m_settings.maxTranslationPerStep;
        if (translation.LengthSquared() > maxT * maxT) v *= maxT / translation.Length();
        const float rotation = dt * w;
        const float maxR = m_settings.maxRotationPerStep;
        if (rotation * rotation > maxR * maxR) w *= maxR / std::fabs(rotation);

        b->m_linearVelocity = v;
        b->m_angularVelocity = w;
        b->m_position += dt * v;
        b->m_angle += dt * w;
        b->SynchronizeTransform();

        b->m_force = {};
        b->m_torque = 0.0f;
    }
    integrateMs += static_cast<float>(intTimer.ElapsedMs());
    m_stats.integrateMs = integrateMs;
}

// =============================================================================
// 休眠：用并查集（Union-Find）把通过接触/关节相连的物体合并成"岛屿"，
// 只有整个岛屿都足够安静时才一起入睡 —— 否则堆叠中的箱子会悬在半空。
// =============================================================================
namespace {

int FindRoot(std::vector<int>& parent, int x) {
    while (parent[static_cast<size_t>(x)] != x) {
        parent[static_cast<size_t>(x)] = parent[static_cast<size_t>(parent[static_cast<size_t>(x)])];  // 路径压缩
        x = parent[static_cast<size_t>(x)];
    }
    return x;
}

void Union(std::vector<int>& parent, int a, int b) {
    a = FindRoot(parent, a);
    b = FindRoot(parent, b);
    if (a != b) parent[static_cast<size_t>(a)] = b;
}

}  // namespace

void World::UpdateSleep(float dt) {
    const size_t n = m_bodies.size();
    const float linTolSq = m_settings.linearSleepTolerance * m_settings.linearSleepTolerance;
    const float angTolSq = m_settings.angularSleepTolerance * m_settings.angularSleepTolerance;

    // 1) 更新每个物体的"安静时间"
    for (auto& b : m_bodies) {
        if (!b->IsAwake()) continue;
        const bool quiet = b->IsDynamic() && m_settings.allowSleep && b->m_allowSleep &&
                           b->m_linearVelocity.LengthSquared() <= linTolSq &&
                           b->m_angularVelocity * b->m_angularVelocity <= angTolSq;
        b->m_sleepTime = quiet ? b->m_sleepTime + dt : 0.0f;
    }

    // 被鼠标拖拽的物体不允许睡眠
    for (const auto& j : m_joints) {
        if (j->GetType() == JointType::Mouse) j->GetBodyB()->m_sleepTime = 0.0f;
    }

    // 2) 构建岛屿
    m_islandParent.resize(n);
    for (size_t i = 0; i < n; ++i) m_islandParent[i] = static_cast<int>(i);

    for (Contact* c : m_activeContacts) {
        if (c->m_bodyA->IsStatic() || c->m_bodyB->IsStatic()) continue;  // 静态物体不传播岛屿
        Union(m_islandParent, c->m_bodyA->m_solverIndex, c->m_bodyB->m_solverIndex);
    }
    for (Joint* j : m_activeJoints) {
        Body* a = j->GetBodyA();
        Body* b = j->GetBodyB();
        if (!a || !b || a->IsStatic() || b->IsStatic()) continue;
        Union(m_islandParent, a->m_solverIndex, b->m_solverIndex);
    }

    // 3) 每个岛屿取最小安静时间
    std::vector<float> minSleep(n, std::numeric_limits<float>::max());
    std::vector<char> hasAwake(n, 0);
    for (size_t i = 0; i < n; ++i) {
        Body* b = m_bodies[i].get();
        if (!b->IsAwake()) continue;
        const auto root = static_cast<size_t>(FindRoot(m_islandParent, static_cast<int>(i)));
        hasAwake[root] = 1;
        minSleep[root] = std::min(minSleep[root], b->m_sleepTime);
    }

    int islands = 0;
    for (size_t i = 0; i < n; ++i) {
        if (hasAwake[i]) ++islands;
    }
    m_stats.islandCount = islands;

    if (!m_settings.allowSleep) return;

    // 4) 岛屿整体入睡
    for (size_t i = 0; i < n; ++i) {
        Body* b = m_bodies[i].get();
        if (!b->IsAwake()) continue;
        const auto root = static_cast<size_t>(FindRoot(m_islandParent, static_cast<int>(i)));
        if (minSleep[root] >= m_settings.timeToSleep) b->SetAwake(false);
    }
}

void World::RemoveOutOfBounds() {
    if (!m_settings.enableKillPlane) return;
    std::vector<BodyId> toRemove;
    for (const auto& b : m_bodies) {
        if (b->IsDynamic() && b->GetPosition().y < m_settings.killPlaneY) toRemove.push_back(b->GetId());
    }
    for (BodyId id : toRemove) DestroyBody(id);
}

// =============================================================================
// 查询
// =============================================================================
Body* World::QueryPoint(Vec2 p) const {
    Body* staticHit = nullptr;
    // 反向遍历：后创建的物体绘制在上层，优先被选中
    for (auto it = m_bodies.rbegin(); it != m_bodies.rend(); ++it) {
        Body* b = it->get();
        if (!b->GetAABB().Contains(p)) continue;
        if (!b->GetShape().TestPoint(b->GetTransform(), p)) continue;
        if (!b->IsStatic()) return b;
        if (!staticHit) staticHit = b;
    }
    return staticHit;
}

void World::QueryAABB(const AABB& box, std::vector<Body*>& out) const {
    out.clear();
    for (const auto& b : m_bodies) {
        if (Overlaps(b->GetAABB(), box)) out.push_back(b.get());
    }
}

bool World::RayCast(Vec2 p1, Vec2 p2, RayHit& hit) const {
    RayCastInput input{p1, p2, 1.0f};
    const AABB rayBox{Min(p1, p2), Max(p1, p2)};
    bool found = false;

    for (const auto& b : m_bodies) {
        if (!Overlaps(b->GetAABB(), rayBox)) continue;
        RayCastOutput out;
        if (b->GetShape().RayCast(input, b->GetTransform(), out)) {
            input.maxFraction = out.fraction;  // 只接受更近的命中
            hit.body = b.get();
            hit.fraction = out.fraction;
            hit.normal = out.normal;
            hit.point = p1 + out.fraction * (p2 - p1);
            found = true;
        }
    }
    return found;
}

void World::ApplyExplosion(Vec2 center, float radius, float strength) {
    for (const auto& b : m_bodies) {
        if (!b->IsDynamic()) continue;
        Vec2 d = b->GetPosition() - center;
        const float dist = d.Normalize();
        if (dist >= radius) continue;
        if (dist < kEpsilon) d = {0.0f, 1.0f};
        const float falloff = 1.0f - dist / radius;
        // 冲量与质量成正比 => 所有物体获得相同的速度变化（与大小无关）
        b->ApplyLinearImpulseToCenter(strength * falloff * b->GetMass() * d);
    }
}

}  // namespace kinema
