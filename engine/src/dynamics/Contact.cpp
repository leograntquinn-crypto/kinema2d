#include "kinema/dynamics/Contact.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace kinema {

Contact::Contact(Body* a, Body* b)
    : m_bodyA(a),
      m_bodyB(b),
      m_friction(MixFriction(a->GetMaterial().friction, b->GetMaterial().friction)),
      m_restitution(MixRestitution(a->GetMaterial().restitution, b->GetMaterial().restitution)) {
    assert(a->GetShapeType() >= b->GetShapeType());
}

void Contact::Update(bool warmStarting) {
    const Manifold old = m_manifold;

    // 材质可能在编辑器中被修改，所以每帧重新混合
    m_friction = MixFriction(m_bodyA->GetMaterial().friction, m_bodyB->GetMaterial().friction);
    m_restitution = MixRestitution(m_bodyA->GetMaterial().restitution, m_bodyB->GetMaterial().restitution);

    Collide(m_manifold, m_bodyA->GetShape(), m_bodyA->GetTransform(), m_bodyB->GetShape(), m_bodyB->GetTransform());
    m_touching = m_manifold.pointCount > 0;

    // 通过特征 ID 匹配新旧接触点，继承累积冲量
    for (int i = 0; i < m_manifold.pointCount; ++i) {
        ManifoldPoint& mp = m_manifold.points[i];
        mp.normalImpulse = 0.0f;
        mp.tangentImpulse = 0.0f;
        if (!warmStarting) continue;

        for (int j = 0; j < old.pointCount; ++j) {
            if (old.points[j].id == mp.id) {
                mp.normalImpulse = old.points[j].normalImpulse;
                mp.tangentImpulse = old.points[j].tangentImpulse;
                break;
            }
        }
    }
}

// =============================================================================
// ContactSolver
// =============================================================================
ContactSolver::ContactSolver(const StepContext& ctx, std::vector<Contact*>& contacts)
    : m_ctx(ctx), m_contacts(contacts) {}

void ContactSolver::PrepareAndWarmStart() {
    std::vector<SolverBody>& bodies = *m_ctx.bodies;

    for (Contact* c : m_contacts) {
        SolverBody& A = bodies[static_cast<size_t>(c->m_bodyA->m_solverIndex)];
        SolverBody& B = bodies[static_cast<size_t>(c->m_bodyB->m_solverIndex)];
        Manifold& m = c->m_manifold;

        const Vec2 n = m.normal;
        const Vec2 t = Cross(n, 1.0f);  // 切线方向

        for (int i = 0; i < m.pointCount; ++i) {
            ManifoldPoint& mp = m.points[i];
            mp.rA = mp.point - A.p;
            mp.rB = mp.point - B.p;

            // 有效质量 K = mA + mB + iA (rA×n)² + iB (rB×n)²
            // 它表示"沿 n 方向施加单位冲量，接触点相对速度改变多少"的倒数
            const float rnA = Cross(mp.rA, n);
            const float rnB = Cross(mp.rB, n);
            const float kNormal = A.invMass + B.invMass + A.invI * rnA * rnA + B.invI * rnB * rnB;
            mp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

            const float rtA = Cross(mp.rA, t);
            const float rtB = Cross(mp.rB, t);
            const float kTangent = A.invMass + B.invMass + A.invI * rtA * rtA + B.invI * rtB * rtB;
            mp.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

            // 偏置速度 = max(反弹速度, Baumgarte 位置修正速度)
            const Vec2 dv = B.v + Cross(B.w, mp.rB) - A.v - Cross(A.w, mp.rA);
            const float vn = Dot(dv, n);
            float restitutionBias = 0.0f;
            if (vn < -m_ctx.restitutionThreshold) restitutionBias = -c->m_restitution * vn;

            const float penetration = std::max(-mp.separation - m_ctx.linearSlop, 0.0f);
            const float positionBias = m_ctx.baumgarte * m_ctx.invDt * penetration;

            mp.velocityBias = std::max(restitutionBias, positionBias);

            // 热启动：先把上一帧的累积冲量直接施加上去，迭代从"接近正确答案"处开始
            if (m_ctx.warmStarting) {
                const Vec2 P = mp.normalImpulse * n + mp.tangentImpulse * t;
                A.v -= A.invMass * P;
                A.w -= A.invI * Cross(mp.rA, P);
                B.v += B.invMass * P;
                B.w += B.invI * Cross(mp.rB, P);
            } else {
                mp.normalImpulse = 0.0f;
                mp.tangentImpulse = 0.0f;
            }
        }

        // ---- 两点流形：准备块求解器 ----
        c->m_blockSolve = false;
        if (m.pointCount == 2) {
            const ManifoldPoint& p1 = m.points[0];
            const ManifoldPoint& p2 = m.points[1];
            const float rn1A = Cross(p1.rA, n), rn1B = Cross(p1.rB, n);
            const float rn2A = Cross(p2.rA, n), rn2B = Cross(p2.rB, n);
            const float k11 = A.invMass + B.invMass + A.invI * rn1A * rn1A + B.invI * rn1B * rn1B;
            const float k22 = A.invMass + B.invMass + A.invI * rn2A * rn2A + B.invI * rn2B * rn2B;
            const float k12 = A.invMass + B.invMass + A.invI * rn1A * rn2A + B.invI * rn1B * rn2B;

            // 条件数检查：两个点几乎重合时矩阵接近奇异，退回逐点求解
            constexpr float kMaxConditionNumber = 1000.0f;
            if (k11 * k11 < kMaxConditionNumber * (k11 * k22 - k12 * k12)) {
                c->m_K = Mat22({k11, k12}, {k12, k22});
                c->m_normalMassBlock = c->m_K.GetInverse();
                c->m_blockSolve = true;
            }
        }
    }
}

void ContactSolver::SolveVelocity() {
    std::vector<SolverBody>& bodies = *m_ctx.bodies;

    for (Contact* c : m_contacts) {
        SolverBody& A = bodies[static_cast<size_t>(c->m_bodyA->m_solverIndex)];
        SolverBody& B = bodies[static_cast<size_t>(c->m_bodyB->m_solverIndex)];
        Manifold& m = c->m_manifold;

        const Vec2 n = m.normal;
        const Vec2 t = Cross(n, 1.0f);

        // ---- 1) 摩擦（先解摩擦，因为法向约束更重要，后解的约束"更准确"） ----
        for (int i = 0; i < m.pointCount; ++i) {
            ManifoldPoint& mp = m.points[i];
            const Vec2 dv = B.v + Cross(B.w, mp.rB) - A.v - Cross(A.w, mp.rA);
            const float vt = Dot(dv, t);
            float lambda = -mp.tangentMass * vt;

            // 累积冲量钳制到摩擦锥内（注意：钳制的是累积值，而不是增量！）
            const float maxFriction = c->m_friction * mp.normalImpulse;
            const float newImpulse = Clamp(mp.tangentImpulse + lambda, -maxFriction, maxFriction);
            lambda = newImpulse - mp.tangentImpulse;
            mp.tangentImpulse = newImpulse;

            const Vec2 P = lambda * t;
            A.v -= A.invMass * P;
            A.w -= A.invI * Cross(mp.rA, P);
            B.v += B.invMass * P;
            B.w += B.invI * Cross(mp.rB, P);
        }

        // ---- 2) 法向（不穿透） ----
        if (c->m_blockSolve) {
            SolveBlock(*c, A, B);
            continue;
        }
        for (int i = 0; i < m.pointCount; ++i) {
            ManifoldPoint& mp = m.points[i];
            const Vec2 dv = B.v + Cross(B.w, mp.rB) - A.v - Cross(A.w, mp.rA);
            const float vn = Dot(dv, n);
            float lambda = mp.normalMass * (-vn + mp.velocityBias);

            // 累积法向冲量必须 >= 0：接触只能推，不能拉
            const float newImpulse = std::max(mp.normalImpulse + lambda, 0.0f);
            lambda = newImpulse - mp.normalImpulse;
            mp.normalImpulse = newImpulse;

            const Vec2 P = lambda * n;
            A.v -= A.invMass * P;
            A.w -= A.invI * Cross(mp.rA, P);
            B.v += B.invMass * P;
            B.w += B.invI * Cross(mp.rB, P);
        }
    }
}

// =============================================================================
// 块求解器（参考 Box2D b2ContactSolver）
//
// 求 x = (λ1, λ2) 使得：
//   vn = K x + b        （b 为施加 x 之前的法向速度减去偏置）
//   x >= 0, vn >= 0, x · vn = 0     —— 线性互补条件
// 2 维 LCP 只有 4 种可能：两点都受力 / 只有点 1 / 只有点 2 / 都不受力。
// 依次尝试，第一个满足条件的就是解。
// =============================================================================
void ContactSolver::SolveBlock(Contact& c, SolverBody& A, SolverBody& B) {
    Manifold& m = c.m_manifold;
    ManifoldPoint& cp1 = m.points[0];
    ManifoldPoint& cp2 = m.points[1];
    const Vec2 n = m.normal;

    const Vec2 a{cp1.normalImpulse, cp2.normalImpulse};  // 当前累积冲量
    const Vec2 dv1 = B.v + Cross(B.w, cp1.rB) - A.v - Cross(A.w, cp1.rA);
    const Vec2 dv2 = B.v + Cross(B.w, cp2.rB) - A.v - Cross(A.w, cp2.rA);

    // b' = vn - bias - K a
    Vec2 b{Dot(dv1, n) - cp1.velocityBias, Dot(dv2, n) - cp2.velocityBias};
    b -= c.m_K * a;

    auto apply = [&](Vec2 x) {
        const Vec2 d = x - a;
        const Vec2 P1 = d.x * n;
        const Vec2 P2 = d.y * n;
        A.v -= A.invMass * (P1 + P2);
        A.w -= A.invI * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
        B.v += B.invMass * (P1 + P2);
        B.w += B.invI * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
        cp1.normalImpulse = x.x;
        cp2.normalImpulse = x.y;
    };

    // 情况 1：两点都受力  x = -K⁻¹ b'
    Vec2 x = -1.0f * (c.m_normalMassBlock * b);
    if (x.x >= 0.0f && x.y >= 0.0f) {
        apply(x);
        return;
    }

    // 情况 2：只有点 1 受力（x2 = 0）
    x = {-cp1.normalMass * b.x, 0.0f};
    float vn2 = c.m_K.ex.y * x.x + b.y;
    if (x.x >= 0.0f && vn2 >= 0.0f) {
        apply(x);
        return;
    }

    // 情况 3：只有点 2 受力（x1 = 0）
    x = {0.0f, -cp2.normalMass * b.y};
    float vn1 = c.m_K.ey.x * x.y + b.x;
    if (x.y >= 0.0f && vn1 >= 0.0f) {
        apply(x);
        return;
    }

    // 情况 4：两点都分离
    x = {0.0f, 0.0f};
    vn1 = b.x;
    vn2 = b.y;
    if (vn1 >= 0.0f && vn2 >= 0.0f) {
        apply(x);
        return;
    }
    // 数值误差导致无解：保持冲量不变
}

}  // namespace kinema
