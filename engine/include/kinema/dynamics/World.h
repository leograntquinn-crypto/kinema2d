// =============================================================================
// World：物理世界，引擎的总入口
//
// 一次 Step(dt) 的流水线：
//
//   ┌──────────────┐   ┌──────────────┐   ┌───────────────┐   ┌──────────────┐   ┌───────────┐
//   │ 1. 宽阶段     │ → │ 2. 窄阶段     │ → │ 3. 积分速度    │ → │ 4. 约束求解   │ → │ 5. 积分位置 │
//   │ AABB 候选对   │   │ 生成接触流形  │   │ 重力/外力      │   │ 顺序冲量迭代  │   │ 更新位姿    │
//   └──────────────┘   └──────────────┘   └───────────────┘   └──────────────┘   └───────────┘
//                                                                                      ↓
//                                                                 ┌────────────────────────────┐
//                                                                 │ 6. 岛屿构建 + 休眠判定       │
//                                                                 └────────────────────────────┘
//
// 积分方式：半隐式欧拉（Symplectic Euler）——先更新速度，再用新速度更新位置。
// 它是能量近似守恒的一阶方法，游戏物理的标准选择。
// =============================================================================
#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "kinema/collision/BroadPhase.h"
#include "kinema/dynamics/Body.h"
#include "kinema/dynamics/Contact.h"
#include "kinema/joints/Joint.h"

namespace kinema {

struct WorldSettings {
    Vec2 gravity{0.0f, -9.81f};
    int velocityIterations = 10;
    bool warmStarting = true;
    bool allowSleep = true;

    float baumgarte = 0.2f;
    float linearSlop = kLinearSlop;
    float restitutionThreshold = 1.0f;

    float maxTranslationPerStep = 2.0f;           // 防止数值爆炸
    float maxRotationPerStep = 0.5f * kPi;

    float linearSleepTolerance = 0.05f;           // m/s
    float angularSleepTolerance = DegToRad(2.0f); // rad/s
    float timeToSleep = 0.5f;                     // s

    BroadPhaseType broadPhase = BroadPhaseType::SweepAndPrune;

    bool enableKillPlane = true;                  // 掉出世界的动态物体自动删除
    float killPlaneY = -100.0f;
};

/// 每一步的统计信息（界面中的性能面板会显示）
struct WorldStats {
    int bodyCount = 0;
    int awakeBodyCount = 0;
    int jointCount = 0;
    int pairCount = 0;      // 宽阶段候选对
    int contactCount = 0;   // 实际接触
    int pointCount = 0;     // 接触点
    int islandCount = 0;
    float broadPhaseMs = 0.0f;
    float narrowPhaseMs = 0.0f;
    float solverMs = 0.0f;
    float integrateMs = 0.0f;
    float sleepMs = 0.0f;
    float totalMs = 0.0f;
    uint64_t stepCount = 0;
};

struct RayHit {
    Body* body = nullptr;
    Vec2 point;
    Vec2 normal;
    float fraction = 1.0f;
};

class World {
public:
    explicit World(const WorldSettings& settings = {});
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // ---- 物体 ---------------------------------------------------------------
    Body* CreateBody(const BodyDef& def, const Shape& shape);
    void DestroyBody(BodyId id);
    [[nodiscard]] Body* FindBody(BodyId id) const;
    [[nodiscard]] const std::vector<std::unique_ptr<Body>>& GetBodies() const { return m_bodies; }

    // ---- 关节 ---------------------------------------------------------------
    DistanceJoint* CreateJoint(const DistanceJointDef& def);
    RevoluteJoint* CreateJoint(const RevoluteJointDef& def);
    MouseJoint* CreateJoint(const MouseJointDef& def);
    void DestroyJoint(JointId id);
    [[nodiscard]] Joint* FindJoint(JointId id) const;
    [[nodiscard]] const std::vector<std::unique_ptr<Joint>>& GetJoints() const { return m_joints; }

    // ---- 接触 ---------------------------------------------------------------
    void ForEachContact(const std::function<void(const Contact&)>& fn) const;

    /// 删除所有物体、关节与接触
    void Clear();

    /// 推进模拟 dt 秒
    void Step(float dt);

    // ---- 查询 ---------------------------------------------------------------
    /// 返回包含该点的物体（优先返回非静态物体）
    [[nodiscard]] Body* QueryPoint(Vec2 p) const;
    void QueryAABB(const AABB& box, std::vector<Body*>& out) const;
    /// 最近命中的射线检测
    bool RayCast(Vec2 p1, Vec2 p2, RayHit& hit) const;

    /// 以 center 为圆心施加径向爆炸冲量
    void ApplyExplosion(Vec2 center, float radius, float strength);

    // ---- 设置与统计 ---------------------------------------------------------
    [[nodiscard]] const WorldSettings& GetSettings() const { return m_settings; }
    void SetSettings(const WorldSettings& settings);
    void SetGravity(Vec2 g);
    [[nodiscard]] const WorldStats& GetStats() const { return m_stats; }
    [[nodiscard]] float ComputeKineticEnergy() const;

    /// 唤醒所有物体
    void WakeAll();

private:
    template <typename TJoint, typename TDef>
    TJoint* AddJoint(const TDef& def);

    void UpdatePairs();
    void Solve(float dt);
    void UpdateSleep(float dt);
    void RemoveOutOfBounds();
    void RebuildJointPairs();

    static uint64_t PairKey(BodyId a, BodyId b) {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
    }

    WorldSettings m_settings;
    WorldStats m_stats;

    std::vector<std::unique_ptr<Body>> m_bodies;
    std::unordered_map<BodyId, Body*> m_bodyMap;
    std::vector<std::unique_ptr<Joint>> m_joints;
    std::unordered_map<uint64_t, Contact> m_contacts;
    std::unordered_map<uint64_t, int> m_jointPairs;  // 不允许碰撞的关节连接对

    std::unique_ptr<BroadPhase> m_broadPhase;

    // 每步复用的临时缓冲区（避免频繁分配内存）
    std::vector<BroadPhaseProxy> m_proxies;
    std::vector<ProxyPair> m_pairs;
    std::vector<SolverBody> m_solverBodies;
    std::vector<Contact*> m_activeContacts;
    std::vector<Joint*> m_activeJoints;
    std::vector<int> m_islandParent;

    BodyId m_nextBodyId = 1;
    JointId m_nextJointId = 1;
    uint32_t m_stepIndex = 0;
};

}  // namespace kinema
