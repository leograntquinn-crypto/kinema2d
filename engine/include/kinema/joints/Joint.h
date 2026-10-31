// =============================================================================
// 关节（Joint）—— 物体之间的约束
//
//   DistanceJoint : 保持两个锚点之间的距离（刚性杆，或设置频率后变成弹簧）
//   RevoluteJoint : 两个锚点重合，物体可以绕该点自由旋转（铰链），可选马达
//   MouseJoint    : 用软约束把物体上的一点拉向目标点（鼠标拖拽）
//
// 所有关节都使用与接触相同的"顺序冲量"框架：
//   PrepareAndWarmStart() 预计算有效质量 + 热启动
//   SolveVelocity()       每次迭代施加修正冲量
//
// 软约束（Soft Constraint）参数：
//   frequencyHz  : 弹簧固有频率，越大越硬
//   dampingRatio : 阻尼比，1 = 临界阻尼
// =============================================================================
#pragma once

#include <cstdint>

#include "kinema/dynamics/Body.h"
#include "kinema/dynamics/TimeStep.h"

namespace kinema {

using JointId = uint32_t;
inline constexpr JointId kInvalidJointId = 0;

enum class JointType : uint8_t {
    Distance = 0,
    Revolute = 1,
    Mouse = 2,
};

const char* ToString(JointType type);

struct JointDef {
    Body* bodyA = nullptr;
    Body* bodyB = nullptr;
    bool collideConnected = false;  // 被连接的两个物体之间是否还要碰撞
    uint64_t userData = 0;
};

struct DistanceJointDef : JointDef {
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    float length = 1.0f;
    float frequencyHz = 0.0f;  // 0 = 刚性
    float dampingRatio = 0.7f;

    /// 用世界坐标锚点初始化，长度取当前距离
    void Initialize(Body* a, Body* b, Vec2 worldAnchorA, Vec2 worldAnchorB);
};

struct RevoluteJointDef : JointDef {
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    bool enableMotor = false;
    float motorSpeed = 0.0f;      // rad/s
    float maxMotorTorque = 0.0f;  // N·m

    void Initialize(Body* a, Body* b, Vec2 worldAnchor);
};

struct MouseJointDef : JointDef {
    // bodyA 不使用（可为空），bodyB 为被拖拽的物体
    Vec2 target;
    float maxForce = 1000.0f;
    float frequencyHz = 5.0f;
    float dampingRatio = 0.7f;
};

// -----------------------------------------------------------------------------
class Joint {
public:
    virtual ~Joint() = default;

    Joint(const Joint&) = delete;
    Joint& operator=(const Joint&) = delete;

    [[nodiscard]] JointId GetId() const { return m_id; }
    [[nodiscard]] JointType GetType() const { return m_type; }
    [[nodiscard]] Body* GetBodyA() const { return m_bodyA; }
    [[nodiscard]] Body* GetBodyB() const { return m_bodyB; }
    [[nodiscard]] bool GetCollideConnected() const { return m_collideConnected; }
    [[nodiscard]] uint64_t GetUserData() const { return m_userData; }

    /// 世界坐标锚点（用于绘制）
    [[nodiscard]] virtual Vec2 GetAnchorA() const = 0;
    [[nodiscard]] virtual Vec2 GetAnchorB() const = 0;

    /// 上一步中约束施加的力（冲量 / dt）
    [[nodiscard]] virtual Vec2 GetReactionForce(float invDt) const = 0;

protected:
    Joint(JointId id, JointType type, const JointDef& def);

    friend class World;

    virtual void PrepareAndWarmStart(const StepContext& ctx) = 0;
    virtual void SolveVelocity(const StepContext& ctx) = 0;

    /// 取得物体在本步求解器数组中的数据
    static SolverBody& Solver(const StepContext& ctx, const Body* body) {
        return (*ctx.bodies)[static_cast<size_t>(body->m_solverIndex)];
    }

    JointId m_id;
    JointType m_type;
    Body* m_bodyA;
    Body* m_bodyB;
    bool m_collideConnected;
    uint64_t m_userData;
};

// -----------------------------------------------------------------------------
class DistanceJoint final : public Joint {
public:
    DistanceJoint(JointId id, const DistanceJointDef& def);

    [[nodiscard]] Vec2 GetAnchorA() const override { return m_bodyA->GetWorldPoint(m_localAnchorA); }
    [[nodiscard]] Vec2 GetAnchorB() const override { return m_bodyB->GetWorldPoint(m_localAnchorB); }
    [[nodiscard]] Vec2 GetReactionForce(float invDt) const override { return (invDt * m_impulse) * m_u; }

    [[nodiscard]] Vec2 GetLocalAnchorA() const { return m_localAnchorA; }
    [[nodiscard]] Vec2 GetLocalAnchorB() const { return m_localAnchorB; }
    [[nodiscard]] float GetLength() const { return m_length; }
    void SetLength(float length) { m_length = std::max(length, 0.01f); }
    [[nodiscard]] float GetFrequency() const { return m_frequencyHz; }
    void SetFrequency(float hz) { m_frequencyHz = std::max(hz, 0.0f); }
    [[nodiscard]] float GetDampingRatio() const { return m_dampingRatio; }
    void SetDampingRatio(float r) { m_dampingRatio = std::max(r, 0.0f); }

private:
    void PrepareAndWarmStart(const StepContext& ctx) override;
    void SolveVelocity(const StepContext& ctx) override;

    Vec2 m_localAnchorA, m_localAnchorB;
    float m_length, m_frequencyHz, m_dampingRatio;

    // 求解器临时数据
    Vec2 m_u, m_rA, m_rB;
    float m_mass = 0.0f, m_bias = 0.0f, m_gamma = 0.0f;
    float m_impulse = 0.0f;
};

// -----------------------------------------------------------------------------
class RevoluteJoint final : public Joint {
public:
    RevoluteJoint(JointId id, const RevoluteJointDef& def);

    [[nodiscard]] Vec2 GetAnchorA() const override { return m_bodyA->GetWorldPoint(m_localAnchorA); }
    [[nodiscard]] Vec2 GetAnchorB() const override { return m_bodyB->GetWorldPoint(m_localAnchorB); }
    [[nodiscard]] Vec2 GetReactionForce(float invDt) const override { return invDt * m_impulse; }

    [[nodiscard]] Vec2 GetLocalAnchorA() const { return m_localAnchorA; }
    [[nodiscard]] Vec2 GetLocalAnchorB() const { return m_localAnchorB; }

    [[nodiscard]] bool IsMotorEnabled() const { return m_enableMotor; }
    void EnableMotor(bool flag);
    [[nodiscard]] float GetMotorSpeed() const { return m_motorSpeed; }
    void SetMotorSpeed(float speed);
    [[nodiscard]] float GetMaxMotorTorque() const { return m_maxMotorTorque; }
    void SetMaxMotorTorque(float torque) { m_maxMotorTorque = std::max(torque, 0.0f); }
    [[nodiscard]] float GetMotorTorque(float invDt) const { return invDt * m_motorImpulse; }

private:
    void PrepareAndWarmStart(const StepContext& ctx) override;
    void SolveVelocity(const StepContext& ctx) override;

    Vec2 m_localAnchorA, m_localAnchorB;
    bool m_enableMotor;
    float m_motorSpeed, m_maxMotorTorque;

    Vec2 m_rA, m_rB;
    Mat22 m_mass;
    Vec2 m_bias;
    float m_motorMass = 0.0f;
    Vec2 m_impulse;
    float m_motorImpulse = 0.0f;
};

// -----------------------------------------------------------------------------
class MouseJoint final : public Joint {
public:
    MouseJoint(JointId id, const MouseJointDef& def);

    [[nodiscard]] Vec2 GetAnchorA() const override { return m_target; }
    [[nodiscard]] Vec2 GetAnchorB() const override { return m_bodyB->GetWorldPoint(m_localAnchorB); }
    [[nodiscard]] Vec2 GetReactionForce(float invDt) const override { return invDt * m_impulse; }

    void SetTarget(Vec2 target);
    [[nodiscard]] Vec2 GetTarget() const { return m_target; }
    void SetMaxForce(float f) { m_maxForce = f; }

private:
    void PrepareAndWarmStart(const StepContext& ctx) override;
    void SolveVelocity(const StepContext& ctx) override;

    Vec2 m_localAnchorB;
    Vec2 m_target;
    float m_maxForce, m_frequencyHz, m_dampingRatio;

    Vec2 m_rB;
    Mat22 m_mass;
    Vec2 m_C;
    float m_gamma = 0.0f;
    Vec2 m_impulse;
};

}  // namespace kinema
