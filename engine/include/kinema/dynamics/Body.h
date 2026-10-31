// =============================================================================
// 刚体（Rigid Body）
//
// Body 持有：
//   * 形状（几何）
//   * 位置 / 角度（状态）
//   * 线速度 / 角速度（状态的导数）
//   * 质量 / 转动惯量（由形状 + 密度算出）
//   * 材质（摩擦、弹性）
//
// 三种类型：
//   Static    : 质量无穷大，永不移动（地面、墙壁）
//   Kinematic : 质量无穷大，但按照设定的速度运动，不受力影响（电梯、传送带）
//   Dynamic   : 正常受力、受碰撞影响的物体
//
// 生命周期：只能通过 World::CreateBody / World::DestroyBody 创建与销毁。
// 外部代码推荐持有 BodyId，而不是长期持有 Body*（物体可能已被删除）。
// =============================================================================
#pragma once

#include <cstdint>
#include <memory>

#include "kinema/collision/Shape.h"
#include "kinema/math/AABB.h"
#include "kinema/math/Math.h"

namespace kinema {

using BodyId = uint32_t;
inline constexpr BodyId kInvalidBodyId = 0;

enum class BodyType : uint8_t {
    Static = 0,
    Kinematic = 1,
    Dynamic = 2,
};

const char* ToString(BodyType type);

struct Material {
    float density = 1.0f;      // kg/m²
    float friction = 0.5f;     // 库仑摩擦系数
    float restitution = 0.0f;  // 弹性系数：0 = 完全非弹性，1 = 完全弹性
};

struct BodyDef {
    BodyType type = BodyType::Dynamic;
    Vec2 position;
    float angle = 0.0f;
    Vec2 linearVelocity;
    float angularVelocity = 0.0f;
    float linearDamping = 0.0f;
    float angularDamping = 0.01f;
    float gravityScale = 1.0f;
    bool fixedRotation = false;
    bool awake = true;
    bool allowSleep = true;
    Material material;

    /// 碰撞过滤：(A.category & B.mask) && (B.category & A.mask) 时才碰撞
    uint16_t categoryBits = 0x0001;
    uint16_t maskBits = 0xFFFF;

    /// 用户数据（引擎不解释它；示例程序用它存放颜色）
    uint64_t userData = 0;
};

class Body {
public:
    Body(BodyId id, const BodyDef& def, std::unique_ptr<Shape> shape);

    Body(const Body&) = delete;
    Body& operator=(const Body&) = delete;

    // ---- 基本信息 -----------------------------------------------------------
    [[nodiscard]] BodyId GetId() const { return m_id; }
    [[nodiscard]] BodyType GetType() const { return m_type; }
    void SetType(BodyType type);

    [[nodiscard]] const Shape& GetShape() const { return *m_shape; }
    [[nodiscard]] ShapeType GetShapeType() const { return m_shape->GetType(); }
    /// 替换形状（会重新计算质量）
    void SetShape(const Shape& shape);

    // ---- 位姿 ---------------------------------------------------------------
    [[nodiscard]] Vec2 GetPosition() const { return m_position; }
    [[nodiscard]] float GetAngle() const { return m_angle; }
    [[nodiscard]] const Transform& GetTransform() const { return m_xf; }
    void SetTransform(Vec2 position, float angle);

    // ---- 速度 ---------------------------------------------------------------
    [[nodiscard]] Vec2 GetLinearVelocity() const { return m_linearVelocity; }
    [[nodiscard]] float GetAngularVelocity() const { return m_angularVelocity; }
    void SetLinearVelocity(Vec2 v);
    void SetAngularVelocity(float w);

    // ---- 质量 ---------------------------------------------------------------
    [[nodiscard]] float GetMass() const { return m_mass; }
    [[nodiscard]] float GetInvMass() const { return m_invMass; }
    [[nodiscard]] float GetInertia() const { return m_inertia; }
    [[nodiscard]] float GetInvInertia() const { return m_invInertia; }

    // ---- 材质与参数 ---------------------------------------------------------
    [[nodiscard]] const Material& GetMaterial() const { return m_material; }
    void SetDensity(float density);
    void SetFriction(float f) { m_material.friction = f; }
    void SetRestitution(float r) { m_material.restitution = r; }

    [[nodiscard]] float GetLinearDamping() const { return m_linearDamping; }
    [[nodiscard]] float GetAngularDamping() const { return m_angularDamping; }
    void SetLinearDamping(float d) { m_linearDamping = d; }
    void SetAngularDamping(float d) { m_angularDamping = d; }

    [[nodiscard]] float GetGravityScale() const { return m_gravityScale; }
    void SetGravityScale(float s) { m_gravityScale = s; }

    [[nodiscard]] bool IsFixedRotation() const { return m_fixedRotation; }
    void SetFixedRotation(bool flag);

    // ---- 休眠 ---------------------------------------------------------------
    [[nodiscard]] bool IsAwake() const { return m_awake; }
    void SetAwake(bool flag);
    [[nodiscard]] bool IsSleepingAllowed() const { return m_allowSleep; }
    void SetSleepingAllowed(bool flag);
    [[nodiscard]] float GetSleepTime() const { return m_sleepTime; }

    // ---- 碰撞过滤 -----------------------------------------------------------
    [[nodiscard]] uint16_t GetCategoryBits() const { return m_categoryBits; }
    [[nodiscard]] uint16_t GetMaskBits() const { return m_maskBits; }
    void SetFilter(uint16_t category, uint16_t mask) { m_categoryBits = category; m_maskBits = mask; }
    [[nodiscard]] bool ShouldCollide(const Body& other) const {
        return (m_categoryBits & other.m_maskBits) != 0 && (other.m_categoryBits & m_maskBits) != 0;
    }

    [[nodiscard]] uint64_t GetUserData() const { return m_userData; }
    void SetUserData(uint64_t data) { m_userData = data; }

    // ---- 施加力 / 冲量 ------------------------------------------------------
    /// 力在下一次 Step 中生效后被清零
    void ApplyForce(Vec2 force, Vec2 worldPoint, bool wake = true);
    void ApplyForceToCenter(Vec2 force, bool wake = true);
    void ApplyTorque(float torque, bool wake = true);
    /// 冲量立即改变速度：Δv = J / m，Δω = (r × J) / I
    void ApplyLinearImpulse(Vec2 impulse, Vec2 worldPoint, bool wake = true);
    void ApplyLinearImpulseToCenter(Vec2 impulse, bool wake = true);
    void ApplyAngularImpulse(float impulse, bool wake = true);

    // ---- 坐标变换辅助 -------------------------------------------------------
    [[nodiscard]] Vec2 GetWorldPoint(Vec2 local) const { return m_xf.Apply(local); }
    [[nodiscard]] Vec2 GetLocalPoint(Vec2 world) const { return m_xf.ApplyInv(world); }
    [[nodiscard]] Vec2 GetWorldVector(Vec2 local) const { return m_xf.q.Rotate(local); }
    [[nodiscard]] Vec2 GetLocalVector(Vec2 world) const { return m_xf.q.InvRotate(world); }
    /// 刚体上某个世界点的速度：v + ω × r
    [[nodiscard]] Vec2 GetLinearVelocityAt(Vec2 worldPoint) const {
        return m_linearVelocity + Cross(m_angularVelocity, worldPoint - m_position);
    }

    [[nodiscard]] const AABB& GetAABB() const { return m_aabb; }
    [[nodiscard]] float GetKineticEnergy() const;

    [[nodiscard]] bool IsStatic() const { return m_type == BodyType::Static; }
    [[nodiscard]] bool IsDynamic() const { return m_type == BodyType::Dynamic; }

private:
    friend class World;
    friend class ContactSolver;
    friend class Joint;

    void ResetMassData();
    void SynchronizeTransform();

    BodyId m_id;
    BodyType m_type;
    std::unique_ptr<Shape> m_shape;

    Vec2 m_position;
    float m_angle;
    Transform m_xf;
    AABB m_aabb;

    Vec2 m_linearVelocity;
    float m_angularVelocity;
    Vec2 m_force;
    float m_torque = 0.0f;

    float m_mass = 0.0f, m_invMass = 0.0f;
    float m_inertia = 0.0f, m_invInertia = 0.0f;

    Material m_material;
    float m_linearDamping;
    float m_angularDamping;
    float m_gravityScale;
    bool m_fixedRotation;

    bool m_awake;
    bool m_allowSleep;
    float m_sleepTime = 0.0f;

    uint16_t m_categoryBits;
    uint16_t m_maskBits;
    uint64_t m_userData;

    // World 每步使用的临时数据
    int m_solverIndex = -1;
};

}  // namespace kinema
