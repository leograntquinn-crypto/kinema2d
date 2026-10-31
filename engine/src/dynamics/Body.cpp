#include "kinema/dynamics/Body.h"

#include <cassert>

namespace kinema {

const char* ToString(BodyType type) {
    switch (type) {
        case BodyType::Static: return "Static";
        case BodyType::Kinematic: return "Kinematic";
        case BodyType::Dynamic: return "Dynamic";
    }
    return "Unknown";
}

Body::Body(BodyId id, const BodyDef& def, std::unique_ptr<Shape> shape)
    : m_id(id),
      m_type(def.type),
      m_shape(std::move(shape)),
      m_position(def.position),
      m_angle(def.angle),
      m_linearVelocity(def.linearVelocity),
      m_angularVelocity(def.angularVelocity),
      m_material(def.material),
      m_linearDamping(def.linearDamping),
      m_angularDamping(def.angularDamping),
      m_gravityScale(def.gravityScale),
      m_fixedRotation(def.fixedRotation),
      m_awake(def.awake),
      m_allowSleep(def.allowSleep),
      m_categoryBits(def.categoryBits),
      m_maskBits(def.maskBits),
      m_userData(def.userData) {
    assert(m_shape != nullptr);
    if (m_type == BodyType::Static) {
        m_linearVelocity = {};
        m_angularVelocity = 0.0f;
        m_awake = false;  // 静态物体不参与"醒着"的概念
    }
    ResetMassData();
    SynchronizeTransform();
}

void Body::SetType(BodyType type) {
    if (m_type == type) return;
    m_type = type;
    ResetMassData();
    if (m_type == BodyType::Static) {
        m_linearVelocity = {};
        m_angularVelocity = 0.0f;
        m_awake = false;
    } else {
        SetAwake(true);
    }
    m_force = {};
    m_torque = 0.0f;
}

void Body::SetShape(const Shape& shape) {
    m_shape = shape.Clone();
    ResetMassData();
    SynchronizeTransform();
    if (m_type != BodyType::Static) SetAwake(true);
}

void Body::SetTransform(Vec2 position, float angle) {
    m_position = position;
    m_angle = angle;
    SynchronizeTransform();
    if (m_type != BodyType::Static) SetAwake(true);
}

void Body::SetLinearVelocity(Vec2 v) {
    if (m_type == BodyType::Static) return;
    if (Dot(v, v) > 0.0f) SetAwake(true);
    m_linearVelocity = v;
}

void Body::SetAngularVelocity(float w) {
    if (m_type == BodyType::Static) return;
    if (w != 0.0f) SetAwake(true);
    m_angularVelocity = w;
}

void Body::SetDensity(float density) {
    m_material.density = std::max(density, 0.0f);
    ResetMassData();
}

void Body::SetFixedRotation(bool flag) {
    if (m_fixedRotation == flag) return;
    m_fixedRotation = flag;
    m_angularVelocity = 0.0f;
    ResetMassData();
}

void Body::SetAwake(bool flag) {
    if (m_type == BodyType::Static) return;
    if (flag) {
        m_awake = true;
        m_sleepTime = 0.0f;
    } else {
        m_awake = false;
        m_sleepTime = 0.0f;
        m_linearVelocity = {};
        m_angularVelocity = 0.0f;
        m_force = {};
        m_torque = 0.0f;
    }
}

void Body::SetSleepingAllowed(bool flag) {
    m_allowSleep = flag;
    if (!flag) SetAwake(true);
}

void Body::ApplyForce(Vec2 force, Vec2 worldPoint, bool wake) {
    if (m_type != BodyType::Dynamic) return;
    if (wake && !m_awake) SetAwake(true);
    if (!m_awake) return;
    m_force += force;
    m_torque += Cross(worldPoint - m_position, force);
}

void Body::ApplyForceToCenter(Vec2 force, bool wake) {
    if (m_type != BodyType::Dynamic) return;
    if (wake && !m_awake) SetAwake(true);
    if (!m_awake) return;
    m_force += force;
}

void Body::ApplyTorque(float torque, bool wake) {
    if (m_type != BodyType::Dynamic) return;
    if (wake && !m_awake) SetAwake(true);
    if (!m_awake) return;
    m_torque += torque;
}

void Body::ApplyLinearImpulse(Vec2 impulse, Vec2 worldPoint, bool wake) {
    if (m_type != BodyType::Dynamic) return;
    if (wake && !m_awake) SetAwake(true);
    if (!m_awake) return;
    m_linearVelocity += m_invMass * impulse;
    m_angularVelocity += m_invInertia * Cross(worldPoint - m_position, impulse);
}

void Body::ApplyLinearImpulseToCenter(Vec2 impulse, bool wake) {
    if (m_type != BodyType::Dynamic) return;
    if (wake && !m_awake) SetAwake(true);
    if (!m_awake) return;
    m_linearVelocity += m_invMass * impulse;
}

void Body::ApplyAngularImpulse(float impulse, bool wake) {
    if (m_type != BodyType::Dynamic) return;
    if (wake && !m_awake) SetAwake(true);
    if (!m_awake) return;
    m_angularVelocity += m_invInertia * impulse;
}

float Body::GetKineticEnergy() const {
    return 0.5f * m_mass * m_linearVelocity.LengthSquared() + 0.5f * m_inertia * m_angularVelocity * m_angularVelocity;
}

void Body::ResetMassData() {
    m_mass = m_invMass = m_inertia = m_invInertia = 0.0f;

    if (m_type != BodyType::Dynamic) return;  // 静态和运动学物体：质量视为无穷大

    const MassData md = m_shape->ComputeMass(m_material.density);
    m_mass = md.mass > 0.0f ? md.mass : 1.0f;
    m_invMass = 1.0f / m_mass;

    if (!m_fixedRotation && md.inertia > 0.0f) {
        m_inertia = md.inertia;
        m_invInertia = 1.0f / m_inertia;
    }
}

void Body::SynchronizeTransform() {
    m_xf.p = m_position;
    m_xf.q.Set(m_angle);
    m_aabb = m_shape->ComputeAABB(m_xf);
}

}  // namespace kinema
