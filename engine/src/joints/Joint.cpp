#include "kinema/joints/Joint.h"

#include "kinema/collision/Collision.h"

#include <cassert>

namespace kinema {

const char* ToString(JointType type) {
    switch (type) {
        case JointType::Distance: return "Distance";
        case JointType::Revolute: return "Revolute";
        case JointType::Mouse: return "Mouse";
    }
    return "Unknown";
}

namespace {

/// 软约束系数（参考 Erin Catto "Soft Constraints", GDC 2011）
///   弹簧刚度 k = m ω²，阻尼 c = 2 m ζ ω
///   gamma = 1 / (h (c + h k))   —— 约束"柔软度"
///   beta  = h k gamma           —— 位置误差反馈系数
void ComputeSoftParams(float mass, float hz, float zeta, float h, float& gamma, float& beta) {
    const float omega = 2.0f * kPi * hz;
    const float c = 2.0f * mass * zeta * omega;
    const float k = mass * omega * omega;
    gamma = h * (c + h * k);
    gamma = gamma > 0.0f ? 1.0f / gamma : 0.0f;
    beta = h * k * gamma;
}

}  // namespace

Joint::Joint(JointId id, JointType type, const JointDef& def)
    : m_id(id),
      m_type(type),
      m_bodyA(def.bodyA),
      m_bodyB(def.bodyB),
      m_collideConnected(def.collideConnected),
      m_userData(def.userData) {}

// =============================================================================
// DistanceJoint
//   约束：C = |pB - pA| - L = 0
//   速度：Cdot = u · (vB + ωB × rB - vA - ωA × rA)
// =============================================================================
void DistanceJointDef::Initialize(Body* a, Body* b, Vec2 worldAnchorA, Vec2 worldAnchorB) {
    bodyA = a;
    bodyB = b;
    localAnchorA = a->GetLocalPoint(worldAnchorA);
    localAnchorB = b->GetLocalPoint(worldAnchorB);
    length = std::max(Distance(worldAnchorA, worldAnchorB), 0.01f);
}

DistanceJoint::DistanceJoint(JointId id, const DistanceJointDef& def)
    : Joint(id, JointType::Distance, def),
      m_localAnchorA(def.localAnchorA),
      m_localAnchorB(def.localAnchorB),
      m_length(std::max(def.length, 0.01f)),
      m_frequencyHz(def.frequencyHz),
      m_dampingRatio(def.dampingRatio) {
    assert(m_bodyA && m_bodyB);
}

void DistanceJoint::PrepareAndWarmStart(const StepContext& ctx) {
    SolverBody& A = Solver(ctx, m_bodyA);
    SolverBody& B = Solver(ctx, m_bodyB);

    m_rA = A.q.Rotate(m_localAnchorA);
    m_rB = B.q.Rotate(m_localAnchorB);
    m_u = B.p + m_rB - A.p - m_rA;

    const float length = m_u.Length();
    if (length > kLinearSlop) {
        m_u *= 1.0f / length;
    } else {
        m_u = {0.0f, 0.0f};
    }

    const float crA = Cross(m_rA, m_u);
    const float crB = Cross(m_rB, m_u);
    float invMass = A.invMass + A.invI * crA * crA + B.invMass + B.invI * crB * crB;
    m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

    const float C = length - m_length;
    if (m_frequencyHz > 0.0f) {
        // 弹簧模式
        float beta = 0.0f;
        ComputeSoftParams(m_mass, m_frequencyHz, m_dampingRatio, ctx.dt, m_gamma, beta);
        m_bias = C * beta;
        invMass += m_gamma;
        m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
    } else {
        // 刚性模式：Baumgarte 稳定化
        m_gamma = 0.0f;
        m_bias = ctx.baumgarte * ctx.invDt * C;
    }

    if (ctx.warmStarting) {
        const Vec2 P = m_impulse * m_u;
        A.v -= A.invMass * P;
        A.w -= A.invI * Cross(m_rA, P);
        B.v += B.invMass * P;
        B.w += B.invI * Cross(m_rB, P);
    } else {
        m_impulse = 0.0f;
    }
}

void DistanceJoint::SolveVelocity(const StepContext& ctx) {
    SolverBody& A = Solver(ctx, m_bodyA);
    SolverBody& B = Solver(ctx, m_bodyB);

    const Vec2 vpA = A.v + Cross(A.w, m_rA);
    const Vec2 vpB = B.v + Cross(B.w, m_rB);
    const float Cdot = Dot(m_u, vpB - vpA);

    const float impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
    m_impulse += impulse;

    const Vec2 P = impulse * m_u;
    A.v -= A.invMass * P;
    A.w -= A.invI * Cross(m_rA, P);
    B.v += B.invMass * P;
    B.w += B.invI * Cross(m_rB, P);
}

// =============================================================================
// RevoluteJoint
//   约束：C = pB + rB - pA - rA = 0（二维向量约束）
//   有效质量是 2x2 矩阵 K，求解 K * λ = -Cdot + bias
// =============================================================================
void RevoluteJointDef::Initialize(Body* a, Body* b, Vec2 worldAnchor) {
    bodyA = a;
    bodyB = b;
    localAnchorA = a->GetLocalPoint(worldAnchor);
    localAnchorB = b->GetLocalPoint(worldAnchor);
}

RevoluteJoint::RevoluteJoint(JointId id, const RevoluteJointDef& def)
    : Joint(id, JointType::Revolute, def),
      m_localAnchorA(def.localAnchorA),
      m_localAnchorB(def.localAnchorB),
      m_enableMotor(def.enableMotor),
      m_motorSpeed(def.motorSpeed),
      m_maxMotorTorque(def.maxMotorTorque) {
    assert(m_bodyA && m_bodyB);
}

void RevoluteJoint::EnableMotor(bool flag) {
    m_enableMotor = flag;
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
}

void RevoluteJoint::SetMotorSpeed(float speed) {
    m_motorSpeed = speed;
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
}

void RevoluteJoint::PrepareAndWarmStart(const StepContext& ctx) {
    SolverBody& A = Solver(ctx, m_bodyA);
    SolverBody& B = Solver(ctx, m_bodyB);

    m_rA = A.q.Rotate(m_localAnchorA);
    m_rB = B.q.Rotate(m_localAnchorB);

    const float mA = A.invMass, mB = B.invMass;
    const float iA = A.invI, iB = B.invI;

    // K = [mA+mB+iA*rAy²+iB*rBy²,   -iA*rAx*rAy-iB*rBx*rBy]
    //     [-iA*rAx*rAy-iB*rBx*rBy,  mA+mB+iA*rAx²+iB*rBx² ]
    Mat22 K;
    K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
    K.ey.x = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
    K.ex.y = K.ey.x;
    K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;
    m_mass = K.GetInverse();

    m_motorMass = iA + iB;
    if (m_motorMass > 0.0f) m_motorMass = 1.0f / m_motorMass;

    const Vec2 C = B.p + m_rB - A.p - m_rA;
    m_bias = -ctx.baumgarte * ctx.invDt * C;

    if (!m_enableMotor) m_motorImpulse = 0.0f;

    if (ctx.warmStarting) {
        const Vec2 P = m_impulse;
        A.v -= mA * P;
        A.w -= iA * (Cross(m_rA, P) + m_motorImpulse);
        B.v += mB * P;
        B.w += iB * (Cross(m_rB, P) + m_motorImpulse);
    } else {
        m_impulse = {};
        m_motorImpulse = 0.0f;
    }
}

void RevoluteJoint::SolveVelocity(const StepContext& ctx) {
    SolverBody& A = Solver(ctx, m_bodyA);
    SolverBody& B = Solver(ctx, m_bodyB);

    // 马达：目标是 ωB - ωA = motorSpeed，力矩受限
    if (m_enableMotor) {
        const float Cdot = B.w - A.w - m_motorSpeed;
        float impulse = -m_motorMass * Cdot;
        const float old = m_motorImpulse;
        const float maxImpulse = ctx.dt * m_maxMotorTorque;
        m_motorImpulse = Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = m_motorImpulse - old;
        A.w -= A.invI * impulse;
        B.w += B.invI * impulse;
    }

    // 点约束
    const Vec2 Cdot = B.v + Cross(B.w, m_rB) - A.v - Cross(A.w, m_rA);
    const Vec2 impulse = m_mass * (m_bias - Cdot);
    m_impulse += impulse;

    A.v -= A.invMass * impulse;
    A.w -= A.invI * Cross(m_rA, impulse);
    B.v += B.invMass * impulse;
    B.w += B.invI * Cross(m_rB, impulse);
}

// =============================================================================
// MouseJoint：软点约束 C = pB + rB - target
// =============================================================================
MouseJoint::MouseJoint(JointId id, const MouseJointDef& def)
    : Joint(id, JointType::Mouse, def),
      m_localAnchorB(def.bodyB->GetLocalPoint(def.target)),
      m_target(def.target),
      m_maxForce(def.maxForce),
      m_frequencyHz(def.frequencyHz),
      m_dampingRatio(def.dampingRatio) {
    assert(m_bodyB);
}

void MouseJoint::SetTarget(Vec2 target) {
    if (target != m_target) m_bodyB->SetAwake(true);
    m_target = target;
}

void MouseJoint::PrepareAndWarmStart(const StepContext& ctx) {
    SolverBody& B = Solver(ctx, m_bodyB);

    const float mass = m_bodyB->GetMass();
    float beta = 0.0f;
    ComputeSoftParams(mass, m_frequencyHz, m_dampingRatio, ctx.dt, m_gamma, beta);

    m_rB = B.q.Rotate(m_localAnchorB);

    const float mB = B.invMass, iB = B.invI;
    Mat22 K;
    K.ex.x = mB + iB * m_rB.y * m_rB.y + m_gamma;
    K.ex.y = -iB * m_rB.x * m_rB.y;
    K.ey.x = K.ex.y;
    K.ey.y = mB + iB * m_rB.x * m_rB.x + m_gamma;
    m_mass = K.GetInverse();

    m_C = beta * (B.p + m_rB - m_target);

    // 额外的角阻尼，拖拽时物体不会疯狂旋转
    B.w *= 0.98f;

    if (ctx.warmStarting) {
        B.v += mB * m_impulse;
        B.w += iB * Cross(m_rB, m_impulse);
    } else {
        m_impulse = {};
    }
}

void MouseJoint::SolveVelocity(const StepContext& ctx) {
    SolverBody& B = Solver(ctx, m_bodyB);

    const Vec2 Cdot = B.v + Cross(B.w, m_rB);
    Vec2 impulse = m_mass * (-(Cdot + m_C + m_gamma * m_impulse));

    const Vec2 old = m_impulse;
    m_impulse += impulse;
    const float maxImpulse = ctx.dt * m_maxForce;
    if (m_impulse.LengthSquared() > maxImpulse * maxImpulse) m_impulse *= maxImpulse / m_impulse.Length();
    impulse = m_impulse - old;

    B.v += B.invMass * impulse;
    B.w += B.invI * Cross(m_rB, impulse);
}

}  // namespace kinema
