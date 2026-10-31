// =============================================================================
// 接触（Contact）与接触求解器（ContactSolver）
//
// Contact：一对 AABB 重叠的物体。它持有窄阶段生成的 Manifold，
//          并且在帧与帧之间保留下来，从而支持"热启动"（warm starting）。
//
// ContactSolver：顺序冲量法（Sequential Impulses，Erin Catto, GDC 2006）
//   对每个接触点反复施加冲量，使得：
//     1. 法向相对速度 >= 0（不穿透）且法向冲量 >= 0（只能推不能拉）
//     2. 切向摩擦冲量 |Pt| <= μ * Pn（库仑摩擦锥）
//   迭代多次后收敛到近似的全局解。
//
//   对于有两个接触点的流形（例如箱子平放在地面上），逐点求解会导致两点之间
//   互相"拉锯"，高堆叠会左右摇晃。因此对两点流形使用"块求解器"：
//   把两个法向约束写成 2x2 的线性互补问题（LCP），枚举 4 种情况精确求解。
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "kinema/collision/Collision.h"
#include "kinema/dynamics/Body.h"
#include "kinema/dynamics/TimeStep.h"

namespace kinema {

class Contact {
public:
    Contact(Body* a, Body* b);

    [[nodiscard]] Body* GetBodyA() const { return m_bodyA; }
    [[nodiscard]] Body* GetBodyB() const { return m_bodyB; }
    [[nodiscard]] const Manifold& GetManifold() const { return m_manifold; }
    [[nodiscard]] bool IsTouching() const { return m_touching; }
    [[nodiscard]] float GetFriction() const { return m_friction; }
    [[nodiscard]] float GetRestitution() const { return m_restitution; }

    /// 重新运行窄阶段，并把上一帧匹配的累积冲量拷贝过来（热启动）
    void Update(bool warmStarting);

    /// 摩擦混合：几何平均，任一表面光滑则整体光滑
    static float MixFriction(float a, float b) { return std::sqrt(a * b); }
    /// 弹性混合：取最大值，弹力球撞到任何东西都会弹
    static float MixRestitution(float a, float b) { return std::max(a, b); }

private:
    friend class World;
    friend class ContactSolver;

    Body* m_bodyA;
    Body* m_bodyB;
    Manifold m_manifold;
    float m_friction;
    float m_restitution;
    bool m_touching = false;
    uint32_t m_lastSeenStep = 0;  // 宽阶段最后一次报告该对的步数

    // 块求解器（两个接触点同时求解）所需的数据
    Mat22 m_K;                // 2x2 有效质量矩阵 K
    Mat22 m_normalMassBlock;  // K⁻¹
    bool m_blockSolve = false;
};

class ContactSolver {
public:
    ContactSolver(const StepContext& ctx, std::vector<Contact*>& contacts);

    /// 预计算有效质量、偏置速度，并施加热启动冲量
    void PrepareAndWarmStart();

    /// 一次速度迭代
    void SolveVelocity();

private:
    static void SolveBlock(Contact& c, SolverBody& A, SolverBody& B);

    const StepContext& m_ctx;
    std::vector<Contact*>& m_contacts;
};

}  // namespace kinema
