// =============================================================================
// 求解器共享的数据结构
//
// SolverBody 是 Body 中"求解器真正关心的数据"的紧凑拷贝（面向数据设计）：
// 迭代求解时所有约束都只读写这个连续数组，缓存命中率更高，
// 求解结束后再把速度写回 Body。
// =============================================================================
#pragma once

#include <vector>

#include "kinema/math/Math.h"

namespace kinema {

struct SolverBody {
    Vec2 v;               // 线速度
    float w = 0.0f;       // 角速度
    float invMass = 0.0f;
    float invI = 0.0f;
    Vec2 p;               // 质心位置
    Rot q;                // 旋转
};

struct StepContext {
    float dt = 0.0f;
    float invDt = 0.0f;
    bool warmStarting = true;
    float baumgarte = 0.2f;             // 位置误差修正系数
    float linearSlop = 0.005f;          // 允许的穿透量
    float restitutionThreshold = 1.0f;  // 相对速度低于此值时不反弹（防止静止抖动）
    std::vector<SolverBody>* bodies = nullptr;
};

}  // namespace kinema
