// =============================================================================
// 窄阶段碰撞检测（Narrow Phase）
//
// 输入：两个形状及其世界变换
// 输出：接触流形（Manifold）—— 最多 2 个接触点 + 一个公共法线
//
// 支持的组合（A 的类型 >= B 的类型，World 创建接触时会保证这个顺序）：
//   Circle  vs Circle   : 圆心距离
//   Polygon vs Circle   : 最近特征（边或顶点）
//   Polygon vs Polygon  : 分离轴定理 SAT + 参考面/入射面裁剪（Sutherland-Hodgman）
// =============================================================================
#pragma once

#include <cstdint>

#include "kinema/collision/Shape.h"

namespace kinema {

/// 接触特征 ID：标记接触点来自哪个顶点/边，用于在帧之间匹配接触点（热启动）
struct ContactFeature {
    enum Type : uint8_t { Vertex = 0, Face = 1 };
    uint8_t indexA = 0;
    uint8_t indexB = 0;
    uint8_t typeA = 0;
    uint8_t typeB = 0;

    [[nodiscard]] uint32_t Key() const {
        return static_cast<uint32_t>(indexA) | (static_cast<uint32_t>(indexB) << 8) |
               (static_cast<uint32_t>(typeA) << 16) | (static_cast<uint32_t>(typeB) << 24);
    }
};

/// 单个接触点
struct ManifoldPoint {
    Vec2 point;               // 世界坐标接触点（两表面中点）
    float separation = 0.0f;  // 分离距离，负数表示穿透深度
    uint32_t id = 0;          // ContactFeature::Key()

    // ---- 以下为求解器状态（累积冲量会跨帧保存用于热启动） ----
    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
    float normalMass = 0.0f;
    float tangentMass = 0.0f;
    float velocityBias = 0.0f;
    Vec2 rA;  // 质心 A 指向接触点
    Vec2 rB;  // 质心 B 指向接触点
};

/// 接触流形
struct Manifold {
    Vec2 normal;  // 世界坐标，从 A 指向 B
    ManifoldPoint points[2];
    int pointCount = 0;
};

void CollideCircles(Manifold& m, const CircleShape& a, const Transform& xfA, const CircleShape& b,
                    const Transform& xfB);

void CollidePolygonAndCircle(Manifold& m, const PolygonShape& a, const Transform& xfA, const CircleShape& b,
                             const Transform& xfB);

void CollidePolygons(Manifold& m, const PolygonShape& a, const Transform& xfA, const PolygonShape& b,
                     const Transform& xfB);

/// 根据形状类型分派到具体的碰撞函数。要求 a.GetType() >= b.GetType()。
void Collide(Manifold& m, const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB);

/// 宽容度：在这个距离内认为"接触"，用于提高堆叠稳定性（单位：米）
inline constexpr float kLinearSlop = 0.005f;

}  // namespace kinema
