// =============================================================================
// AABB：轴对齐包围盒（Axis-Aligned Bounding Box）
//
// 宽阶段（broad phase）用它做快速的"可能相交"判断。
// 两个 AABB 是否重叠只需要 4 次比较，比精确的形状测试便宜得多。
// =============================================================================
#pragma once

#include "kinema/math/Math.h"

namespace kinema {

struct AABB {
    Vec2 lower;  // 左下角
    Vec2 upper;  // 右上角

    constexpr AABB() = default;
    constexpr AABB(Vec2 lo, Vec2 hi) : lower(lo), upper(hi) {}

    [[nodiscard]] constexpr Vec2 Center() const { return 0.5f * (lower + upper); }
    [[nodiscard]] constexpr Vec2 Extents() const { return 0.5f * (upper - lower); }
    [[nodiscard]] constexpr float Width() const { return upper.x - lower.x; }
    [[nodiscard]] constexpr float Height() const { return upper.y - lower.y; }
    [[nodiscard]] constexpr float Perimeter() const { return 2.0f * (Width() + Height()); }

    [[nodiscard]] constexpr bool Contains(Vec2 p) const {
        return p.x >= lower.x && p.x <= upper.x && p.y >= lower.y && p.y <= upper.y;
    }

    /// 向四周扩大 margin
    [[nodiscard]] constexpr AABB Inflated(float margin) const {
        return {lower - Vec2{margin, margin}, upper + Vec2{margin, margin}};
    }

    [[nodiscard]] static constexpr AABB Combine(const AABB& a, const AABB& b) {
        return {Min(a.lower, b.lower), Max(a.upper, b.upper)};
    }
};

/// 分离轴定理在 AABB 上的特例：任意一个轴上分离 => 不相交
constexpr bool Overlaps(const AABB& a, const AABB& b) {
    if (a.upper.x < b.lower.x || b.upper.x < a.lower.x) return false;
    if (a.upper.y < b.lower.y || b.upper.y < a.lower.y) return false;
    return true;
}

}  // namespace kinema
