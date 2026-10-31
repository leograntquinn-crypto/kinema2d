// =============================================================================
// Kinema2D - 数学基础库
//
// 2D 物理引擎需要的全部数学类型都在这里：
//   Vec2      二维向量
//   Rot       旋转（缓存 sin/cos，避免反复计算三角函数）
//   Transform 刚体变换 = 平移 + 旋转
//   Mat22     2x2 矩阵（用于关节求解中的 2x2 线性方程组）
//
// 设计要点：
//   * 全部是 header-only 的值类型（POD），编译器容易内联优化；
//   * 不依赖任何第三方数学库，方便学习每一行公式的含义。
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace kinema {

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kEpsilon = 1.1920929e-7f;  // == FLT_EPSILON

// -----------------------------------------------------------------------------
// Vec2
// -----------------------------------------------------------------------------
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2 operator-() const { return {-x, -y}; }

    constexpr Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }
    constexpr Vec2& operator*=(float s) { x *= s; y *= s; return *this; }
    constexpr Vec2& operator/=(float s) { x /= s; y /= s; return *this; }

    [[nodiscard]] constexpr float LengthSquared() const { return x * x + y * y; }
    [[nodiscard]] float Length() const { return std::sqrt(x * x + y * y); }

    /// 归一化自身，返回原长度。长度过小时保持不变并返回 0。
    float Normalize() {
        const float len = Length();
        if (len < kEpsilon) return 0.0f;
        const float inv = 1.0f / len;
        x *= inv;
        y *= inv;
        return len;
    }

    [[nodiscard]] Vec2 Normalized() const {
        Vec2 v = *this;
        v.Normalize();
        return v;
    }

    [[nodiscard]] bool IsValid() const { return std::isfinite(x) && std::isfinite(y); }
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, float s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(float s, Vec2 v) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator/(Vec2 v, float s) { return {v.x / s, v.y / s}; }
constexpr bool operator==(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }
constexpr bool operator!=(Vec2 a, Vec2 b) { return !(a == b); }

/// 点积：|a||b|cosθ
constexpr float Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

/// 2D 叉积（标量）：a × b = |a||b|sinθ，正值表示 b 在 a 的逆时针方向
constexpr float Cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

/// 向量 × 标量：相当于 3D 中 (v.x, v.y, 0) × (0, 0, s)，结果是 v 顺时针旋转 90° 再乘 s
constexpr Vec2 Cross(Vec2 v, float s) { return {s * v.y, -s * v.x}; }

/// 标量 × 向量：相当于 (0, 0, s) × (v.x, v.y, 0)，结果是 v 逆时针旋转 90° 再乘 s。
/// 物理意义：角速度 ω 与半径 r 的叉积 ω × r 就是该点的线速度。
constexpr Vec2 Cross(float s, Vec2 v) { return {-s * v.y, s * v.x}; }

/// 逆时针旋转 90°
constexpr Vec2 LeftPerp(Vec2 v) { return {-v.y, v.x}; }
/// 顺时针旋转 90°
constexpr Vec2 RightPerp(Vec2 v) { return {v.y, -v.x}; }

inline float Distance(Vec2 a, Vec2 b) { return (a - b).Length(); }
constexpr float DistanceSquared(Vec2 a, Vec2 b) { return (a - b).LengthSquared(); }

constexpr Vec2 Min(Vec2 a, Vec2 b) { return {std::min(a.x, b.x), std::min(a.y, b.y)}; }
constexpr Vec2 Max(Vec2 a, Vec2 b) { return {std::max(a.x, b.x), std::max(a.y, b.y)}; }
inline Vec2 Abs(Vec2 v) { return {std::fabs(v.x), std::fabs(v.y)}; }
constexpr Vec2 Lerp(Vec2 a, Vec2 b, float t) { return a + t * (b - a); }

template <typename T>
constexpr T Clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

constexpr float DegToRad(float deg) { return deg * (kPi / 180.0f); }
constexpr float RadToDeg(float rad) { return rad * (180.0f / kPi); }

// -----------------------------------------------------------------------------
// Rot：旋转。只存 sin 和 cos，所有旋转都变成乘加运算。
// -----------------------------------------------------------------------------
struct Rot {
    float s = 0.0f;  // sin(angle)
    float c = 1.0f;  // cos(angle)

    constexpr Rot() = default;
    explicit Rot(float angle) : s(std::sin(angle)), c(std::cos(angle)) {}

    void Set(float angle) { s = std::sin(angle); c = std::cos(angle); }
    [[nodiscard]] float Angle() const { return std::atan2(s, c); }

    /// 局部 x 轴在世界坐标中的方向
    [[nodiscard]] constexpr Vec2 XAxis() const { return {c, s}; }
    [[nodiscard]] constexpr Vec2 YAxis() const { return {-s, c}; }

    /// R * v
    [[nodiscard]] constexpr Vec2 Rotate(Vec2 v) const { return {c * v.x - s * v.y, s * v.x + c * v.y}; }
    /// R^T * v（逆旋转）
    [[nodiscard]] constexpr Vec2 InvRotate(Vec2 v) const { return {c * v.x + s * v.y, -s * v.x + c * v.y}; }
};

/// q^T * r：把 r 表示到 q 的坐标系中
constexpr Rot MulT(const Rot& q, const Rot& r) {
    // [ qc qs] * [rc -rs] = [qc*rc+qs*rs  -qc*rs+qs*rc]
    // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs  qs*rs+qc*rc]
    Rot out;
    out.s = q.c * r.s - q.s * r.c;
    out.c = q.c * r.c + q.s * r.s;
    return out;
}

// -----------------------------------------------------------------------------
// Transform：局部坐标 -> 世界坐标
// -----------------------------------------------------------------------------
struct Transform {
    Vec2 p;  // 平移
    Rot q;   // 旋转

    constexpr Transform() = default;
    constexpr Transform(Vec2 position, Rot rotation) : p(position), q(rotation) {}

    /// 局部点 -> 世界点
    [[nodiscard]] constexpr Vec2 Apply(Vec2 v) const { return q.Rotate(v) + p; }
    /// 世界点 -> 局部点
    [[nodiscard]] constexpr Vec2 ApplyInv(Vec2 v) const { return q.InvRotate(v - p); }
};

/// A^-1 * B：把 B 的变换表示到 A 的局部坐标系
constexpr Transform MulT(const Transform& a, const Transform& b) {
    Transform out;
    out.q = MulT(a.q, b.q);
    out.p = a.q.InvRotate(b.p - a.p);
    return out;
}

// -----------------------------------------------------------------------------
// Mat22：列主序 2x2 矩阵
// -----------------------------------------------------------------------------
struct Mat22 {
    Vec2 ex{1.0f, 0.0f};  // 第一列
    Vec2 ey{0.0f, 1.0f};  // 第二列

    constexpr Mat22() = default;
    constexpr Mat22(Vec2 c1, Vec2 c2) : ex(c1), ey(c2) {}

    [[nodiscard]] constexpr Mat22 GetInverse() const {
        const float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
        float det = a * d - b * c;
        if (det != 0.0f) det = 1.0f / det;
        return Mat22({det * d, -det * c}, {-det * b, det * a});
    }

    /// 求解 A * x = b（比先求逆更快更稳定）
    [[nodiscard]] constexpr Vec2 Solve(Vec2 b) const {
        const float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
        float det = a11 * a22 - a12 * a21;
        if (det != 0.0f) det = 1.0f / det;
        return {det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x)};
    }
};

constexpr Vec2 operator*(const Mat22& m, Vec2 v) {
    return {m.ex.x * v.x + m.ey.x * v.y, m.ex.y * v.x + m.ey.y * v.y};
}

}  // namespace kinema
