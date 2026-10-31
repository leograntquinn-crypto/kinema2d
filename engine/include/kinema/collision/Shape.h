// =============================================================================
// 形状（Shape）
//
// 形状只描述"几何"，不关心速度、质量等物理状态（那是 Body 的事）。
// 引擎支持两种基本形状：
//   * CircleShape   圆
//   * PolygonShape  凸多边形（最多 kMaxPolygonVertices 个顶点）
//
// 约定：形状的局部坐标原点 == 质心。
//   PolygonShape::Set() 会自动把顶点平移到以质心为原点的坐标系，
//   这样 Body 的 position 就是质心，动力学公式会简单很多。
// =============================================================================
#pragma once

#include <array>
#include <memory>
#include <vector>

#include "kinema/math/AABB.h"
#include "kinema/math/Math.h"

namespace kinema {

inline constexpr int kMaxPolygonVertices = 12;

enum class ShapeType : uint8_t {
    Circle = 0,
    Polygon = 1,
    Count
};

const char* ToString(ShapeType type);

/// 质量属性
struct MassData {
    float mass = 0.0f;     // kg
    Vec2 center;           // 质心（局部坐标）
    float inertia = 0.0f;  // 绕质心的转动惯量 kg·m²
};

/// 射线检测输入：从 p1 到 p2 的线段，只检测 [0, maxFraction] 范围
struct RayCastInput {
    Vec2 p1;
    Vec2 p2;
    float maxFraction = 1.0f;
};

/// 射线检测输出：命中点 = p1 + fraction * (p2 - p1)
struct RayCastOutput {
    Vec2 normal;
    float fraction = 0.0f;
};

// -----------------------------------------------------------------------------
// Shape 抽象基类
// -----------------------------------------------------------------------------
class Shape {
public:
    virtual ~Shape() = default;

    Shape(const Shape&) = default;
    Shape& operator=(const Shape&) = default;

    [[nodiscard]] ShapeType GetType() const { return m_type; }

    /// 深拷贝（原型模式）：World::CreateBody 会克隆一份传入的形状
    [[nodiscard]] virtual std::unique_ptr<Shape> Clone() const = 0;

    /// 根据密度计算质量属性
    [[nodiscard]] virtual MassData ComputeMass(float density) const = 0;

    /// 计算在给定变换下的世界 AABB
    [[nodiscard]] virtual AABB ComputeAABB(const Transform& xf) const = 0;

    /// 点是否在形状内部（用于鼠标拾取）
    [[nodiscard]] virtual bool TestPoint(const Transform& xf, Vec2 p) const = 0;

    /// 射线检测
    virtual bool RayCast(const RayCastInput& input, const Transform& xf, RayCastOutput& output) const = 0;

protected:
    explicit Shape(ShapeType type) : m_type(type) {}

private:
    ShapeType m_type;
};

// -----------------------------------------------------------------------------
// 圆
// -----------------------------------------------------------------------------
class CircleShape final : public Shape {
public:
    explicit CircleShape(float radius = 0.5f) : Shape(ShapeType::Circle), m_radius(radius) {}

    [[nodiscard]] float GetRadius() const { return m_radius; }
    void SetRadius(float r) { m_radius = r; }

    [[nodiscard]] std::unique_ptr<Shape> Clone() const override;
    [[nodiscard]] MassData ComputeMass(float density) const override;
    [[nodiscard]] AABB ComputeAABB(const Transform& xf) const override;
    [[nodiscard]] bool TestPoint(const Transform& xf, Vec2 p) const override;
    bool RayCast(const RayCastInput& input, const Transform& xf, RayCastOutput& output) const override;

private:
    float m_radius;
};

// -----------------------------------------------------------------------------
// 凸多边形（逆时针 CCW 顶点顺序）
// -----------------------------------------------------------------------------
class PolygonShape final : public Shape {
public:
    PolygonShape() : Shape(ShapeType::Polygon) {}

    /// 由任意点集构造：内部计算凸包、去重、保证 CCW 并把质心移到原点。
    /// 返回 false 表示点集退化（共线或点数不足）。
    bool Set(const std::vector<Vec2>& points);

    /// 构造以原点为中心的矩形
    void SetAsBox(float halfWidth, float halfHeight);

    /// 正多边形
    void SetAsRegular(int sides, float radius);

    static PolygonShape MakeBox(float halfWidth, float halfHeight);
    static PolygonShape MakeRegular(int sides, float radius);

    [[nodiscard]] int GetVertexCount() const { return m_count; }
    [[nodiscard]] Vec2 GetVertex(int i) const { return m_vertices[static_cast<size_t>(i)]; }
    [[nodiscard]] Vec2 GetNormal(int i) const { return m_normals[static_cast<size_t>(i)]; }
    [[nodiscard]] const Vec2* GetVertices() const { return m_vertices.data(); }
    [[nodiscard]] const Vec2* GetNormals() const { return m_normals.data(); }

    /// Set() 时为了把质心放在原点而做的平移量（原始坐标 = 现坐标 + offset）
    [[nodiscard]] Vec2 GetCentroidOffset() const { return m_centroidOffset; }

    [[nodiscard]] std::unique_ptr<Shape> Clone() const override;
    [[nodiscard]] MassData ComputeMass(float density) const override;
    [[nodiscard]] AABB ComputeAABB(const Transform& xf) const override;
    [[nodiscard]] bool TestPoint(const Transform& xf, Vec2 p) const override;
    bool RayCast(const RayCastInput& input, const Transform& xf, RayCastOutput& output) const override;

private:
    std::array<Vec2, kMaxPolygonVertices> m_vertices{};
    std::array<Vec2, kMaxPolygonVertices> m_normals{};  // 每条边的外法线
    int m_count = 0;
    Vec2 m_centroidOffset;
};

/// 计算点集的凸包（Andrew 单调链算法），结果为 CCW 顺序。
std::vector<Vec2> ComputeConvexHull(std::vector<Vec2> points);

}  // namespace kinema
