#include "kinema/collision/Shape.h"

#include <algorithm>
#include <cassert>

namespace kinema {

const char* ToString(ShapeType type) {
    switch (type) {
        case ShapeType::Circle: return "Circle";
        case ShapeType::Polygon: return "Polygon";
        default: return "Unknown";
    }
}

// =============================================================================
// CircleShape
// =============================================================================
std::unique_ptr<Shape> CircleShape::Clone() const { return std::make_unique<CircleShape>(*this); }

MassData CircleShape::ComputeMass(float density) const {
    MassData md;
    md.mass = density * kPi * m_radius * m_radius;
    md.center = {0.0f, 0.0f};
    // 实心圆盘绕中心：I = 1/2 * m * r²
    md.inertia = 0.5f * md.mass * m_radius * m_radius;
    return md;
}

AABB CircleShape::ComputeAABB(const Transform& xf) const {
    const Vec2 r{m_radius, m_radius};
    return {xf.p - r, xf.p + r};
}

bool CircleShape::TestPoint(const Transform& xf, Vec2 p) const {
    return DistanceSquared(xf.p, p) <= m_radius * m_radius;
}

bool CircleShape::RayCast(const RayCastInput& input, const Transform& xf, RayCastOutput& output) const {
    // 解方程 |p1 + t*d - c|² = r²，即 t² (d·d) + 2t (s·d) + (s·s - r²) = 0
    const Vec2 s = input.p1 - xf.p;
    const float b = Dot(s, s) - m_radius * m_radius;
    const Vec2 d = input.p2 - input.p1;
    const float c = Dot(s, d);
    const float rr = Dot(d, d);
    const float sigma = c * c - rr * b;

    if (sigma < 0.0f || rr < kEpsilon) return false;

    float a = -(c + std::sqrt(sigma));  // 取较小的根（第一次进入圆）
    if (0.0f <= a && a <= input.maxFraction * rr) {
        a /= rr;
        output.fraction = a;
        output.normal = (s + a * d).Normalized();
        return true;
    }
    return false;
}

// =============================================================================
// PolygonShape
// =============================================================================
std::vector<Vec2> ComputeConvexHull(std::vector<Vec2> pts) {
    // Andrew's monotone chain：先按 x 再按 y 排序，然后分别构建下凸壳和上凸壳。
    std::sort(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });

    // 去掉非常接近的重复点
    constexpr float kWeldSq = 0.5f * 0.005f * 0.5f * 0.005f;
    pts.erase(std::unique(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return DistanceSquared(a, b) < kWeldSq; }),
              pts.end());

    const size_t n = pts.size();
    if (n < 3) return {};

    std::vector<Vec2> hull(2 * n);
    size_t k = 0;
    // 下凸壳
    for (size_t i = 0; i < n; ++i) {
        while (k >= 2 && Cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0.0f) --k;
        hull[k++] = pts[i];
    }
    // 上凸壳
    for (size_t i = n - 1, t = k + 1; i-- > 0;) {
        while (k >= t && Cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0.0f) --k;
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);  // 最后一个点与第一个点重复
    return hull;
}

namespace {

/// 计算多边形质心（三角扇分解，每个三角形的质心按面积加权）
Vec2 ComputeCentroid(const Vec2* vs, int count) {
    Vec2 c{0.0f, 0.0f};
    float area = 0.0f;
    const Vec2 origin = vs[0];  // 以第一个顶点为参考点可以提高精度
    constexpr float inv3 = 1.0f / 3.0f;

    for (int i = 1; i < count - 1; ++i) {
        const Vec2 e1 = vs[i] - origin;
        const Vec2 e2 = vs[i + 1] - origin;
        const float a = 0.5f * Cross(e1, e2);
        c += a * inv3 * (e1 + e2);
        area += a;
    }
    assert(area > kEpsilon);
    return c / area + origin;
}

}  // namespace

bool PolygonShape::Set(const std::vector<Vec2>& points) {
    std::vector<Vec2> hull = ComputeConvexHull(points);
    if (hull.size() < 3) return false;
    if (hull.size() > static_cast<size_t>(kMaxPolygonVertices)) hull.resize(kMaxPolygonVertices);

    m_count = static_cast<int>(hull.size());

    // 平移到质心坐标系
    m_centroidOffset = ComputeCentroid(hull.data(), m_count);
    for (int i = 0; i < m_count; ++i) m_vertices[static_cast<size_t>(i)] = hull[static_cast<size_t>(i)] - m_centroidOffset;

    // 每条边的外法线：CCW 多边形边向量顺时针转 90° 即为外法线
    for (int i = 0; i < m_count; ++i) {
        const int next = (i + 1) % m_count;
        const Vec2 edge = m_vertices[static_cast<size_t>(next)] - m_vertices[static_cast<size_t>(i)];
        m_normals[static_cast<size_t>(i)] = Cross(edge, 1.0f).Normalized();
    }
    return true;
}

void PolygonShape::SetAsBox(float hw, float hh) {
    m_count = 4;
    m_vertices[0] = {-hw, -hh};
    m_vertices[1] = {hw, -hh};
    m_vertices[2] = {hw, hh};
    m_vertices[3] = {-hw, hh};
    m_normals[0] = {0.0f, -1.0f};
    m_normals[1] = {1.0f, 0.0f};
    m_normals[2] = {0.0f, 1.0f};
    m_normals[3] = {-1.0f, 0.0f};
    m_centroidOffset = {0.0f, 0.0f};
}

void PolygonShape::SetAsRegular(int sides, float radius) {
    sides = Clamp(sides, 3, kMaxPolygonVertices);
    std::vector<Vec2> pts;
    pts.reserve(static_cast<size_t>(sides));
    for (int i = 0; i < sides; ++i) {
        const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(sides) + 0.5f * kPi;
        pts.push_back({radius * std::cos(a), radius * std::sin(a)});
    }
    Set(pts);
}

PolygonShape PolygonShape::MakeBox(float hw, float hh) {
    PolygonShape p;
    p.SetAsBox(hw, hh);
    return p;
}

PolygonShape PolygonShape::MakeRegular(int sides, float radius) {
    PolygonShape p;
    p.SetAsRegular(sides, radius);
    return p;
}

std::unique_ptr<Shape> PolygonShape::Clone() const { return std::make_unique<PolygonShape>(*this); }

MassData PolygonShape::ComputeMass(float density) const {
    // 把多边形分解为以参考点 s 为公共顶点的三角形扇，逐个累加面积、质心与惯量。
    // 三角形 (0, e1, e2) 绕原点的二阶矩：
    //   ∫x² dA = D/12 * (e1x² + e1x*e2x + e2x²)，D = cross(e1, e2)
    assert(m_count >= 3);

    Vec2 center{0.0f, 0.0f};
    float area = 0.0f;
    float I = 0.0f;
    const Vec2 s = m_vertices[0];
    constexpr float inv3 = 1.0f / 3.0f;

    for (int i = 0; i < m_count; ++i) {
        const Vec2 e1 = m_vertices[static_cast<size_t>(i)] - s;
        const Vec2 e2 = m_vertices[static_cast<size_t>((i + 1) % m_count)] - s;
        const float D = Cross(e1, e2);
        const float triArea = 0.5f * D;
        area += triArea;
        center += triArea * inv3 * (e1 + e2);

        const float intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
        const float inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
        I += (0.25f * inv3 * D) * (intx2 + inty2);
    }

    MassData md;
    md.mass = density * area;
    center *= 1.0f / area;
    md.center = center + s;
    // 平行轴定理：I_origin 先移到参考点 s，再移到质心
    md.inertia = density * I;
    md.inertia += md.mass * (Dot(md.center, md.center) - Dot(center, center));
    // 转为绕质心的惯量（质心已在原点附近，这一步对一般多边形是很小的修正）
    md.inertia -= md.mass * Dot(md.center, md.center);
    return md;
}

AABB PolygonShape::ComputeAABB(const Transform& xf) const {
    Vec2 lo = xf.Apply(m_vertices[0]);
    Vec2 hi = lo;
    for (int i = 1; i < m_count; ++i) {
        const Vec2 v = xf.Apply(m_vertices[static_cast<size_t>(i)]);
        lo = Min(lo, v);
        hi = Max(hi, v);
    }
    return {lo, hi};
}

bool PolygonShape::TestPoint(const Transform& xf, Vec2 p) const {
    const Vec2 local = xf.ApplyInv(p);
    for (int i = 0; i < m_count; ++i) {
        if (Dot(m_normals[static_cast<size_t>(i)], local - m_vertices[static_cast<size_t>(i)]) > 0.0f) return false;
    }
    return true;
}

bool PolygonShape::RayCast(const RayCastInput& input, const Transform& xf, RayCastOutput& output) const {
    // 把射线变换到多边形局部坐标，再用"半平面裁剪"求进入/离开参数区间 [lower, upper]
    const Vec2 p1 = xf.ApplyInv(input.p1);
    const Vec2 p2 = xf.ApplyInv(input.p2);
    const Vec2 d = p2 - p1;

    float lower = 0.0f;
    float upper = input.maxFraction;
    int index = -1;

    for (int i = 0; i < m_count; ++i) {
        const Vec2 n = m_normals[static_cast<size_t>(i)];
        const float numerator = Dot(n, m_vertices[static_cast<size_t>(i)] - p1);
        const float denominator = Dot(n, d);

        if (denominator == 0.0f) {
            if (numerator < 0.0f) return false;  // 平行且在外侧
        } else if (denominator < 0.0f && numerator < lower * denominator) {
            lower = numerator / denominator;  // 进入半平面
            index = i;
        } else if (denominator > 0.0f && numerator < upper * denominator) {
            upper = numerator / denominator;  // 离开半平面
        }

        if (upper < lower) return false;
    }

    if (index >= 0) {
        output.fraction = lower;
        output.normal = xf.q.Rotate(m_normals[static_cast<size_t>(index)]);
        return true;
    }
    return false;
}

}  // namespace kinema
