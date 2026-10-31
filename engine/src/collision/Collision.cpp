#include "kinema/collision/Collision.h"

#include <cassert>
#include <limits>

namespace kinema {

// =============================================================================
// 圆 vs 圆
// =============================================================================
void CollideCircles(Manifold& m, const CircleShape& a, const Transform& xfA, const CircleShape& b,
                    const Transform& xfB) {
    m.pointCount = 0;

    const Vec2 pA = xfA.p;
    const Vec2 pB = xfB.p;
    const Vec2 d = pB - pA;
    const float distSq = d.LengthSquared();
    const float rA = a.GetRadius();
    const float rB = b.GetRadius();
    const float radius = rA + rB;

    if (distSq > radius * radius) return;

    const float dist = std::sqrt(distSq);
    const Vec2 normal = dist > kEpsilon ? d / dist : Vec2{0.0f, 1.0f};

    const Vec2 cA = pA + rA * normal;  // A 表面上离 B 最近的点
    const Vec2 cB = pB - rB * normal;  // B 表面上离 A 最近的点

    m.normal = normal;
    m.pointCount = 1;
    m.points[0].point = 0.5f * (cA + cB);
    m.points[0].separation = dist - radius;
    m.points[0].id = 0;
}

// =============================================================================
// 多边形 vs 圆
// =============================================================================
void CollidePolygonAndCircle(Manifold& m, const PolygonShape& poly, const Transform& xfA,
                             const CircleShape& circle, const Transform& xfB) {
    m.pointCount = 0;

    // 把圆心变换到多边形的局部坐标系
    const Vec2 c = xfA.ApplyInv(xfB.p);
    const float radius = circle.GetRadius();

    const int count = poly.GetVertexCount();
    const Vec2* vertices = poly.GetVertices();
    const Vec2* normals = poly.GetNormals();

    // 1) 找到圆心"最突出"的那条边（分离量最大）
    int normalIndex = 0;
    float separation = -std::numeric_limits<float>::max();
    for (int i = 0; i < count; ++i) {
        const float s = Dot(normals[i], c - vertices[i]);
        if (s > radius) return;  // 找到分离轴，提前退出
        if (s > separation) {
            separation = s;
            normalIndex = i;
        }
    }

    const int i1 = normalIndex;
    const int i2 = (i1 + 1) % count;
    const Vec2 v1 = vertices[i1];
    const Vec2 v2 = vertices[i2];

    Vec2 localNormal;
    Vec2 closest;  // 多边形上离圆心最近的点

    if (separation < kEpsilon) {
        // 2a) 圆心在多边形内部：直接使用该面的法线
        localNormal = normals[i1];
        closest = c - separation * localNormal;
    } else {
        // 2b) 判断圆心投影落在边的哪个区域（Voronoi 区域）
        const float u1 = Dot(c - v1, v2 - v1);
        const float u2 = Dot(c - v2, v1 - v2);

        if (u1 <= 0.0f) {
            // 顶点 v1 区域
            if (DistanceSquared(c, v1) > radius * radius) return;
            localNormal = (c - v1).Normalized();
            closest = v1;
        } else if (u2 <= 0.0f) {
            // 顶点 v2 区域
            if (DistanceSquared(c, v2) > radius * radius) return;
            localNormal = (c - v2).Normalized();
            closest = v2;
        } else {
            // 边区域
            const Vec2 faceCenter = 0.5f * (v1 + v2);
            const float s = Dot(c - faceCenter, normals[i1]);
            if (s > radius) return;
            localNormal = normals[i1];
            closest = c - s * localNormal;
        }
    }

    const Vec2 normal = xfA.q.Rotate(localNormal);
    const Vec2 pA = xfA.Apply(closest);        // 多边形表面点
    const Vec2 pB = xfB.p - radius * normal;    // 圆表面点

    m.normal = normal;
    m.pointCount = 1;
    m.points[0].point = 0.5f * (pA + pB);
    m.points[0].separation = Dot(xfB.p - pA, normal) - radius;
    ContactFeature f;
    f.indexA = static_cast<uint8_t>(normalIndex);
    f.typeA = ContactFeature::Face;
    m.points[0].id = f.Key();
}

// =============================================================================
// 多边形 vs 多边形（SAT + 裁剪）
// =============================================================================
namespace {

struct ClipVertex {
    Vec2 v;
    ContactFeature id;
};

/// 在 poly1 的所有面法线上，找出与 poly2 分离量最大的那一个。
/// 分离量 = poly2 在该法线方向上最"深"的顶点到该面的距离。
float FindMaxSeparation(int& edgeIndex, const PolygonShape& poly1, const Transform& xf1, const PolygonShape& poly2,
                        const Transform& xf2) {
    const int count1 = poly1.GetVertexCount();
    const int count2 = poly2.GetVertexCount();
    const Vec2* n1s = poly1.GetNormals();
    const Vec2* v1s = poly1.GetVertices();
    const Vec2* v2s = poly2.GetVertices();

    // 在 poly2 的局部坐标系中计算，减少变换次数
    const Transform xf = MulT(xf2, xf1);

    int bestIndex = 0;
    float maxSeparation = -std::numeric_limits<float>::max();
    for (int i = 0; i < count1; ++i) {
        const Vec2 n = xf.q.Rotate(n1s[i]);
        const Vec2 v1 = xf.Apply(v1s[i]);

        // poly2 在 -n 方向上的支撑点
        float si = std::numeric_limits<float>::max();
        for (int j = 0; j < count2; ++j) {
            const float sij = Dot(n, v2s[j] - v1);
            if (sij < si) si = sij;
        }

        if (si > maxSeparation) {
            maxSeparation = si;
            bestIndex = i;
        }
    }

    edgeIndex = bestIndex;
    return maxSeparation;
}

/// 在 poly2 上寻找"入射边"：法线与参考面法线最反向的那条边
void FindIncidentEdge(ClipVertex c[2], const PolygonShape& poly1, const Transform& xf1, int edge1,
                      const PolygonShape& poly2, const Transform& xf2) {
    const Vec2* normals1 = poly1.GetNormals();
    const int count2 = poly2.GetVertexCount();
    const Vec2* vertices2 = poly2.GetVertices();
    const Vec2* normals2 = poly2.GetNormals();

    // 参考面法线变换到 poly2 的局部坐标
    const Vec2 normal1 = xf2.q.InvRotate(xf1.q.Rotate(normals1[edge1]));

    int index = 0;
    float minDot = std::numeric_limits<float>::max();
    for (int i = 0; i < count2; ++i) {
        const float d = Dot(normal1, normals2[i]);
        if (d < minDot) {
            minDot = d;
            index = i;
        }
    }

    const int i1 = index;
    const int i2 = (i1 + 1) % count2;

    c[0].v = xf2.Apply(vertices2[i1]);
    c[0].id.indexA = static_cast<uint8_t>(edge1);
    c[0].id.indexB = static_cast<uint8_t>(i1);
    c[0].id.typeA = ContactFeature::Face;
    c[0].id.typeB = ContactFeature::Vertex;

    c[1].v = xf2.Apply(vertices2[i2]);
    c[1].id.indexA = static_cast<uint8_t>(edge1);
    c[1].id.indexB = static_cast<uint8_t>(i2);
    c[1].id.typeA = ContactFeature::Face;
    c[1].id.typeB = ContactFeature::Vertex;
}

/// Sutherland-Hodgman 裁剪：保留线段在半平面 dot(normal, x) <= offset 内的部分
int ClipSegmentToLine(ClipVertex vOut[2], const ClipVertex vIn[2], Vec2 normal, float offset, int vertexIndexA) {
    int numOut = 0;

    const float d0 = Dot(normal, vIn[0].v) - offset;
    const float d1 = Dot(normal, vIn[1].v) - offset;

    if (d0 <= 0.0f) vOut[numOut++] = vIn[0];
    if (d1 <= 0.0f) vOut[numOut++] = vIn[1];

    // 两端点在平面两侧：求交点
    if (d0 * d1 < 0.0f) {
        const float t = d0 / (d0 - d1);
        vOut[numOut].v = vIn[0].v + t * (vIn[1].v - vIn[0].v);
        vOut[numOut].id.indexA = static_cast<uint8_t>(vertexIndexA);
        vOut[numOut].id.indexB = vIn[0].id.indexB;
        vOut[numOut].id.typeA = ContactFeature::Vertex;
        vOut[numOut].id.typeB = ContactFeature::Face;
        ++numOut;
    }
    return numOut;
}

}  // namespace

void CollidePolygons(Manifold& m, const PolygonShape& polyA, const Transform& xfA, const PolygonShape& polyB,
                     const Transform& xfB) {
    m.pointCount = 0;

    // 1) 分离轴测试：分别以 A 和 B 的面法线为候选轴
    int edgeA = 0;
    const float separationA = FindMaxSeparation(edgeA, polyA, xfA, polyB, xfB);
    if (separationA > 0.0f) return;

    int edgeB = 0;
    const float separationB = FindMaxSeparation(edgeB, polyB, xfB, polyA, xfA);
    if (separationB > 0.0f) return;

    // 2) 选择参考面：偏向 A（加一点容差，避免两帧之间来回切换导致抖动）
    const PolygonShape* poly1;
    const PolygonShape* poly2;
    Transform xf1, xf2;
    int edge1;
    bool flip;
    constexpr float kTol = 0.1f * kLinearSlop;

    if (separationB > separationA + kTol) {
        poly1 = &polyB; poly2 = &polyA;
        xf1 = xfB; xf2 = xfA;
        edge1 = edgeB;
        flip = true;
    } else {
        poly1 = &polyA; poly2 = &polyB;
        xf1 = xfA; xf2 = xfB;
        edge1 = edgeA;
        flip = false;
    }

    // 3) 找入射边
    ClipVertex incidentEdge[2];
    FindIncidentEdge(incidentEdge, *poly1, xf1, edge1, *poly2, xf2);

    const int count1 = poly1->GetVertexCount();
    const int iv1 = edge1;
    const int iv2 = (edge1 + 1) % count1;

    Vec2 v11 = poly1->GetVertex(iv1);
    Vec2 v12 = poly1->GetVertex(iv2);

    const Vec2 localTangent = (v12 - v11).Normalized();
    const Vec2 tangent = xf1.q.Rotate(localTangent);
    const Vec2 normal = Cross(tangent, 1.0f);  // 参考面外法线

    v11 = xf1.Apply(v11);
    v12 = xf1.Apply(v12);

    const float frontOffset = Dot(normal, v11);
    const float sideOffset1 = -Dot(tangent, v11);
    const float sideOffset2 = Dot(tangent, v12);

    // 4) 用参考面的两条侧边裁剪入射边
    ClipVertex clip1[2];
    ClipVertex clip2[2];
    if (ClipSegmentToLine(clip1, incidentEdge, -tangent, sideOffset1, iv1) < 2) return;
    if (ClipSegmentToLine(clip2, clip1, tangent, sideOffset2, iv2) < 2) return;

    // 5) 保留位于参考面后方（穿透）的点
    m.normal = flip ? -normal : normal;

    int pointCount = 0;
    for (const ClipVertex& cv : clip2) {
        const float separation = Dot(normal, cv.v) - frontOffset;
        if (separation <= 0.0f) {
            ManifoldPoint& mp = m.points[pointCount];
            // cv.v 在入射多边形表面；减去一半穿透得到两表面中点
            mp.point = cv.v - 0.5f * separation * normal;
            mp.separation = separation;

            ContactFeature id = cv.id;
            if (flip) {
                std::swap(id.indexA, id.indexB);
                std::swap(id.typeA, id.typeB);
            }
            mp.id = id.Key();
            ++pointCount;
        }
    }
    m.pointCount = pointCount;
}

// =============================================================================
// 分派
// =============================================================================
void Collide(Manifold& m, const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB) {
    assert(a.GetType() >= b.GetType());

    if (a.GetType() == ShapeType::Circle) {
        CollideCircles(m, static_cast<const CircleShape&>(a), xfA, static_cast<const CircleShape&>(b), xfB);
    } else if (b.GetType() == ShapeType::Circle) {
        CollidePolygonAndCircle(m, static_cast<const PolygonShape&>(a), xfA, static_cast<const CircleShape&>(b), xfB);
    } else {
        CollidePolygons(m, static_cast<const PolygonShape&>(a), xfA, static_cast<const PolygonShape&>(b), xfB);
    }
}

}  // namespace kinema
