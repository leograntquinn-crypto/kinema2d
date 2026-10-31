#include <gtest/gtest.h>

#include <kinema/collision/Shape.h>

using namespace kinema;

TEST(CircleShape, MassProperties) {
    const CircleShape c(2.0f);
    const MassData md = c.ComputeMass(1.0f);
    EXPECT_NEAR(md.mass, kPi * 4.0f, 1e-4f);
    EXPECT_NEAR(md.inertia, 0.5f * md.mass * 4.0f, 1e-3f);
}

TEST(PolygonShape, BoxMassProperties) {
    const PolygonShape box = PolygonShape::MakeBox(1.0f, 0.5f);  // 2 x 1
    const MassData md = box.ComputeMass(2.0f);
    EXPECT_NEAR(md.mass, 4.0f, 1e-5f);
    // 矩形绕中心：I = m (w² + h²) / 12
    EXPECT_NEAR(md.inertia, 4.0f * (4.0f + 1.0f) / 12.0f, 1e-4f);
    EXPECT_NEAR(md.center.x, 0.0f, 1e-6f);
    EXPECT_NEAR(md.center.y, 0.0f, 1e-6f);
}

TEST(PolygonShape, SetComputesHullAndRecenters) {
    PolygonShape p;
    // 一个偏离原点的正方形 + 一个内部点 + 重复点
    ASSERT_TRUE(p.Set({{10, 10}, {12, 10}, {12, 12}, {10, 12}, {11, 11}, {12, 12}}));
    EXPECT_EQ(p.GetVertexCount(), 4);
    EXPECT_NEAR(p.GetCentroidOffset().x, 11.0f, 1e-5f);
    EXPECT_NEAR(p.GetCentroidOffset().y, 11.0f, 1e-5f);

    // 顶点必须是逆时针（相邻边叉积为正）
    for (int i = 0; i < p.GetVertexCount(); ++i) {
        const Vec2 a = p.GetVertex(i);
        const Vec2 b = p.GetVertex((i + 1) % 4);
        const Vec2 c = p.GetVertex((i + 2) % 4);
        EXPECT_GT(Cross(b - a, c - b), 0.0f);
    }
    // 法线是单位长度且朝外
    for (int i = 0; i < p.GetVertexCount(); ++i) {
        EXPECT_NEAR(p.GetNormal(i).Length(), 1.0f, 1e-5f);
        EXPECT_GT(Dot(p.GetNormal(i), p.GetVertex(i)), 0.0f);
    }
}

TEST(PolygonShape, DegenerateInputFails) {
    PolygonShape p;
    EXPECT_FALSE(p.Set({{0, 0}, {1, 1}, {2, 2}}));  // 共线
    EXPECT_FALSE(p.Set({{0, 0}, {1, 0}}));
}

TEST(ConvexHull, Square) {
    const auto hull = ComputeConvexHull({{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0.5f, 0.5f}, {0.2f, 0.8f}});
    EXPECT_EQ(hull.size(), 4u);
}

TEST(Shapes, TestPoint) {
    const Transform xf({5.0f, 0.0f}, Rot(kPi / 4.0f));
    const PolygonShape box = PolygonShape::MakeBox(1.0f, 1.0f);
    EXPECT_TRUE(box.TestPoint(xf, {5.0f, 0.0f}));
    EXPECT_TRUE(box.TestPoint(xf, {5.0f, 1.3f}));   // 旋转 45° 后顶点在 y≈1.414
    EXPECT_FALSE(box.TestPoint(xf, {6.0f, 1.0f}));

    const CircleShape circle(1.0f);
    EXPECT_TRUE(circle.TestPoint(xf, {5.5f, 0.5f}));
    EXPECT_FALSE(circle.TestPoint(xf, {6.0f, 1.0f}));
}

TEST(Shapes, RayCastCircle) {
    const CircleShape circle(1.0f);
    const Transform xf({5.0f, 0.0f}, Rot(0.0f));
    RayCastOutput out;
    ASSERT_TRUE(circle.RayCast({{0.0f, 0.0f}, {10.0f, 0.0f}, 1.0f}, xf, out));
    EXPECT_NEAR(out.fraction, 0.4f, 1e-5f);  // 命中 x = 4
    EXPECT_NEAR(out.normal.x, -1.0f, 1e-5f);
    EXPECT_FALSE(circle.RayCast({{0.0f, 2.0f}, {10.0f, 2.0f}, 1.0f}, xf, out));
}

TEST(Shapes, RayCastPolygon) {
    const PolygonShape box = PolygonShape::MakeBox(1.0f, 1.0f);
    const Transform xf({0.0f, 5.0f}, Rot(0.0f));
    RayCastOutput out;
    ASSERT_TRUE(box.RayCast({{0.0f, 0.0f}, {0.0f, 10.0f}, 1.0f}, xf, out));
    EXPECT_NEAR(out.fraction, 0.4f, 1e-5f);  // 命中 y = 4
    EXPECT_NEAR(out.normal.y, -1.0f, 1e-5f);
    EXPECT_FALSE(box.RayCast({{3.0f, 0.0f}, {3.0f, 10.0f}, 1.0f}, xf, out));
}

TEST(Shapes, AABB) {
    const PolygonShape box = PolygonShape::MakeBox(1.0f, 2.0f);
    const AABB a = box.ComputeAABB(Transform({1.0f, 1.0f}, Rot(0.0f)));
    EXPECT_NEAR(a.lower.x, 0.0f, 1e-6f);
    EXPECT_NEAR(a.upper.y, 3.0f, 1e-6f);
    const AABB r = box.ComputeAABB(Transform({0.0f, 0.0f}, Rot(kPi / 2.0f)));
    EXPECT_NEAR(r.Width(), 4.0f, 1e-5f);
    EXPECT_NEAR(r.Height(), 2.0f, 1e-5f);
}
