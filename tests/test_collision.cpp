#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <set>

#include <kinema/collision/BroadPhase.h>
#include <kinema/collision/Collision.h>

using namespace kinema;

TEST(Collision, CirclesOverlapping) {
    Manifold m;
    CollideCircles(m, CircleShape(1.0f), Transform({0, 0}, Rot(0)), CircleShape(1.0f), Transform({1.5f, 0}, Rot(0)));
    ASSERT_EQ(m.pointCount, 1);
    EXPECT_NEAR(m.normal.x, 1.0f, 1e-6f);  // 从 A 指向 B
    EXPECT_NEAR(m.points[0].separation, -0.5f, 1e-6f);
    EXPECT_NEAR(m.points[0].point.x, 0.75f, 1e-6f);
}

TEST(Collision, CirclesSeparated) {
    Manifold m;
    CollideCircles(m, CircleShape(1.0f), Transform({0, 0}, Rot(0)), CircleShape(1.0f), Transform({2.5f, 0}, Rot(0)));
    EXPECT_EQ(m.pointCount, 0);
}

TEST(Collision, PolygonCircleFace) {
    Manifold m;
    const PolygonShape box = PolygonShape::MakeBox(1.0f, 1.0f);
    CollidePolygonAndCircle(m, box, Transform({0, 0}, Rot(0)), CircleShape(0.5f), Transform({0, 1.4f}, Rot(0)));
    ASSERT_EQ(m.pointCount, 1);
    EXPECT_NEAR(m.normal.y, 1.0f, 1e-5f);
    EXPECT_NEAR(m.points[0].separation, -0.1f, 1e-5f);
}

TEST(Collision, PolygonCircleVertexRegion) {
    Manifold m;
    const PolygonShape box = PolygonShape::MakeBox(1.0f, 1.0f);
    // 圆心在右上角对角线方向
    CollidePolygonAndCircle(m, box, Transform({0, 0}, Rot(0)), CircleShape(0.5f), Transform({1.3f, 1.3f}, Rot(0)));
    ASSERT_EQ(m.pointCount, 1);
    EXPECT_NEAR(m.normal.x, std::sqrt(0.5f), 1e-4f);
    EXPECT_NEAR(m.normal.y, std::sqrt(0.5f), 1e-4f);
    EXPECT_LT(m.points[0].separation, 0.0f);

    CollidePolygonAndCircle(m, box, Transform({0, 0}, Rot(0)), CircleShape(0.5f), Transform({1.5f, 1.5f}, Rot(0)));
    EXPECT_EQ(m.pointCount, 0);  // 0.707 > 0.5
}

TEST(Collision, BoxRestingOnBoxHasTwoPoints) {
    Manifold m;
    const PolygonShape ground = PolygonShape::MakeBox(5.0f, 0.5f);
    const PolygonShape box = PolygonShape::MakeBox(0.5f, 0.5f);
    CollidePolygons(m, ground, Transform({0, 0}, Rot(0)), box, Transform({0, 0.99f}, Rot(0)));
    ASSERT_EQ(m.pointCount, 2);
    EXPECT_NEAR(m.normal.y, 1.0f, 1e-5f);
    for (int i = 0; i < 2; ++i) EXPECT_NEAR(m.points[i].separation, -0.01f, 1e-5f);
    // 两个接触点应位于箱子底部两角
    const float xs[2] = {m.points[0].point.x, m.points[1].point.x};
    EXPECT_NEAR(std::min(xs[0], xs[1]), -0.5f, 1e-4f);
    EXPECT_NEAR(std::max(xs[0], xs[1]), 0.5f, 1e-4f);
    // 两个点的特征 ID 不同（热启动依赖它）
    EXPECT_NE(m.points[0].id, m.points[1].id);
}

TEST(Collision, RotatedBoxCornerContact) {
    Manifold m;
    const PolygonShape ground = PolygonShape::MakeBox(5.0f, 0.5f);
    const PolygonShape box = PolygonShape::MakeBox(0.5f, 0.5f);
    const float h = std::sqrt(0.5f);  // 旋转 45° 的半对角线
    CollidePolygons(m, ground, Transform({0, 0}, Rot(0)), box, Transform({0, 0.5f + h - 0.02f}, Rot(kPi / 4)));
    ASSERT_EQ(m.pointCount, 1);
    EXPECT_NEAR(m.normal.y, 1.0f, 1e-4f);
    EXPECT_NEAR(m.points[0].separation, -0.02f, 1e-4f);
}

TEST(Collision, PolygonsSeparated) {
    Manifold m;
    const PolygonShape a = PolygonShape::MakeBox(0.5f, 0.5f);
    CollidePolygons(m, a, Transform({0, 0}, Rot(0)), a, Transform({1.1f, 0}, Rot(0.2f)));
    EXPECT_EQ(m.pointCount, 0);
}

TEST(Collision, NormalAlwaysPointsFromAToB) {
    // 无论参考面选在 A 还是 B 上，法线都必须从 A 指向 B
    const PolygonShape small = PolygonShape::MakeBox(0.5f, 0.5f);
    const PolygonShape big = PolygonShape::MakeBox(3.0f, 0.5f);
    Manifold m;
    CollidePolygons(m, small, Transform({0, 0.95f}, Rot(0)), big, Transform({0, 0}, Rot(0)));
    ASSERT_GT(m.pointCount, 0);
    EXPECT_NEAR(m.normal.y, -1.0f, 1e-5f);
}

// 宽阶段：SAP 必须与暴力枚举得到完全相同的候选对集合
TEST(BroadPhase, SweepAndPruneMatchesBruteForce) {
    std::mt19937 rng(123);
    std::uniform_real_distribution<float> pos(-50.0f, 50.0f);
    std::uniform_real_distribution<float> size(0.1f, 3.0f);

    std::vector<BroadPhaseProxy> proxies;
    for (uint32_t i = 0; i < 500; ++i) {
        const Vec2 c{pos(rng), pos(rng) * 0.3f};
        const Vec2 e{size(rng), size(rng)};
        proxies.push_back({AABB(c - e, c + e), i});
    }

    auto normalize = [](std::vector<ProxyPair> pairs) {
        std::set<ProxyPair> s;
        for (auto [a, b] : pairs) s.insert({std::min(a, b), std::max(a, b)});
        return s;
    };

    BruteForceBroadPhase brute;
    SweepAndPruneBroadPhase sap;
    std::vector<ProxyPair> p1, p2;
    brute.ComputePairs(proxies, p1);
    sap.ComputePairs(proxies, p2);

    EXPECT_GT(p1.size(), 0u);
    EXPECT_EQ(normalize(p1), normalize(p2));
}
