#include <gtest/gtest.h>

#include <kinema/math/AABB.h>
#include <kinema/math/Math.h>

using namespace kinema;

TEST(Vec2, Arithmetic) {
    const Vec2 a{1.0f, 2.0f};
    const Vec2 b{3.0f, -1.0f};
    EXPECT_EQ(a + b, Vec2(4.0f, 1.0f));
    EXPECT_EQ(a - b, Vec2(-2.0f, 3.0f));
    EXPECT_EQ(2.0f * a, Vec2(2.0f, 4.0f));
    EXPECT_FLOAT_EQ(Dot(a, b), 1.0f);
    EXPECT_FLOAT_EQ(Cross(a, b), -7.0f);
}

TEST(Vec2, CrossWithScalarMatchesAngularVelocity) {
    // ω × r：角速度为正（逆时针）时，x 轴上的点速度应指向 +y
    const Vec2 v = Cross(2.0f, Vec2{1.0f, 0.0f});
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
}

TEST(Vec2, Normalize) {
    Vec2 v{3.0f, 4.0f};
    EXPECT_FLOAT_EQ(v.Normalize(), 5.0f);
    EXPECT_NEAR(v.Length(), 1.0f, 1e-6f);

    Vec2 zero;
    EXPECT_FLOAT_EQ(zero.Normalize(), 0.0f);
    EXPECT_EQ(zero, Vec2());
}

TEST(Rot, RotateAndInverse) {
    const Rot q(kPi / 2.0f);
    const Vec2 r = q.Rotate({1.0f, 0.0f});
    EXPECT_NEAR(r.x, 0.0f, 1e-6f);
    EXPECT_NEAR(r.y, 1.0f, 1e-6f);

    const Vec2 back = q.InvRotate(r);
    EXPECT_NEAR(back.x, 1.0f, 1e-6f);
    EXPECT_NEAR(back.y, 0.0f, 1e-6f);
    EXPECT_NEAR(q.Angle(), kPi / 2.0f, 1e-6f);
}

TEST(Transform, RoundTrip) {
    const Transform xf({2.0f, 3.0f}, Rot(0.7f));
    const Vec2 local{0.5f, -1.5f};
    const Vec2 world = xf.Apply(local);
    const Vec2 again = xf.ApplyInv(world);
    EXPECT_NEAR(again.x, local.x, 1e-5f);
    EXPECT_NEAR(again.y, local.y, 1e-5f);
}

TEST(Transform, MulTComposesCorrectly) {
    const Transform a({1.0f, 2.0f}, Rot(0.3f));
    const Transform b({-2.0f, 0.5f}, Rot(-1.1f));
    const Transform rel = MulT(a, b);  // b 相对 a
    const Vec2 p{0.3f, 0.9f};
    const Vec2 viaB = b.Apply(p);
    const Vec2 viaA = a.Apply(rel.Apply(p));
    EXPECT_NEAR(viaA.x, viaB.x, 1e-5f);
    EXPECT_NEAR(viaA.y, viaB.y, 1e-5f);
}

TEST(Mat22, SolveMatchesInverse) {
    const Mat22 m({4.0f, 1.0f}, {2.0f, 3.0f});
    const Vec2 b{1.0f, 2.0f};
    const Vec2 x1 = m.Solve(b);
    const Vec2 x2 = m.GetInverse() * b;
    EXPECT_NEAR(x1.x, x2.x, 1e-6f);
    EXPECT_NEAR(x1.y, x2.y, 1e-6f);
    const Vec2 check = m * x1;
    EXPECT_NEAR(check.x, b.x, 1e-5f);
    EXPECT_NEAR(check.y, b.y, 1e-5f);
}

TEST(AABB, Overlap) {
    const AABB a({0, 0}, {1, 1});
    const AABB b({0.5f, 0.5f}, {2, 2});
    const AABB c({1.5f, 1.5f}, {2, 2});
    EXPECT_TRUE(Overlaps(a, b));
    EXPECT_FALSE(Overlaps(a, c));
    EXPECT_TRUE(a.Contains({0.5f, 0.5f}));
    EXPECT_FALSE(a.Contains({1.5f, 0.5f}));
    EXPECT_FLOAT_EQ(AABB::Combine(a, c).Width(), 2.0f);
}
