// =============================================================================
// World 集成测试：验证整体物理行为是否符合物理规律
// =============================================================================
#include <gtest/gtest.h>

#include <kinema/Kinema.h>
#include <kinema_scene/SceneLibrary.h>
#include <kinema_scene/SceneSerializer.h>

using namespace kinema;

namespace {

constexpr float kDt = 1.0f / 60.0f;

Body* AddGround(World& world) {
    BodyDef def;
    def.type = BodyType::Static;
    def.position = {0.0f, -1.0f};
    return world.CreateBody(def, PolygonShape::MakeBox(50.0f, 1.0f));
}

}  // namespace

TEST(World, FreeFallMatchesKinematics) {
    World world;
    BodyDef def;
    def.position = {0.0f, 100.0f};
    def.angularDamping = 0.0f;
    Body* ball = world.CreateBody(def, CircleShape(0.5f));

    const int steps = 60;
    for (int i = 0; i < steps; ++i) world.Step(kDt);

    // 半隐式欧拉：y = y0 - g dt² n(n+1)/2
    const float g = 9.81f;
    const float expected = 100.0f - g * kDt * kDt * steps * (steps + 1) / 2.0f;
    EXPECT_NEAR(ball->GetPosition().y, expected, 1e-3f);
    EXPECT_NEAR(ball->GetLinearVelocity().y, -g * steps * kDt, 1e-3f);
}

TEST(World, BallComesToRestOnGround) {
    World world;
    AddGround(world);
    BodyDef def;
    def.position = {0.0f, 5.0f};
    Body* ball = world.CreateBody(def, CircleShape(0.5f));

    for (int i = 0; i < 300; ++i) world.Step(kDt);

    EXPECT_NEAR(ball->GetPosition().y, 0.5f, 0.02f);
    EXPECT_NEAR(ball->GetLinearVelocity().y, 0.0f, 0.05f);
}

TEST(World, BoxStackIsStableAndSleeps) {
    World world;
    AddGround(world);
    std::vector<Body*> boxes;
    for (int i = 0; i < 10; ++i) {
        BodyDef def;
        def.position = {0.0f, 0.5f + static_cast<float>(i)};
        boxes.push_back(world.CreateBody(def, PolygonShape::MakeBox(0.5f, 0.5f)));
    }

    for (int i = 0; i < 600; ++i) world.Step(kDt);

    const Body* top = boxes.back();
    EXPECT_NEAR(top->GetPosition().x, 0.0f, 0.05f);
    EXPECT_NEAR(top->GetPosition().y, 9.5f, 0.1f);
    EXPECT_NEAR(top->GetAngle(), 0.0f, 0.02f);
    // 整个堆叠应当作为一个岛屿进入休眠
    for (const Body* b : boxes) EXPECT_FALSE(b->IsAwake());
}

TEST(World, PyramidSceneSettles) {
    World world;
    scene::BuildPyramid(world);
    for (int i = 0; i < 900; ++i) world.Step(kDt);

    // 顶部箱子仍应在最初的高度附近（没有倒塌）
    float maxY = 0.0f;
    for (const auto& b : world.GetBodies()) {
        if (b->IsDynamic()) maxY = std::max(maxY, b->GetPosition().y);
    }
    EXPECT_GT(maxY, 19.0f);
    EXPECT_EQ(world.GetStats().awakeBodyCount, 0);
}

TEST(World, RestitutionMakesBallBounce) {
    World world;
    AddGround(world);
    BodyDef def;
    def.position = {0.0f, 5.0f};
    def.material.restitution = 0.9f;
    Body* ball = world.CreateBody(def, CircleShape(0.5f));

    float maxUpwardVelocity = 0.0f;
    for (int i = 0; i < 120; ++i) {
        world.Step(kDt);
        maxUpwardVelocity = std::max(maxUpwardVelocity, ball->GetLinearVelocity().y);
    }
    // 下落 4.5m 撞击速度约 9.4 m/s，弹起速度应接近 0.9 倍
    EXPECT_GT(maxUpwardVelocity, 7.0f);
}

TEST(World, FrictionHoldsBoxOnRamp) {
    // tan(15°) ≈ 0.27：摩擦 0.5 静止，摩擦 0 下滑
    for (float friction : {0.0f, 0.5f}) {
        World world;
        BodyDef rampDef;
        rampDef.type = BodyType::Static;
        rampDef.angle = DegToRad(-15.0f);
        rampDef.material.friction = friction;
        Body* ramp = world.CreateBody(rampDef, PolygonShape::MakeBox(10.0f, 0.2f));

        BodyDef boxDef;
        boxDef.angle = rampDef.angle;
        boxDef.position = ramp->GetWorldPoint({0.0f, 0.2f + 0.5f});
        boxDef.material.friction = friction;
        Body* box = world.CreateBody(boxDef, PolygonShape::MakeBox(0.5f, 0.5f));
        const Vec2 start = box->GetPosition();

        for (int i = 0; i < 120; ++i) world.Step(kDt);
        const float moved = Distance(start, box->GetPosition());
        if (friction == 0.0f) EXPECT_GT(moved, 1.0f);
        else EXPECT_LT(moved, 0.05f);
    }
}

TEST(World, DistanceJointKeepsLength) {
    World world;
    BodyDef anchorDef;
    anchorDef.type = BodyType::Static;
    anchorDef.position = {0.0f, 10.0f};
    Body* anchor = world.CreateBody(anchorDef, CircleShape(0.1f));

    BodyDef bobDef;
    bobDef.position = {3.0f, 10.0f};
    Body* bob = world.CreateBody(bobDef, CircleShape(0.25f));

    DistanceJointDef jd;
    jd.Initialize(anchor, bob, anchor->GetPosition(), bob->GetPosition());
    world.CreateJoint(jd);

    for (int i = 0; i < 240; ++i) {
        world.Step(kDt);
        EXPECT_NEAR(Distance(anchor->GetPosition(), bob->GetPosition()), 3.0f, 0.05f);
    }
}

TEST(World, RevoluteMotorSpinsWheel) {
    World world;
    BodyDef hubDef;
    hubDef.type = BodyType::Static;
    Body* hub = world.CreateBody(hubDef, CircleShape(0.1f));

    BodyDef wheelDef;
    wheelDef.angularDamping = 0.0f;
    wheelDef.gravityScale = 0.0f;
    Body* wheel = world.CreateBody(wheelDef, CircleShape(1.0f));

    RevoluteJointDef jd;
    jd.Initialize(hub, wheel, {0.0f, 0.0f});
    jd.enableMotor = true;
    jd.motorSpeed = 3.0f;
    jd.maxMotorTorque = 1000.0f;
    world.CreateJoint(jd);

    for (int i = 0; i < 60; ++i) world.Step(kDt);
    EXPECT_NEAR(wheel->GetAngularVelocity(), 3.0f, 0.01f);
    EXPECT_NEAR(wheel->GetPosition().x, 0.0f, 1e-3f);
}

TEST(World, CollisionFilterPreventsContact) {
    World world;
    AddGround(world);
    BodyDef def;
    def.position = {0.0f, 2.0f};
    def.maskBits = 0x0000;  // 不与任何东西碰撞
    Body* ghost = world.CreateBody(def, CircleShape(0.5f));
    for (int i = 0; i < 120; ++i) world.Step(kDt);
    EXPECT_LT(ghost->GetPosition().y, -1.0f);  // 穿过地面
}

TEST(World, DestroyBodyRemovesJointsAndContacts) {
    World world;
    AddGround(world);
    BodyDef def;
    def.position = {0.0f, 0.5f};
    Body* a = world.CreateBody(def, PolygonShape::MakeBox(0.5f, 0.5f));
    def.position = {0.0f, 1.5f};
    Body* b = world.CreateBody(def, PolygonShape::MakeBox(0.5f, 0.5f));
    RevoluteJointDef jd;
    jd.Initialize(a, b, {0.0f, 1.0f});
    world.CreateJoint(jd);
    world.Step(kDt);

    const BodyId id = a->GetId();
    world.DestroyBody(id);
    EXPECT_EQ(world.FindBody(id), nullptr);
    EXPECT_TRUE(world.GetJoints().empty());
    world.ForEachContact([&](const Contact& c) {
        EXPECT_NE(c.GetBodyA()->GetId(), id);
        EXPECT_NE(c.GetBodyB()->GetId(), id);
    });
    world.Step(kDt);  // 不应崩溃
}

TEST(World, KillPlaneRemovesFallingBodies) {
    WorldSettings s;
    s.killPlaneY = -10.0f;
    World world(s);
    BodyDef def;
    world.CreateBody(def, CircleShape(0.5f));
    for (int i = 0; i < 120; ++i) world.Step(kDt);
    EXPECT_TRUE(world.GetBodies().empty());
}

TEST(World, QueryAndRayCast) {
    World world;
    AddGround(world);
    BodyDef def;
    def.position = {3.0f, 2.0f};
    Body* ball = world.CreateBody(def, CircleShape(1.0f));

    EXPECT_EQ(world.QueryPoint({3.2f, 2.1f}), ball);
    EXPECT_NE(world.QueryPoint({-10.0f, -0.5f}), nullptr);  // 地面
    EXPECT_EQ(world.QueryPoint({-10.0f, 10.0f}), nullptr);

    RayHit hit;
    ASSERT_TRUE(world.RayCast({3.0f, 10.0f}, {3.0f, -10.0f}, hit));
    EXPECT_EQ(hit.body, ball);
    EXPECT_NEAR(hit.point.y, 3.0f, 1e-4f);
    EXPECT_NEAR(hit.normal.y, 1.0f, 1e-4f);
}

TEST(World, AllScenesRunWithoutNaN) {
    for (const auto& info : scene::GetScenes()) {
        World world;
        info.build(world);
        for (int i = 0; i < 120; ++i) world.Step(kDt);
        for (const auto& b : world.GetBodies()) {
            ASSERT_TRUE(b->GetPosition().IsValid()) << "scene " << info.id;
            ASSERT_TRUE(std::isfinite(b->GetAngle())) << "scene " << info.id;
        }
    }
}

TEST(Serializer, RoundTripPreservesWorld) {
    World world;
    scene::BuildBridge(world);
    for (int i = 0; i < 30; ++i) world.Step(kDt);

    const std::string text = scene::SerializeWorld(world);
    World copy;
    std::string error;
    ASSERT_TRUE(scene::DeserializeWorld(copy, text, &error)) << error;

    ASSERT_EQ(copy.GetBodies().size(), world.GetBodies().size());
    ASSERT_EQ(copy.GetJoints().size(), world.GetJoints().size());
    for (size_t i = 0; i < world.GetBodies().size(); ++i) {
        const Body& a = *world.GetBodies()[i];
        const Body& b = *copy.GetBodies()[i];
        EXPECT_EQ(a.GetType(), b.GetType());
        EXPECT_EQ(a.GetShapeType(), b.GetShapeType());
        EXPECT_NEAR(a.GetPosition().x, b.GetPosition().x, 1e-5f);
        EXPECT_NEAR(a.GetPosition().y, b.GetPosition().y, 1e-5f);
        EXPECT_NEAR(a.GetMass(), b.GetMass(), 1e-4f);
        EXPECT_EQ(a.GetUserData(), b.GetUserData());
    }
    // 关节的局部锚点也应保持一致
    for (size_t i = 0; i < world.GetJoints().size(); ++i) {
        const Joint& a = *world.GetJoints()[i];
        const Joint& b = *copy.GetJoints()[i];
        EXPECT_EQ(a.GetType(), b.GetType());
        EXPECT_NEAR(a.GetAnchorA().x, b.GetAnchorA().x, 1e-4f);
        EXPECT_NEAR(a.GetAnchorB().y, b.GetAnchorB().y, 1e-4f);
    }
}

TEST(Serializer, RejectsGarbage) {
    World world;
    std::string error;
    EXPECT_FALSE(scene::DeserializeWorld(world, "hello world", &error));
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(scene::DeserializeWorld(world, "kinema_scene version=999\n", &error));
}
