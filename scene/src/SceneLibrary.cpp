#include "kinema_scene/SceneLibrary.h"

#include <cmath>
#include <random>
#include <vector>

#include "kinema_scene/Color.h"

namespace kinema::scene {

namespace {

using namespace palette;

Body* AddBox(World& world, Vec2 pos, float hw, float hh, uint64_t color, float angle = 0.0f,
             BodyType type = BodyType::Dynamic) {
    BodyDef def;
    def.type = type;
    def.position = pos;
    def.angle = angle;
    def.userData = color;
    return world.CreateBody(def, PolygonShape::MakeBox(hw, hh));
}

Body* AddCircle(World& world, Vec2 pos, float radius, uint64_t color, BodyType type = BodyType::Dynamic) {
    BodyDef def;
    def.type = type;
    def.position = pos;
    def.userData = color;
    return world.CreateBody(def, CircleShape(radius));
}

/// 沿 p0 -> p1 放一段静态"线段"（细长方块，上表面恰好落在该线上）
Body* AddStaticSegment(World& world, Vec2 p0, Vec2 p1, float thickness = 0.25f, uint64_t color = Ground) {
    const Vec2 d = p1 - p0;
    const float len = d.Length();
    const float angle = std::atan2(d.y, d.x);
    const Vec2 normal{-std::sin(angle), std::cos(angle)};
    const Vec2 center = 0.5f * (p0 + p1) - (0.5f * thickness) * normal;
    return AddBox(world, center, 0.5f * len, 0.5f * thickness, color, angle, BodyType::Static);
}

/// 随机凸多边形/圆，用于"大量物体"场景
Body* AddRandomShape(World& world, std::mt19937& rng, Vec2 pos, float minSize, float maxSize) {
    std::uniform_real_distribution<float> size(minSize, maxSize);
    std::uniform_int_distribution<int> kind(0, 3);
    std::uniform_int_distribution<int> color(0, kCycleCount - 1);
    std::uniform_real_distribution<float> angle(0.0f, 2.0f * kPi);

    BodyDef def;
    def.position = pos;
    def.angle = angle(rng);
    def.userData = Pick(color(rng));

    const float s = size(rng);
    switch (kind(rng)) {
        case 0: return world.CreateBody(def, CircleShape(s));
        case 1: return world.CreateBody(def, PolygonShape::MakeBox(s, s * 0.8f));
        case 2: return world.CreateBody(def, PolygonShape::MakeRegular(3, s * 1.2f));
        default: return world.CreateBody(def, PolygonShape::MakeRegular(6, s));
    }
}

void AddWalls(World& world, float halfWidth, float height) {
    CreateGround(world, {-halfWidth - 0.5f, height * 0.5f}, 0.5f, height * 0.5f);
    CreateGround(world, {halfWidth + 0.5f, height * 0.5f}, 0.5f, height * 0.5f);
}

}  // namespace

Body* CreateGround(World& world, Vec2 center, float halfWidth, float halfHeight, float angle) {
    return AddBox(world, center, halfWidth, halfHeight, Ground, angle, BodyType::Static);
}

// =============================================================================
void BuildSandbox(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 40.0f, 1.0f);
    AddWalls(world, 40.0f, 30.0f);
}

void BuildPyramid(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 40.0f, 1.0f);
    constexpr int kRows = 20;
    constexpr float kHalf = 0.5f;
    constexpr float kGap = 0.05f;
    for (int row = 0; row < kRows; ++row) {
        const int count = kRows - row;
        const float startX = -0.5f * static_cast<float>(count - 1) * (2.0f * kHalf + kGap);
        for (int i = 0; i < count; ++i) {
            const Vec2 pos{startX + static_cast<float>(i) * (2.0f * kHalf + kGap), kHalf + static_cast<float>(row) * 2.0f * kHalf};
            AddBox(world, pos, kHalf, kHalf, Pick(row));
        }
    }
}

void BuildTower(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 40.0f, 1.0f);

    // 两座塔
    for (int i = 0; i < 16; ++i) AddBox(world, {0.0f, 0.5f + static_cast<float>(i)}, 0.75f, 0.5f, Pick(i));
    for (int i = 0; i < 12; ++i) AddBox(world, {5.0f, 0.5f + static_cast<float>(i)}, 0.75f, 0.5f, Pick(i + 3));

    // 破坏球：一根刚性距离关节吊着一个重球，从水平位置释放
    Body* anchor = AddCircle(world, {-1.0f, 24.0f}, 0.25f, Steel, BodyType::Static);
    BodyDef ballDef;
    ballDef.position = {-17.0f, 24.0f};
    ballDef.material.density = 6.0f;
    ballDef.userData = Steel;
    Body* ball = world.CreateBody(ballDef, CircleShape(1.4f));

    DistanceJointDef jd;
    jd.Initialize(anchor, ball, anchor->GetPosition(), ball->GetPosition());
    world.CreateJoint(jd);
}

void BuildNewtonsCradle(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 30.0f, 1.0f);
    Body* bar = CreateGround(world, {0.0f, 12.2f}, 4.0f, 0.2f);

    constexpr int kBalls = 5;
    constexpr float kRadius = 0.5f;
    constexpr float kLength = 6.0f;
    for (int i = 0; i < kBalls; ++i) {
        const float x = (static_cast<float>(i) - 2.0f) * (2.0f * kRadius + 0.002f);
        const Vec2 anchor{x, 12.0f};
        Vec2 pos{x, 12.0f - kLength};
        if (i == 0) {
            const float a = DegToRad(55.0f);  // 第一个球拉开 55°
            pos = anchor + kLength * Vec2{-std::sin(a), -std::cos(a)};
        }

        BodyDef def;
        def.position = pos;
        def.material = {2.0f, 0.0f, 1.0f};  // 无摩擦、完全弹性
        def.angularDamping = 0.0f;
        def.allowSleep = false;
        def.userData = Pick(i);
        Body* ball = world.CreateBody(def, CircleShape(kRadius));

        DistanceJointDef jd;
        jd.Initialize(bar, ball, anchor, pos);
        world.CreateJoint(jd);
    }
}

void BuildBridge(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 40.0f, 1.0f);
    Body* left = CreateGround(world, {-13.0f, 4.0f}, 2.0f, 4.0f);
    Body* right = CreateGround(world, {13.0f, 4.0f}, 2.0f, 4.0f);

    constexpr int kPlanks = 20;
    constexpr float kSpan = 22.0f;
    constexpr float kPlankLen = kSpan / kPlanks;
    constexpr float kY = 7.875f;

    Body* prev = left;
    for (int i = 0; i < kPlanks; ++i) {
        const float x = -11.0f + kPlankLen * (static_cast<float>(i) + 0.5f);
        BodyDef def;
        def.position = {x, kY};
        def.material.density = 2.0f;
        def.material.friction = 0.6f;
        def.userData = i % 2 ? Orange : Yellow;
        Body* plank = world.CreateBody(def, PolygonShape::MakeBox(0.5f * kPlankLen, 0.125f));

        RevoluteJointDef jd;
        jd.Initialize(prev, plank, {-11.0f + kPlankLen * static_cast<float>(i), kY});
        world.CreateJoint(jd);
        prev = plank;
    }
    RevoluteJointDef last;
    last.Initialize(prev, right, {11.0f, kY});
    world.CreateJoint(last);

    // 往桥上丢点东西
    for (int i = 0; i < 6; ++i) {
        AddBox(world, {-8.0f + 3.0f * static_cast<float>(i), 12.0f + static_cast<float>(i % 2)}, 0.5f, 0.5f, Pick(i));
        AddCircle(world, {-6.5f + 3.0f * static_cast<float>(i), 14.0f}, 0.4f, Pick(i + 4));
    }
}

void BuildChain(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 40.0f, 1.0f);

    constexpr int kLinks = 26;
    constexpr float kLinkLen = 0.8f;
    constexpr float kY = 24.0f;
    Body* anchor = AddBox(world, {0.0f, kY}, 0.3f, 0.3f, Steel, 0.0f, BodyType::Static);

    Body* prev = anchor;
    for (int i = 0; i < kLinks; ++i) {
        BodyDef def;
        def.position = {kLinkLen * (static_cast<float>(i) + 0.5f), kY};
        def.userData = i % 2 ? Cyan : Blue;
        Body* link = world.CreateBody(def, PolygonShape::MakeBox(0.5f * kLinkLen, 0.1f));

        RevoluteJointDef jd;
        jd.Initialize(prev, link, {kLinkLen * static_cast<float>(i), kY});
        world.CreateJoint(jd);
        prev = link;
    }

    // 末端重球
    BodyDef ballDef;
    ballDef.position = {kLinkLen * kLinks + 1.0f, kY};
    ballDef.material.density = 4.0f;
    ballDef.userData = Coral;
    Body* ball = world.CreateBody(ballDef, CircleShape(1.0f));
    RevoluteJointDef jd;
    jd.Initialize(prev, ball, {kLinkLen * kLinks, kY});
    world.CreateJoint(jd);

    // 地上一排箱子等着被撞
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 3; ++j) {
            AddBox(world, {-14.0f + 1.1f * static_cast<float>(i), 0.5f + static_cast<float>(j)}, 0.5f, 0.5f, Pick(i + j));
        }
    }
}

void BuildDominoes(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 40.0f, 1.0f);

    constexpr int kCount = 36;
    for (int i = 0; i < kCount; ++i) {
        BodyDef def;
        def.position = {-18.0f + static_cast<float>(i), 1.0f};
        def.material.friction = 0.6f;
        def.userData = Pick(i / 4);
        if (i == 0) {
            def.angle = -0.25f;  // 第一块稍微倾斜，并给一点角速度
            def.angularVelocity = -2.0f;
        }
        world.CreateBody(def, PolygonShape::MakeBox(0.1f, 1.0f));
    }

    // 终点的台阶与小球
    for (int i = 0; i < 4; ++i) {
        AddBox(world, {19.5f + 0.0f, 0.5f + static_cast<float>(i)}, 0.5f, 0.5f, Steel);
    }
    AddCircle(world, {19.5f, 4.5f}, 0.5f, Coral);
}

void BuildFunnel(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 18.0f, 1.0f);
    AddWalls(world, 16.0f, 54.0f);

    // V 形漏斗
    AddStaticSegment(world, {-16.0f, 22.0f}, {-1.8f, 13.0f}, 0.5f);
    AddStaticSegment(world, {1.8f, 13.0f}, {16.0f, 22.0f}, 0.5f);

    std::mt19937 rng(42);  // 固定种子：每次加载结果完全一致，方便对比
    for (int row = 0; row < 20; ++row) {
        for (int col = 0; col < 20; ++col) {
            const Vec2 pos{-11.0f + 1.15f * static_cast<float>(col), 27.0f + 1.15f * static_cast<float>(row)};
            AddRandomShape(world, rng, pos, 0.22f, 0.42f);
        }
    }
}

void BuildWindmill(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 16.0f, 1.0f);
    AddWalls(world, 15.0f, 24.0f);

    Body* hub = AddCircle(world, {0.0f, 9.0f}, 0.35f, Steel, BodyType::Static);

    // 两片叶片分别用带马达的旋转关节连接到轴心；
    // 用碰撞过滤让两片叶片之间不碰撞（category 0x2 不与 0x2 碰撞）
    for (int i = 0; i < 2; ++i) {
        BodyDef def;
        def.position = hub->GetPosition();
        def.angle = static_cast<float>(i) * 0.5f * kPi;
        def.material.density = 2.0f;
        def.categoryBits = 0x0002;
        def.maskBits = 0xFFFF & ~0x0002;
        def.userData = i ? Purple : Pink;
        Body* blade = world.CreateBody(def, PolygonShape::MakeBox(7.0f, 0.3f));

        RevoluteJointDef jd;
        jd.Initialize(hub, blade, hub->GetPosition());
        jd.enableMotor = true;
        jd.motorSpeed = 1.2f;
        jd.maxMotorTorque = 20000.0f;
        world.CreateJoint(jd);
    }

    std::mt19937 rng(7);
    for (int i = 0; i < 160; ++i) {
        const Vec2 pos{-13.0f + static_cast<float>(i % 26), 18.0f + 1.1f * static_cast<float>(i / 26)};
        AddRandomShape(world, rng, pos, 0.25f, 0.4f);
    }
}

void BuildCar(World& world) {
    // 起伏的地形：由许多静态线段拼成
    Vec2 prev{-40.0f, 0.0f};
    for (float x = -37.0f; x <= 160.0f; x += 3.0f) {
        float y = 0.0f;
        if (x > -10.0f) y = 1.6f * std::sin(0.18f * x) + 0.6f * std::sin(0.53f * x);
        if (x > 100.0f && x < 125.0f) y = 0.6f * (x - 100.0f);  // 坡道
        if (x >= 125.0f) y = -2.0f;
        const Vec2 p{x, y};
        AddStaticSegment(world, prev, p, 0.5f);
        prev = p;
    }
    CreateGround(world, {-41.0f, 5.0f}, 1.0f, 6.0f);

    // 车身
    BodyDef chassisDef;
    chassisDef.position = {-30.0f, 2.2f};
    chassisDef.userData = Coral;
    PolygonShape chassisShape;
    chassisShape.Set({{-1.6f, -0.35f}, {1.6f, -0.35f}, {1.5f, 0.15f}, {0.5f, 0.65f}, {-1.0f, 0.65f}, {-1.6f, 0.1f}});
    Body* chassis = world.CreateBody(chassisDef, chassisShape);

    // 车轮：带马达的旋转关节（负速度 = 顺时针 = 向右开）
    for (float side : {-1.0f, 1.0f}) {
        BodyDef wheelDef;
        wheelDef.position = chassis->GetWorldPoint({1.05f * side, -0.6f});
        wheelDef.material = {1.0f, 1.2f, 0.1f};
        wheelDef.userData = Steel;
        Body* wheel = world.CreateBody(wheelDef, CircleShape(0.5f));

        RevoluteJointDef jd;
        jd.Initialize(chassis, wheel, wheel->GetPosition());
        jd.enableMotor = true;
        jd.motorSpeed = -12.0f;
        jd.maxMotorTorque = 40.0f;
        world.CreateJoint(jd);
    }

    // 路上的障碍物
    for (int i = 0; i < 6; ++i) AddBox(world, {20.0f + 0.0f, 4.0f + static_cast<float>(i)}, 0.4f, 0.4f, Pick(i));
    for (int i = 0; i < 10; ++i) AddCircle(world, {60.0f + 0.9f * static_cast<float>(i), 6.0f}, 0.35f, Pick(i));
}

void BuildMaterials(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 30.0f, 1.0f);

    // 左侧：弹性系数 0.0 ~ 1.0 的小球从同一高度落下
    for (int i = 0; i <= 5; ++i) {
        BodyDef def;
        def.position = {-15.0f + 2.0f * static_cast<float>(i), 12.0f};
        def.material.restitution = 0.2f * static_cast<float>(i);
        def.allowSleep = false;
        def.userData = Pick(i);
        world.CreateBody(def, CircleShape(0.5f));
    }

    // 右侧：15° 斜坡上，摩擦系数 0.0 / 0.1 / 0.2 / 0.3 / 0.5 的方块
    // tan(15°) ≈ 0.27，所以摩擦 >= 0.3 的方块会静止不动
    const float frictions[] = {0.0f, 0.1f, 0.2f, 0.3f, 0.5f};
    const float angle = DegToRad(-15.0f);
    for (int i = 0; i < 5; ++i) {
        const float y = 16.0f - 3.4f * static_cast<float>(i);
        Body* ramp = CreateGround(world, {8.0f, y}, 6.0f, 0.15f, angle);
        ramp->SetFriction(frictions[i]);

        BodyDef def;
        def.position = ramp->GetWorldPoint({-4.5f, 0.15f + 0.4f + 0.01f});
        def.angle = angle;
        def.material.friction = frictions[i];
        def.userData = Pick(i);
        world.CreateBody(def, PolygonShape::MakeBox(0.4f, 0.4f));
    }
}

void BuildSoftBody(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 30.0f, 1.0f);
    CreateGround(world, {-6.0f, 4.0f}, 6.0f, 0.2f, DegToRad(-12.0f));

    // 用弹簧（软距离关节）把粒子网格连接起来 => 果冻
    auto makeBlob = [&world](Vec2 origin, int n, uint64_t color) {
        constexpr float kSpacing = 0.5f;
        std::vector<Body*> grid(static_cast<size_t>(n * n));
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                BodyDef def;
                def.position = origin + Vec2{kSpacing * static_cast<float>(x), kSpacing * static_cast<float>(y)};
                def.userData = color;
                def.material.friction = 0.8f;
                grid[static_cast<size_t>(y * n + x)] = world.CreateBody(def, CircleShape(0.22f));
            }
        }
        auto link = [&world](Body* a, Body* b) {
            DistanceJointDef jd;
            jd.Initialize(a, b, a->GetPosition(), b->GetPosition());
            jd.frequencyHz = 8.0f;
            jd.dampingRatio = 0.4f;
            world.CreateJoint(jd);
        };
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                Body* b = grid[static_cast<size_t>(y * n + x)];
                if (x + 1 < n) link(b, grid[static_cast<size_t>(y * n + x + 1)]);
                if (y + 1 < n) link(b, grid[static_cast<size_t>((y + 1) * n + x)]);
                if (x + 1 < n && y + 1 < n) link(b, grid[static_cast<size_t>((y + 1) * n + x + 1)]);
                if (x > 0 && y + 1 < n) link(b, grid[static_cast<size_t>((y + 1) * n + x - 1)]);
            }
        }
    };

    makeBlob({-9.0f, 8.0f}, 6, Green);
    makeBlob({3.0f, 4.0f}, 5, Purple);
    makeBlob({-1.0f, 13.0f}, 7, Cyan);
}

void BuildMixer(World& world) {
    CreateGround(world, {0.0f, -1.0f}, 9.0f, 1.0f);
    AddWalls(world, 8.0f, 16.0f);

    // 运动学物体：不受力，按设定角速度永远旋转
    BodyDef paddle;
    paddle.type = BodyType::Kinematic;
    paddle.position = {0.0f, 6.8f};
    paddle.angularVelocity = 1.5f;
    paddle.userData = Purple;
    world.CreateBody(paddle, PolygonShape::MakeBox(6.0f, 0.3f));

    std::mt19937 rng(3);
    for (int i = 0; i < 140; ++i) {
        const Vec2 pos{-6.5f + static_cast<float>(i % 14), 14.0f + 1.0f * static_cast<float>(i / 14)};
        AddRandomShape(world, rng, pos, 0.2f, 0.38f);
    }
}

// =============================================================================
const std::vector<SceneInfo>& GetScenes() {
    static const std::vector<SceneInfo> scenes = {
        {"sandbox", "Sandbox", "Empty playground. Use the toolbar to spawn bodies.", BuildSandbox, {0.0f, 12.0f}, 32.0f},
        {"pyramid", "Pyramid", "210 stacked boxes: tests solver stability, warm starting and sleeping.", BuildPyramid, {0.0f, 10.0f}, 26.0f},
        {"tower", "Wrecking Ball", "A heavy ball on a rigid distance joint smashes two towers.", BuildTower, {-2.0f, 12.0f}, 32.0f},
        {"cradle", "Newton's Cradle", "Elastic balls (restitution = 1) hanging on distance joints.", BuildNewtonsCradle, {0.0f, 8.0f}, 16.0f},
        {"bridge", "Rope Bridge", "Planks connected with revolute joints between two pillars.", BuildBridge, {0.0f, 8.0f}, 22.0f},
        {"chain", "Chain", "A long chain of revolute joints with a heavy ball at the end.", BuildChain, {4.0f, 13.0f}, 34.0f},
        {"dominoes", "Dominoes", "Classic chain reaction - friction and rotational dynamics.", BuildDominoes, {0.0f, 5.0f}, 22.0f},
        {"funnel", "Funnel (400 bodies)", "Many mixed shapes poured through a funnel: a broad-phase benchmark.",
         BuildFunnel, {0.0f, 25.0f}, 56.0f},
        {"windmill", "Windmill", "Motorized revolute joints + collision filtering between the blades.",
         BuildWindmill, {0.0f, 11.0f}, 27.0f},
        {"car", "Car", "Motorized wheels on rough terrain. Select the chassis and enable Follow.", BuildCar, {-24.0f, 4.0f}, 18.0f},
        {"materials", "Materials", "Left: restitution 0.0..1.0. Right: friction 0.0..0.5 on 15-degree ramps.",
         BuildMaterials, {0.0f, 9.0f}, 22.0f},
        {"softbody", "Soft Bodies", "Particle grids linked by soft distance joints (springs).", BuildSoftBody, {-1.0f, 8.0f}, 20.0f},
        {"mixer", "Mixer", "A kinematic paddle stirring dynamic bodies.", BuildMixer, {0.0f, 9.0f}, 22.0f},
    };
    return scenes;
}

int FindScene(const std::string& id) {
    const auto& scenes = GetScenes();
    for (size_t i = 0; i < scenes.size(); ++i) {
        if (scenes[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

}  // namespace kinema::scene
