// =============================================================================
// 编辑器中各模块共享的小型数据结构
// =============================================================================
#pragma once

#include <string>

#include <kinema/Kinema.h>

namespace app {

/// 视口中的交互工具
enum class Tool : int {
    Select = 0,  // 选择 / 拖拽（运行时用 MouseJoint，暂停时直接移动）
    Circle,      // 点击生成圆（拖动 = 弹弓发射）
    Box,         // 点击生成方块
    Polygon,     // 点击生成随机凸多边形
    StaticBox,   // 拖拽矩形生成静态方块
    Joint,       // 从物体 A 拖到物体 B 生成关节
    Explode,     // 点击产生爆炸冲量
    Ray,         // 拖拽发射射线，显示命中点与法线
    Delete,      // 点击删除物体
    Count
};

const char* ToolName(Tool tool);
const char* ToolHint(Tool tool);

enum class JointKind : int { Rod = 0, Spring = 1, Hinge = 2 };

/// 生成物体时使用的参数（在 Settings 面板中编辑）
struct SpawnSettings {
    int bodyType = static_cast<int>(kinema::BodyType::Dynamic);
    float circleRadius = 0.5f;
    float boxHalfWidth = 0.5f;
    float boxHalfHeight = 0.5f;
    int polygonSides = 0;  // 0 = 随机凸多边形
    float polygonRadius = 0.6f;
    kinema::Material material;
    bool randomColor = true;
    float color[3] = {0.31f, 0.55f, 1.0f};
    float throwScale = 4.0f;  // 弹弓：每拖动 1 米给多少 m/s 速度

    JointKind jointKind = JointKind::Rod;
    float springHz = 3.0f;
    float springDamping = 0.3f;
    bool jointCollide = false;

    float explosionRadius = 6.0f;
    float explosionStrength = 14.0f;
};

/// 调试绘制开关
struct DebugDrawSettings {
    bool shapes = true;
    bool outlines = true;
    bool grid = true;
    bool joints = true;
    bool sleepTint = true;
    bool aabbs = false;
    bool contacts = false;
    bool normals = false;
    bool impulses = false;
    bool centers = false;
    bool velocities = false;
    bool rotationMarkers = true;
    float fillAlpha = 0.7f;
    int msaaSamples = 4;
};

/// 屏幕上短暂显示的提示
struct Toast {
    std::string text;
    float timeLeft = 0.0f;
};

/// 一次性的视觉特效（例如爆炸波纹）
struct Effect {
    kinema::Vec2 position;
    float radius = 1.0f;
    float age = 0.0f;
    float life = 0.5f;
};

}  // namespace app
