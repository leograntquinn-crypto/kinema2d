// =============================================================================
// 示例场景库
//
// 每个场景就是一个"往 World 里放东西"的函数。
// 客户端、单元测试、基准测试共享同一套场景，保证看到的和测到的一致。
// =============================================================================
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "kinema/dynamics/World.h"

namespace kinema::scene {

struct SceneInfo {
    std::string id;           // 稳定的标识符，用于命令行参数 / 配置
    std::string name;         // 显示名称
    std::string description;  // 一句话说明这个场景演示了什么
    std::function<void(World&)> build;
    Vec2 cameraCenter{0.0f, 10.0f};  // 推荐的初始相机位置
    float viewHeight = 30.0f;        // 推荐的初始视野高度（米）
};

/// 所有内置场景
const std::vector<SceneInfo>& GetScenes();

/// 按 id 查找，找不到返回 -1
int FindScene(const std::string& id);

/// 创建一个静态地面
Body* CreateGround(World& world, Vec2 center, float halfWidth, float halfHeight, float angle = 0.0f);

// ---- 各个场景的构建函数（也可以单独调用） ----
void BuildSandbox(World& world);
void BuildPyramid(World& world);
void BuildTower(World& world);
void BuildNewtonsCradle(World& world);
void BuildBridge(World& world);
void BuildChain(World& world);
void BuildDominoes(World& world);
void BuildFunnel(World& world);
void BuildWindmill(World& world);
void BuildCar(World& world);
void BuildMaterials(World& world);
void BuildSoftBody(World& world);
void BuildMixer(World& world);

}  // namespace kinema::scene
