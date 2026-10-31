// =============================================================================
// 场景序列化：把 World 保存为人类可读的文本文件（*.kscene），并能读回来。
//
// 文件格式（每行一条记录，"类型 key=value key=value ..."）：
//
//   kinema_scene version=1
//   world gx=0 gy=-9.81 iterations=10 warm=1 sleep=1
//   body id=1 type=static x=0 y=-1 angle=0 ... shape=polygon verts=-40,-1;40,-1;40,1;-40,1
//   body id=2 type=dynamic x=0 y=5 ... shape=circle radius=0.5
//   joint type=revolute a=1 b=2 ax=0 ay=0 bx=0 by=0 motor=1 speed=1.2 torque=100
//
// 选用 key=value 文本格式的原因：
//   * 便于调试（直接用文本编辑器打开）
//   * 向前兼容（读取时忽略未知字段，新增字段不会破坏旧文件）
// =============================================================================
#pragma once

#include <string>

#include "kinema/dynamics/World.h"

namespace kinema::scene {

inline constexpr int kSceneFormatVersion = 1;

/// 把世界序列化为字符串（不包含鼠标关节等临时对象）
std::string SerializeWorld(const World& world);

/// 从字符串恢复世界。会先清空 world。失败时返回 false 并写入 error。
bool DeserializeWorld(World& world, const std::string& text, std::string* error = nullptr);

bool SaveWorldToFile(const World& world, const std::string& path, std::string* error = nullptr);
bool LoadWorldFromFile(World& world, const std::string& path, std::string* error = nullptr);

}  // namespace kinema::scene
