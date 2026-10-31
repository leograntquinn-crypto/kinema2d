// =============================================================================
// DebugDraw：把 World 的状态画出来
//
// 物理引擎本身不知道"画"这件事；这里读取引擎的公开数据（物体、接触、关节），
// 用 Renderer2D 绘制。这种"引擎与渲染解耦"是物理引擎的通用设计。
// =============================================================================
#pragma once

#include <kinema/Kinema.h>

#include "editor/EditorTypes.h"
#include "editor/Theme.h"
#include "render/Camera2D.h"
#include "render/Renderer2D.h"

namespace app {

/// 物体的显示颜色：优先使用 userData 中的颜色，否则按 ID 从调色板选取
Color BodyColor(const kinema::Body& body);

void DrawGrid(Renderer2D& r, const Camera2D& camera, const ViewportPalette& palette);

void DrawWorld(Renderer2D& r, const Camera2D& camera, const kinema::World& world, const DebugDrawSettings& settings,
               const ViewportPalette& palette, kinema::BodyId selected, kinema::BodyId hovered);

/// 绘制单个形状（也用于工具预览）
void DrawShape(Renderer2D& r, const kinema::Shape& shape, const kinema::Transform& xf, Color fill, Color outline,
               float outlineWidth, bool rotationMarker);

}  // namespace app
