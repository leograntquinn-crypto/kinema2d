// =============================================================================
// 主题：ImGui 配色 + 视口配色 + 字体加载
// =============================================================================
#pragma once

#include <imgui.h>

#include "render/Color.h"

namespace app {

enum class ThemeMode { Dark = 0, Light = 1 };

/// 视口（物理场景）使用的颜色
struct ViewportPalette {
    Color background;
    Color gridMinor;
    Color gridMajor;
    Color axisX;
    Color axisY;
    Color staticFill;
    Color selection;
    Color hover;
    Color contactPoint;
    Color contactNormal;
    Color impulse;
    Color joint;
    Color aabb;
    Color velocity;
    Color toolPreview;
};

namespace theme {

void Apply(ThemeMode mode, float dpiScale);
[[nodiscard]] ThemeMode Current();
[[nodiscard]] const ViewportPalette& Palette();
[[nodiscard]] ImVec4 Accent();
[[nodiscard]] ImU32 AccentU32(float alpha = 1.0f);

/// 加载界面字体（自动寻找系统字体），返回是否成功加载了中文字体
bool LoadFonts(float dpiScale);
[[nodiscard]] bool HasCjkFont();
/// 较大号字体（用于标题），可能为 nullptr
[[nodiscard]] ImFont* LargeFont();

}  // namespace theme

}  // namespace app
