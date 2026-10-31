// =============================================================================
// 面板基类与所有面板的声明
//
// 每个面板是一个可停靠的 ImGui 窗口。窗口名使用  "<翻译标题>###<稳定ID>"，
// 这样切换语言时窗口的停靠位置不会丢失。
// =============================================================================
#pragma once

#include <memory>
#include <string>

#include <kinema/collision/Shape.h>
#include <kinema/math/Math.h>

#include "editor/EditorTypes.h"

#include "util/I18n.h"

namespace app {

class Editor;

class Panel {
public:
    Panel(const char* id, const char* title) : m_id(id), m_title(title) {}
    virtual ~Panel() = default;

    virtual void OnImGui(Editor& editor) = 0;

    [[nodiscard]] const char* Id() const { return m_id; }
    [[nodiscard]] const char* Title() const { return m_title; }
    [[nodiscard]] std::string WindowName() const { return TrId(m_title, m_id); }

    bool open = true;

private:
    const char* m_id;
    const char* m_title;
};

// ---- 具体面板 ---------------------------------------------------------------

class ViewportPanel final : public Panel {
public:
    ViewportPanel() : Panel("Viewport", "Viewport") {}
    void OnImGui(Editor& editor) override;

private:
    void DrawToolbar(Editor& editor);
    void HandleInput(Editor& editor, kinema::Vec2 canvasPos, kinema::Vec2 canvasSize, bool hovered);
    void DrawOverlay(Editor& editor, kinema::Vec2 canvasPos, kinema::Vec2 canvasSize);
    void RenderScene(Editor& editor);
    /// 生成工具的预览形状：设置改变（或 force）时重新生成；生成的物体就使用这个形状
    void UpdateGhost(const SpawnSettings& spawn, Tool tool, bool force);

    // 拖拽状态
    bool m_dragging = false;
    int m_dragButton = -1;
    kinema::Vec2 m_dragStartWorld;
    kinema::Vec2 m_dragStartScreen;
    kinema::Vec2 m_mouseWorld;
    kinema::Vec2 m_mouseScreen;
    bool m_mouseInside = false;
    kinema::Vec2 m_moveOffset;           // 暂停时直接移动物体：抓取点相对质心的偏移
    kinema::BodyId m_jointStartBody = 0;  // 关节工具的起点物体
    bool m_rayActive = false;
    kinema::Vec2 m_rayStart, m_rayEnd;

    kinema::Vec2 m_lastCanvasSize;

    std::unique_ptr<kinema::Shape> m_ghost;
    Tool m_ghostTool = Tool::Count;
    float m_ghostKey[5] = {};
};

class ScenesPanel final : public Panel {
public:
    ScenesPanel() : Panel("Scenes", "Scenes") {}
    void OnImGui(Editor& editor) override;
};

class OutlinerPanel final : public Panel {
public:
    OutlinerPanel() : Panel("Outliner", "Outliner") {}
    void OnImGui(Editor& editor) override;

private:
    char m_filter[64] = "";
    int m_typeFilter = 0;  // 0 = 全部
    bool m_scrollToSelection = false;
    unsigned int m_lastSelection = 0;
};

class InspectorPanel final : public Panel {
public:
    InspectorPanel() : Panel("Inspector", "Inspector") {}
    void OnImGui(Editor& editor) override;
};

class SettingsPanel final : public Panel {
public:
    SettingsPanel() : Panel("Settings", "Settings") {}
    void OnImGui(Editor& editor) override;
};

class StatsPanel final : public Panel {
public:
    StatsPanel() : Panel("Stats", "Performance") {}
    void OnImGui(Editor& editor) override;
};

class ConsolePanel final : public Panel {
public:
    ConsolePanel() : Panel("Console", "Console") {}
    void OnImGui(Editor& editor) override;

private:
    bool m_autoScroll = true;
    size_t m_lastCount = 0;
};

}  // namespace app
