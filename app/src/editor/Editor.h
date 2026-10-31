// =============================================================================
// Editor：客户端的"大脑"
//
// 职责：
//   * 拥有模拟（Simulation）、渲染器、相机、各个面板
//   * 管理选择、工具、提示消息等编辑器状态
//   * 每帧构建整个 ImGui 界面：菜单栏 -> 停靠空间 -> 各面板 -> 状态栏 -> 弹窗
//
// 面板（Panel）只通过 Editor 的公共接口读写状态，彼此之间不直接依赖。
// =============================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "editor/EditorTypes.h"
#include "editor/Simulation.h"
#include "render/Camera2D.h"
#include "render/Framebuffer.h"
#include "render/Renderer2D.h"

namespace app {

class Panel;

class Editor {
public:
    Editor();
    ~Editor();

    Editor(const Editor&) = delete;
    Editor& operator=(const Editor&) = delete;

    bool Init(float dpiScale, const std::string& startScene);
    void Shutdown();

    /// 每帧调用（在 ImGui::NewFrame 与 ImGui::Render 之间）
    void OnFrame(float dt);

    // ---- 子系统访问 ---------------------------------------------------------
    Simulation& Sim() { return m_sim; }
    kinema::World& GetWorld() { return m_sim.GetWorld(); }
    Camera2D& GetCamera() { return m_camera; }
    Renderer2D& GetRenderer() { return m_renderer; }
    Framebuffer& GetFramebuffer() { return m_framebuffer; }
    SpawnSettings& Spawn() { return m_spawn; }
    DebugDrawSettings& Debug() { return m_debug; }

    // ---- 工具与选择 ---------------------------------------------------------
    [[nodiscard]] Tool GetTool() const { return m_tool; }
    void SetTool(Tool tool);

    [[nodiscard]] kinema::BodyId GetSelection() const { return m_selection; }
    [[nodiscard]] kinema::Body* GetSelectedBody() { return GetWorld().FindBody(m_selection); }
    void Select(kinema::BodyId id);
    void DeleteSelection();

    kinema::BodyId hoveredBody = kinema::kInvalidBodyId;
    kinema::JointId mouseJoint = kinema::kInvalidJointId;
    bool followSelection = false;

    // ---- 命令 ---------------------------------------------------------------
    void LoadScene(int index);
    void ResetScene();
    void ResetCamera();
    void FocusOn(const kinema::Body& body);
    void TogglePause();
    void StepOnce();

    void OpenSceneDialog() { m_openPopup = true; }
    void SaveSceneDialog() { m_savePopup = true; }
    void RequestScreenshot(const std::string& path) { m_screenshotRequest = path; }

    void Notify(const std::string& text);
    void AddEffect(const Effect& e) { m_effects.push_back(e); }
    [[nodiscard]] const std::vector<Effect>& GetEffects() const { return m_effects; }
    [[nodiscard]] const std::vector<Toast>& GetToasts() const { return m_toasts; }

    /// 生成物体时使用的颜色（随机或固定）
    uint64_t NextSpawnColor();

    // ---- 给 Application 用 -------------------------------------------------
    [[nodiscard]] bool WantsQuit() const { return m_quit; }
    void RequestQuit() { m_quit = true; }
    std::string ConsumeScreenshotRequest();
    [[nodiscard]] std::string WindowTitle() const;

    /// 物理统计历史（用于绘制曲线）
    struct History {
        static constexpr int kSize = 240;
        float stepMs[kSize] = {};
        float frameMs[kSize] = {};
        float energy[kSize] = {};
        float awake[kSize] = {};
        int offset = 0;
    };
    [[nodiscard]] const History& GetHistory() const { return m_history; }

    [[nodiscard]] float DpiScale() const { return m_dpiScale; }

private:
    void DrawMenuBar();
    void DrawStatusBar();
    void DrawDockSpace();
    void BuildDefaultLayout(unsigned int dockspaceId);
    void DrawPopups();
    void HandleShortcuts();
    void UpdateFrameState(float dt);

    Simulation m_sim;
    Camera2D m_camera;
    Renderer2D m_renderer;
    Framebuffer m_framebuffer;
    SpawnSettings m_spawn;
    DebugDrawSettings m_debug;

    std::vector<std::unique_ptr<Panel>> m_panels;

    Tool m_tool = Tool::Select;
    kinema::BodyId m_selection = kinema::kInvalidBodyId;

    std::vector<Toast> m_toasts;
    std::vector<Effect> m_effects;
    History m_history;

    float m_dpiScale = 1.0f;
    bool m_resetLayout = false;
    bool m_openPopup = false;
    bool m_savePopup = false;
    bool m_showAbout = false;
    bool m_showControls = false;
    bool m_showImGuiDemo = false;
    bool m_quit = false;
    std::string m_screenshotRequest;
    char m_pathBuffer[512] = "scene.kscene";
    unsigned int m_colorSeed = 1;
};

}  // namespace app
