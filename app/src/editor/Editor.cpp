#include "editor/Editor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder API、BeginViewportSideBar
#include <kinema_scene/Color.h>
#include <kinema_scene/SceneLibrary.h>

#include "editor/Theme.h"
#include "editor/panels/Panel.h"
#include "util/I18n.h"
#include "util/Log.h"

namespace app {

// =============================================================================
// 工具名称
// =============================================================================
const char* ToolName(Tool tool) {
    switch (tool) {
        case Tool::Select: return "Select";
        case Tool::Circle: return "Circle";
        case Tool::Box: return "Box";
        case Tool::Polygon: return "Polygon";
        case Tool::StaticBox: return "Static";
        case Tool::Joint: return "Joint";
        case Tool::Explode: return "Explode";
        case Tool::Ray: return "Ray";
        case Tool::Delete: return "Delete";
        default: return "?";
    }
}

const char* ToolHint(Tool tool) {
    switch (tool) {
        case Tool::Select: return "Click to select. Drag to move bodies (mouse joint while running).";
        case Tool::Circle: return "Click to spawn a circle. Drag to launch it like a slingshot.";
        case Tool::Box: return "Click to spawn a box. Drag to launch it like a slingshot.";
        case Tool::Polygon: return "Click to spawn a convex polygon. Drag to launch it.";
        case Tool::StaticBox: return "Drag a rectangle to create a static box.";
        case Tool::Joint: return "Drag from one body to another to connect them.";
        case Tool::Explode: return "Click to apply a radial explosion impulse.";
        case Tool::Ray: return "Drag to cast a ray and see the hit point and normal.";
        case Tool::Delete: return "Click a body to delete it.";
        default: return "";
    }
}

// =============================================================================
Editor::Editor() = default;
Editor::~Editor() = default;

bool Editor::Init(float dpiScale, const std::string& startScene) {
    m_dpiScale = dpiScale;

    std::string error;
    if (!m_renderer.Init(&error)) {
        Log::Error("Renderer init failed: %s", error.c_str());
        return false;
    }

    m_panels.push_back(std::make_unique<ViewportPanel>());
    m_panels.push_back(std::make_unique<ScenesPanel>());
    m_panels.push_back(std::make_unique<OutlinerPanel>());
    m_panels.push_back(std::make_unique<InspectorPanel>());
    m_panels.push_back(std::make_unique<SettingsPanel>());
    m_panels.push_back(std::make_unique<ConsolePanel>());
    m_panels.push_back(std::make_unique<StatsPanel>());  // 同一停靠节点中最后提交的窗口默认显示

    int index = kinema::scene::FindScene(startScene);
    if (index < 0) {
        if (!startScene.empty()) Log::Warn("Unknown scene '%s', loading default", startScene.c_str());
        index = kinema::scene::FindScene("pyramid");
    }
    LoadScene(index);
    return true;
}

void Editor::Shutdown() {
    m_panels.clear();
    m_framebuffer.Destroy();
    m_renderer.Shutdown();
}

// =============================================================================
// 状态
// =============================================================================
void Editor::SetTool(Tool tool) {
    if (mouseJoint != kinema::kInvalidJointId) {
        GetWorld().DestroyJoint(mouseJoint);
        mouseJoint = kinema::kInvalidJointId;
    }
    m_tool = tool;
}

void Editor::Select(kinema::BodyId id) {
    m_selection = id;
    if (id == kinema::kInvalidBodyId) followSelection = false;
}

void Editor::DeleteSelection() {
    if (m_selection == kinema::kInvalidBodyId) return;
    GetWorld().DestroyBody(m_selection);
    m_selection = kinema::kInvalidBodyId;
    followSelection = false;
}

void Editor::LoadScene(int index) {
    mouseJoint = kinema::kInvalidJointId;
    m_selection = kinema::kInvalidBodyId;
    followSelection = false;
    m_effects.clear();
    m_sim.LoadScene(index);
    ResetCamera();
}

void Editor::ResetScene() {
    mouseJoint = kinema::kInvalidJointId;
    m_selection = kinema::kInvalidBodyId;
    followSelection = false;
    m_effects.clear();
    m_sim.Reset();
    Notify(Tr("Scene reset"));
}

void Editor::ResetCamera() {
    const auto& scenes = kinema::scene::GetScenes();
    const int index = m_sim.GetSceneIndex();
    if (index >= 0) {
        m_camera.center = scenes[static_cast<size_t>(index)].cameraCenter;
        m_camera.SetViewHeight(scenes[static_cast<size_t>(index)].viewHeight);
        return;
    }
    // 从文件加载的场景：自动框选所有物体
    const auto& bodies = GetWorld().GetBodies();
    if (bodies.empty()) return;
    kinema::AABB box = bodies.front()->GetAABB();
    for (const auto& b : bodies) box = kinema::AABB::Combine(box, b->GetAABB());
    m_camera.center = box.Center();
    const float aspect = m_camera.viewport.x / std::max(m_camera.viewport.y, 1.0f);
    m_camera.SetViewHeight(1.1f * std::max(box.Height(), box.Width() / std::max(aspect, 0.1f)));
}

void Editor::FocusOn(const kinema::Body& body) {
    m_camera.center = body.GetPosition();
    const kinema::AABB& box = body.GetAABB();
    m_camera.SetViewHeight(kinema::Clamp(std::max(box.Width(), box.Height()) * 8.0f, 6.0f, 60.0f));
}

void Editor::TogglePause() {
    m_sim.paused = !m_sim.paused;
    Notify(m_sim.paused ? Tr("Paused") : Tr("Running"));
}

void Editor::StepOnce() {
    m_sim.paused = true;
    m_sim.StepOnce();
}

void Editor::Notify(const std::string& text) {
    m_toasts.push_back({text, 2.2f});
    if (m_toasts.size() > 4) m_toasts.erase(m_toasts.begin());
}

uint64_t Editor::NextSpawnColor() {
    if (!m_spawn.randomColor) {
        return kinema::scene::PackColor(static_cast<uint8_t>(m_spawn.color[0] * 255.0f),
                                        static_cast<uint8_t>(m_spawn.color[1] * 255.0f),
                                        static_cast<uint8_t>(m_spawn.color[2] * 255.0f));
    }
    return kinema::scene::palette::Pick(static_cast<int>(m_colorSeed++ % 8u));
}

std::string Editor::ConsumeScreenshotRequest() {
    std::string out;
    out.swap(m_screenshotRequest);
    return out;
}

std::string Editor::WindowTitle() const {
    std::string title = "Kinema2D - " + m_sim.GetSceneName();
    if (m_sim.paused) title += " [Paused]";
    return title;
}

// =============================================================================
// 每帧
// =============================================================================
void Editor::UpdateFrameState(float dt) {
    m_sim.Update(dt);

    // 物体可能被模拟删除（掉出世界）：清理悬空的引用
    kinema::World& world = GetWorld();
    if (m_selection && !world.FindBody(m_selection)) Select(kinema::kInvalidBodyId);
    if (mouseJoint && !world.FindJoint(mouseJoint)) mouseJoint = kinema::kInvalidJointId;
    if (hoveredBody && !world.FindBody(hoveredBody)) hoveredBody = kinema::kInvalidBodyId;

    // 相机跟随（指数平滑）
    if (followSelection) {
        if (kinema::Body* b = GetSelectedBody()) {
            const float t = 1.0f - std::exp(-8.0f * dt);
            m_camera.center = kinema::Lerp(m_camera.center, b->GetPosition(), t);
        }
    }

    for (Toast& t : m_toasts) t.timeLeft -= dt;
    m_toasts.erase(std::remove_if(m_toasts.begin(), m_toasts.end(), [](const Toast& t) { return t.timeLeft <= 0.0f; }),
                   m_toasts.end());

    for (Effect& e : m_effects) e.age += dt;
    m_effects.erase(std::remove_if(m_effects.begin(), m_effects.end(), [](const Effect& e) { return e.age >= e.life; }),
                    m_effects.end());

    // 统计历史（环形缓冲区）
    const int i = m_history.offset;
    m_history.stepMs[i] = world.GetStats().totalMs;
    m_history.frameMs[i] = dt * 1000.0f;
    m_history.energy[i] = world.ComputeKineticEnergy();
    m_history.awake[i] = static_cast<float>(world.GetStats().awakeBodyCount);
    m_history.offset = (i + 1) % History::kSize;
}

void Editor::OnFrame(float dt) {
    UpdateFrameState(dt);
    HandleShortcuts();

    DrawMenuBar();
    DrawStatusBar();
    DrawDockSpace();

    for (auto& panel : m_panels) {
        if (panel->open) panel->OnImGui(*this);
    }

    DrawPopups();
    if (m_showImGuiDemo) ImGui::ShowDemoWindow(&m_showImGuiDemo);
}

// =============================================================================
// 快捷键
// =============================================================================
void Editor::HandleShortcuts() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) return;

    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_S)) SaveSceneDialog();
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) OpenSceneDialog();
    if (io.KeyCtrl || io.KeySuper) return;

    if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) TogglePause();
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow) || ImGui::IsKeyPressed(ImGuiKey_N)) StepOnce();
    if (ImGui::IsKeyPressed(ImGuiKey_R, false)) ResetScene();
    if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) || ImGui::IsKeyPressed(ImGuiKey_Backspace, false)) DeleteSelection();
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) Select(kinema::kInvalidBodyId);
    if (ImGui::IsKeyPressed(ImGuiKey_H, false) || ImGui::IsKeyPressed(ImGuiKey_Home, false)) ResetCamera();
    if (ImGui::IsKeyPressed(ImGuiKey_F, false) && m_selection) {
        followSelection = !followSelection;
        Notify(followSelection ? Tr("Camera follows selection") : Tr("Camera follow off"));
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F12, false)) {
        const auto now = std::time(nullptr);
        char name[64];
        std::strftime(name, sizeof(name), "screenshot_%Y%m%d_%H%M%S.png", std::localtime(&now));
        RequestScreenshot(name);
    }

    for (int i = 0; i < static_cast<int>(Tool::Count); ++i) {
        if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_1 + i), false)) SetTool(static_cast<Tool>(i));
    }
}

// =============================================================================
// 菜单栏
// =============================================================================
void Editor::DrawMenuBar() {
    if (!ImGui::BeginMainMenuBar()) return;

    // 品牌标识
    ImGui::PushStyleColor(ImGuiCol_Text, theme::Accent());
    ImGui::TextUnformatted("Kinema2D");
    ImGui::PopStyleColor();
    ImGui::Spacing();

    if (ImGui::BeginMenu(TrId("File", "menu_file").c_str())) {
        if (ImGui::MenuItem(Tr("New Sandbox"))) LoadScene(kinema::scene::FindScene("sandbox"));
        if (ImGui::MenuItem(Tr("Open Scene..."), "Ctrl+O")) OpenSceneDialog();
        if (ImGui::MenuItem(Tr("Save Scene As..."), "Ctrl+S")) SaveSceneDialog();
        ImGui::Separator();
        if (ImGui::MenuItem(Tr("Save Screenshot"), "F12")) {
            const auto now = std::time(nullptr);
            char name[64];
            std::strftime(name, sizeof(name), "screenshot_%Y%m%d_%H%M%S.png", std::localtime(&now));
            RequestScreenshot(name);
        }
        ImGui::Separator();
        if (ImGui::MenuItem(Tr("Exit"))) m_quit = true;
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu(TrId("Edit", "menu_edit").c_str())) {
        if (ImGui::MenuItem(Tr("Delete Selected"), "Del", false, m_selection != 0)) DeleteSelection();
        if (ImGui::MenuItem(Tr("Deselect"), "Esc", false, m_selection != 0)) Select(kinema::kInvalidBodyId);
        ImGui::Separator();
        if (ImGui::MenuItem(Tr("Wake All Bodies"))) GetWorld().WakeAll();
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu(TrId("Simulation", "menu_sim").c_str())) {
        if (ImGui::MenuItem(m_sim.paused ? Tr("Play") : Tr("Pause"), "Space")) TogglePause();
        if (ImGui::MenuItem(Tr("Step"), "N")) StepOnce();
        if (ImGui::MenuItem(Tr("Reset Scene"), "R")) ResetScene();
        ImGui::Separator();
        if (ImGui::BeginMenu(Tr("Scenes"))) {
            const auto& scenes = kinema::scene::GetScenes();
            for (size_t i = 0; i < scenes.size(); ++i) {
                const bool current = static_cast<int>(i) == m_sim.GetSceneIndex();
                if (ImGui::MenuItem(Tr(scenes[i].name.c_str()), nullptr, current)) LoadScene(static_cast<int>(i));
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu(TrId("View", "menu_view").c_str())) {
        for (auto& panel : m_panels) ImGui::MenuItem(Tr(panel->Title()), nullptr, &panel->open);
        ImGui::Separator();
        if (ImGui::MenuItem(Tr("Reset Layout"))) m_resetLayout = true;
        if (ImGui::MenuItem(Tr("Reset Camera"), "H")) ResetCamera();
        if (ImGui::MenuItem(Tr("Follow Selection"), "F", &followSelection, m_selection != 0)) {
        }
        ImGui::Separator();
        if (ImGui::BeginMenu(Tr("Theme"))) {
            if (ImGui::MenuItem(Tr("Dark"), nullptr, theme::Current() == ThemeMode::Dark))
                theme::Apply(ThemeMode::Dark, m_dpiScale);
            if (ImGui::MenuItem(Tr("Light"), nullptr, theme::Current() == ThemeMode::Light))
                theme::Apply(ThemeMode::Light, m_dpiScale);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Language / 语言")) {
            if (ImGui::MenuItem("English", nullptr, GetLanguage() == Language::English)) SetLanguage(Language::English);
            if (ImGui::MenuItem("简体中文", nullptr, GetLanguage() == Language::Chinese, theme::HasCjkFont()))
                SetLanguage(Language::Chinese);
            if (!theme::HasCjkFont()) ImGui::SetItemTooltip("No Chinese font found on this system");
            ImGui::EndMenu();
        }
        ImGui::Separator();
        ImGui::MenuItem(Tr("ImGui Demo"), nullptr, &m_showImGuiDemo);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu(TrId("Help", "menu_help").c_str())) {
        if (ImGui::MenuItem(Tr("Controls"))) m_showControls = true;
        if (ImGui::MenuItem(Tr("About Kinema2D"))) m_showAbout = true;
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

// =============================================================================
// 状态栏
// =============================================================================
void Editor::DrawStatusBar() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float height = ImGui::GetFrameHeight();
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoNav;

    if (ImGui::BeginViewportSideBar("##StatusBar", const_cast<ImGuiViewport*>(vp), ImGuiDir_Down, height, flags)) {
        if (ImGui::BeginMenuBar()) {
            const auto& stats = GetWorld().GetStats();
            const ImVec4 dot = m_sim.paused ? ImVec4(1.0f, 0.69f, 0.13f, 1.0f) : ImVec4(0.02f, 0.84f, 0.63f, 1.0f);
            ImGui::TextColored(dot, "●");
            ImGui::TextUnformatted(m_sim.paused ? Tr("Paused") : Tr("Running"));
            ImGui::Separator();
            ImGui::Text("%s: %s", Tr("Scene"), Tr(m_sim.GetSceneName().c_str()));
            ImGui::Separator();
            ImGui::Text("%s: %d (%d %s)", Tr("Bodies"), stats.bodyCount, stats.awakeBodyCount, Tr("awake"));
            ImGui::Separator();
            ImGui::Text("%s: %d", Tr("Contacts"), stats.contactCount);
            ImGui::Separator();
            ImGui::Text("%s: %.2f ms", Tr("Step"), m_sim.GetLastStepMs());
            ImGui::Separator();
            ImGui::Text("t = %.2f s", m_sim.GetSimTime());

            const char* toolText = Tr(ToolName(m_tool));
            char right[128];
            std::snprintf(right, sizeof(right), "%s: %s   |   %.0f FPS", Tr("Tool"), toolText, ImGui::GetIO().Framerate);
            const float w = ImGui::CalcTextSize(right).x;
            ImGui::SameLine(ImGui::GetWindowWidth() - w - ImGui::GetStyle().WindowPadding.x * 2.0f);
            ImGui::TextDisabled("%s", right);
            ImGui::EndMenuBar();
        }
    }
    ImGui::End();
}

// =============================================================================
// 停靠空间与默认布局
// =============================================================================
void Editor::DrawDockSpace() {
    const ImGuiID dockspaceId = ImHashStr("KinemaDockSpace");
    if (m_resetLayout || ImGui::DockBuilderGetNode(dockspaceId) == nullptr) {
        BuildDefaultLayout(dockspaceId);
        m_resetLayout = false;
    }
    ImGui::DockSpaceOverViewport(dockspaceId, ImGui::GetMainViewport());
}

void Editor::BuildDefaultLayout(unsigned int dockspaceId) {
    //  ┌──────────┬──────────────────────────┬────────────┐
    //  │ Scenes   │                          │ Inspector  │
    //  ├──────────┤        Viewport          ├────────────┤
    //  │ Outliner │                          │ Settings   │
    //  │          ├──────────────────────────┤            │
    //  │          │ Performance | Console    │            │
    //  └──────────┴──────────────────────────┴────────────┘
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::DockBuilderRemoveNode(dockspaceId);
    ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspaceId, vp->WorkSize);

    ImGuiID center = dockspaceId;
    ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.19f, nullptr, &center);
    ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f, nullptr, &center);
    ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.27f, nullptr, &center);
    ImGuiID leftBottom = ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.55f, nullptr, &left);
    ImGuiID rightBottom = ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.5f, nullptr, &right);

    ImGui::DockBuilderDockWindow("###Viewport", center);
    ImGui::DockBuilderDockWindow("###Scenes", left);
    ImGui::DockBuilderDockWindow("###Outliner", leftBottom);
    ImGui::DockBuilderDockWindow("###Inspector", right);
    ImGui::DockBuilderDockWindow("###Settings", rightBottom);
    ImGui::DockBuilderDockWindow("###Console", bottom);
    ImGui::DockBuilderDockWindow("###Stats", bottom);  // 最后停靠的标签页默认处于选中状态
    ImGui::DockBuilderFinish(dockspaceId);
}

// =============================================================================
// 弹窗
// =============================================================================
namespace {

/// 列出当前目录下的 .kscene 文件，点击即填入路径
void SceneFileList(char* buffer, size_t size) {
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(fs::current_path(ec), ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".kscene") files.push_back(entry.path().filename().string());
    }
    if (files.empty()) {
        ImGui::TextDisabled("%s", Tr("(no .kscene files in the working directory)"));
        return;
    }
    if (ImGui::BeginChild("##files", ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 6), ImGuiChildFlags_Borders)) {
        for (const auto& f : files) {
            if (ImGui::Selectable(f.c_str(), f == buffer)) std::snprintf(buffer, size, "%s", f.c_str());
        }
    }
    ImGui::EndChild();
}

void CenterNextWindow() {
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
}

}  // namespace

void Editor::DrawPopups() {
    const std::string openId = TrId("Open Scene", "popup_open");
    const std::string saveId = TrId("Save Scene", "popup_save");
    const std::string aboutId = TrId("About Kinema2D", "popup_about");
    const std::string controlsId = TrId("Controls", "popup_controls");

    if (m_openPopup) { ImGui::OpenPopup(openId.c_str()); m_openPopup = false; }
    if (m_savePopup) { ImGui::OpenPopup(saveId.c_str()); m_savePopup = false; }
    if (m_showAbout) { ImGui::OpenPopup(aboutId.c_str()); m_showAbout = false; }
    if (m_showControls) { ImGui::OpenPopup(controlsId.c_str()); m_showControls = false; }

    // ---- 打开 ----
    CenterNextWindow();
    ImGui::SetNextWindowSize(ImVec2(460 * m_dpiScale, 0));
    if (ImGui::BeginPopupModal(openId.c_str(), nullptr, ImGuiWindowFlags_NoResize)) {
        ImGui::TextUnformatted(Tr("File path:"));
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        const bool enter = ImGui::InputText("##path", m_pathBuffer, sizeof(m_pathBuffer), ImGuiInputTextFlags_EnterReturnsTrue);
        SceneFileList(m_pathBuffer, sizeof(m_pathBuffer));
        ImGui::Spacing();
        if (ImGui::Button(Tr("Open"), ImVec2(120 * m_dpiScale, 0)) || enter) {
            std::string error;
            if (m_sim.LoadFile(m_pathBuffer, &error)) {
                m_selection = kinema::kInvalidBodyId;
                mouseJoint = kinema::kInvalidJointId;
                ResetCamera();
                Notify(std::string(Tr("Opened")) + " " + m_pathBuffer);
            } else {
                Log::Error("%s", error.c_str());
                Notify(error);
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(Tr("Cancel"), ImVec2(120 * m_dpiScale, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // ---- 保存 ----
    CenterNextWindow();
    ImGui::SetNextWindowSize(ImVec2(460 * m_dpiScale, 0));
    if (ImGui::BeginPopupModal(saveId.c_str(), nullptr, ImGuiWindowFlags_NoResize)) {
        ImGui::TextUnformatted(Tr("File path:"));
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        const bool enter = ImGui::InputText("##path", m_pathBuffer, sizeof(m_pathBuffer), ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::TextDisabled("%s", Tr("Saves the current state (positions and velocities) of the world."));
        ImGui::Spacing();
        if (ImGui::Button(Tr("Save"), ImVec2(120 * m_dpiScale, 0)) || enter) {
            std::string error;
            if (m_sim.SaveFile(m_pathBuffer, &error)) {
                Notify(std::string(Tr("Saved")) + " " + m_pathBuffer);
            } else {
                Log::Error("%s", error.c_str());
                Notify(error);
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(Tr("Cancel"), ImVec2(120 * m_dpiScale, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // ---- 关于 ----
    CenterNextWindow();
    if (ImGui::BeginPopupModal(aboutId.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (theme::LargeFont()) ImGui::PushFont(theme::LargeFont());
        ImGui::TextColored(theme::Accent(), "Kinema2D %s", kinema::kVersionString);
        if (theme::LargeFont()) ImGui::PopFont();
        ImGui::TextUnformatted(Tr("A 2D rigid body physics engine built for learning."));
        ImGui::Separator();
        ImGui::BulletText("%s", Tr("Sequential impulse solver with warm starting"));
        ImGui::BulletText("%s", Tr("SAT + clipping contact manifolds"));
        ImGui::BulletText("%s", Tr("Sweep-and-prune broad phase"));
        ImGui::BulletText("%s", Tr("Island-based sleeping"));
        ImGui::BulletText("%s", Tr("Distance, revolute (motor) and mouse joints"));
        ImGui::Separator();
        ImGui::TextDisabled("C++17 · OpenGL 3.3 · GLFW · Dear ImGui %s", IMGUI_VERSION);
        ImGui::Spacing();
        if (ImGui::Button(Tr("Close"), ImVec2(-FLT_MIN, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // ---- 操作说明 ----
    CenterNextWindow();
    if (ImGui::BeginPopupModal(controlsId.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        struct Row { const char* key; const char* action; };
        const Row rows[] = {
            {"Space", "Play / pause"},
            {"N / →", "Step one frame"},
            {"R", "Reset scene"},
            {"1 - 9", "Select tool"},
            {"Del", "Delete selected body"},
            {"F", "Toggle camera follow"},
            {"H", "Reset camera"},
            {"Esc", "Deselect"},
            {"Ctrl+S / Ctrl+O", "Save / open scene"},
            {"F12", "Save screenshot"},
            {"Mouse wheel", "Zoom at cursor"},
            {"Right / middle drag", "Pan camera"},
            {"Left drag (empty space)", "Pan camera (Select tool)"},
        };
        if (ImGui::BeginTable("##controls", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn(Tr("Input"));
            ImGui::TableSetupColumn(Tr("Action"));
            ImGui::TableHeadersRow();
            for (const Row& r : rows) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(theme::Accent(), "%s", r.key);
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(Tr(r.action));
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        if (ImGui::Button(Tr("Close"), ImVec2(-FLT_MIN, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

}  // namespace app
