// =============================================================================
// Settings 面板：世界参数、求解器参数、调试绘制、工具参数
// 所有修改立即生效 —— 这是理解每个参数作用的最好方式。
// =============================================================================
#include <imgui.h>

#include "editor/Editor.h"
#include "editor/Theme.h"
#include "editor/panels/Panel.h"

namespace app {

namespace {

void HelpMarker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

}  // namespace

void SettingsPanel::OnImGui(Editor& ed) {
    if (!ImGui::Begin(WindowName().c_str(), &open)) {
        ImGui::End();
        return;
    }

    kinema::World& world = ed.GetWorld();
    kinema::WorldSettings ws = world.GetSettings();
    bool changed = false;
    Simulation& sim = ed.Sim();
    // 控件占 50% 宽度，剩下的留给标签
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);

    // =================== 世界 ===================
    if (ImGui::CollapsingHeader(Tr("World"), ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::DragFloat2(Tr("Gravity"), &ws.gravity.x, 0.05f, -50.0f, 50.0f, "%.2f")) {
            world.SetGravity(ws.gravity);
        }
        const float bw = (ImGui::GetContentRegionAvail().x - 3.0f * ImGui::GetStyle().ItemSpacing.x) / 4.0f;
        if (ImGui::Button(Tr("Earth"), ImVec2(bw, 0))) world.SetGravity({0.0f, -9.81f});
        ImGui::SameLine();
        if (ImGui::Button(Tr("Moon"), ImVec2(bw, 0))) world.SetGravity({0.0f, -1.62f});
        ImGui::SameLine();
        if (ImGui::Button(Tr("Zero-G"), ImVec2(bw, 0))) world.SetGravity({0.0f, 0.0f});
        ImGui::SameLine();
        if (ImGui::Button(Tr("Flip"), ImVec2(bw, 0))) world.SetGravity(-1.0f * world.GetSettings().gravity);
        ws.gravity = world.GetSettings().gravity;

        changed |= ImGui::Checkbox(Tr("Remove bodies below kill plane"), &ws.enableKillPlane);
    }

    // =================== 时间 ===================
    if (ImGui::CollapsingHeader(Tr("Time Step"), ImGuiTreeNodeFlags_DefaultOpen)) {
        const int hzOptions[] = {30, 60, 120, 240};
        int hzIndex = 1;
        for (int i = 0; i < 4; ++i) {
            if (static_cast<int>(sim.hz) == hzOptions[i]) hzIndex = i;
        }
        const char* hzLabels[] = {"30 Hz", "60 Hz", "120 Hz", "240 Hz"};
        if (ImGui::Combo(Tr("Physics rate"), &hzIndex, hzLabels, 4)) sim.hz = static_cast<float>(hzOptions[hzIndex]);
        HelpMarker(Tr("Fixed time step frequency. Higher = more accurate but more expensive."));

        ImGui::SliderInt(Tr("Sub-steps"), &sim.subSteps, 1, 8);
        HelpMarker(Tr("Each fixed step is split into N smaller steps. Greatly improves stiff stacks and fast objects."));

        ImGui::SliderFloat(Tr("Time scale"), &sim.timeScale, 0.05f, 2.0f, "%.2fx", ImGuiSliderFlags_Logarithmic);
    }

    // =================== 求解器 ===================
    if (ImGui::CollapsingHeader(Tr("Solver"), ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderInt(Tr("Velocity iterations"), &ws.velocityIterations, 1, 50);
        HelpMarker(Tr("How many times the sequential impulse solver sweeps over all constraints per step."));

        changed |= ImGui::Checkbox(Tr("Warm starting"), &ws.warmStarting);
        HelpMarker(Tr("Reuse last frame's impulses as the initial guess. Turn it off and watch the pyramid sag."));

        changed |= ImGui::SliderFloat(Tr("Baumgarte"), &ws.baumgarte, 0.0f, 1.0f, "%.2f");
        HelpMarker(Tr("Fraction of penetration corrected per step. 0 = bodies sink, too high = jitter."));

        changed |= ImGui::SliderFloat(Tr("Linear slop"), &ws.linearSlop, 0.0f, 0.05f, "%.3f m");
        HelpMarker(Tr("Allowed penetration. A small slop prevents contacts from flickering."));

        changed |= ImGui::SliderFloat(Tr("Restitution threshold"), &ws.restitutionThreshold, 0.0f, 5.0f, "%.2f m/s");

        int bp = static_cast<int>(ws.broadPhase);
        const char* bpNames[] = {Tr("Brute Force O(n²)"), Tr("Sweep and Prune")};
        if (ImGui::Combo(Tr("Broad phase"), &bp, bpNames, 2)) {
            ws.broadPhase = static_cast<kinema::BroadPhaseType>(bp);
            changed = true;
        }
        HelpMarker(Tr("Compare both on the Funnel scene and watch the Performance panel."));

        changed |= ImGui::Checkbox(Tr("Allow sleeping"), &ws.allowSleep);
        HelpMarker(Tr("Islands of bodies that stay still for a while are put to sleep and skipped."));
        changed |= ImGui::SliderFloat(Tr("Time to sleep"), &ws.timeToSleep, 0.1f, 3.0f, "%.1f s");
    }

    if (changed) world.SetSettings(ws);

    // =================== 调试绘制 ===================
    if (ImGui::CollapsingHeader(Tr("Debug Draw"), ImGuiTreeNodeFlags_DefaultOpen)) {
        DebugDrawSettings& d = ed.Debug();
        if (ImGui::BeginTable("##dbg", 2)) {
            auto cb = [](const char* label, bool* v) {
                ImGui::TableNextColumn();
                ImGui::Checkbox(label, v);
            };
            cb(Tr("Shapes"), &d.shapes);
            cb(Tr("Outlines"), &d.outlines);
            cb(Tr("Grid"), &d.grid);
            cb(Tr("Joints"), &d.joints);
            cb(Tr("AABBs"), &d.aabbs);
            cb(Tr("Contact points"), &d.contacts);
            cb(Tr("Contact normals"), &d.normals);
            cb(Tr("Contact impulses"), &d.impulses);
            cb(Tr("Centers of mass"), &d.centers);
            cb(Tr("Velocities"), &d.velocities);
            cb(Tr("Sleep tint"), &d.sleepTint);
            cb(Tr("Rotation markers"), &d.rotationMarkers);
            ImGui::EndTable();
        }
        ImGui::SliderFloat(Tr("Fill opacity"), &d.fillAlpha, 0.0f, 1.0f, "%.2f");
        const int samples[] = {0, 2, 4, 8};
        int si = 2;
        for (int i = 0; i < 4; ++i) {
            if (d.msaaSamples == samples[i]) si = i;
        }
        const char* msaa[] = {Tr("Off"), "2x", "4x", "8x"};
        if (ImGui::Combo(Tr("Anti-aliasing"), &si, msaa, 4)) d.msaaSamples = samples[si];
    }

    // =================== 工具参数 ===================
    if (ImGui::CollapsingHeader(Tr("Tool Options"), ImGuiTreeNodeFlags_DefaultOpen)) {
        SpawnSettings& s = ed.Spawn();
        ImGui::SeparatorText(Tr("New bodies"));
        const char* types[] = {Tr("Static"), Tr("Kinematic"), Tr("Dynamic")};
        ImGui::Combo(Tr("Body type"), &s.bodyType, types, 3);
        ImGui::DragFloat(Tr("Circle radius"), &s.circleRadius, 0.01f, 0.05f, 10.0f, "%.2f m");
        ImGui::DragFloat2(Tr("Box half size"), &s.boxHalfWidth, 0.01f, 0.05f, 10.0f, "%.2f m");
        ImGui::SliderInt(Tr("Polygon sides"), &s.polygonSides, 0, kinema::kMaxPolygonVertices,
                         s.polygonSides < 3 ? Tr("random") : "%d");
        ImGui::DragFloat(Tr("Polygon radius"), &s.polygonRadius, 0.01f, 0.1f, 10.0f, "%.2f m");
        ImGui::DragFloat(Tr("Density"), &s.material.density, 0.02f, 0.01f, 100.0f, "%.2f");
        ImGui::SliderFloat(Tr("Friction"), &s.material.friction, 0.0f, 1.5f, "%.2f");
        ImGui::SliderFloat(Tr("Restitution"), &s.material.restitution, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat(Tr("Launch strength"), &s.throwScale, 0.5f, 15.0f, "%.1f");
        ImGui::Checkbox(Tr("Random color"), &s.randomColor);
        if (!s.randomColor) {
            ImGui::SameLine();
            ImGui::ColorEdit3("##spawncolor", s.color, ImGuiColorEditFlags_NoInputs);
        }

        ImGui::SeparatorText(Tr("Joint tool"));
        int jk = static_cast<int>(s.jointKind);
        const char* kinds[] = {Tr("Rod (rigid distance)"), Tr("Spring (soft distance)"), Tr("Hinge (revolute)")};
        if (ImGui::Combo(Tr("Joint type"), &jk, kinds, 3)) s.jointKind = static_cast<JointKind>(jk);
        if (s.jointKind == JointKind::Spring) {
            ImGui::SliderFloat(Tr("Spring frequency"), &s.springHz, 0.1f, 20.0f, "%.1f Hz");
            ImGui::SliderFloat(Tr("Spring damping"), &s.springDamping, 0.0f, 2.0f, "%.2f");
        }
        ImGui::Checkbox(Tr("Connected bodies collide"), &s.jointCollide);

        ImGui::SeparatorText(Tr("Explosion"));
        ImGui::SliderFloat(Tr("Radius"), &s.explosionRadius, 1.0f, 30.0f, "%.1f m");
        ImGui::SliderFloat(Tr("Strength"), &s.explosionStrength, 1.0f, 60.0f, "%.1f m/s");
    }

    ImGui::PopItemWidth();
    ImGui::End();
}

}  // namespace app
