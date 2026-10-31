// =============================================================================
// Performance 面板：帧率、物理各阶段耗时、计数器与历史曲线
// =============================================================================
#include <algorithm>
#include <cstdio>

#include <imgui.h>

#include "editor/Editor.h"
#include "editor/Theme.h"
#include "editor/panels/Panel.h"

namespace app {

namespace {

/// 一行"阶段耗时"：名称 + 彩色比例条 + 数值
void TimingBar(const char* label, float ms, float totalMs, ImU32 color) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(label);
    ImGui::TableNextColumn();

    const float frac = totalMs > 0.0f ? std::clamp(ms / totalMs, 0.0f, 1.0f) : 0.0f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    const float h = ImGui::GetTextLineHeight();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    dl->AddRectFilled(p, ImVec2(p.x + w * frac, p.y + h), color, 3.0f);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f ms", ms);
    dl->AddText(ImVec2(p.x + 6.0f, p.y), ImGui::GetColorU32(ImGuiCol_Text), buf);
    ImGui::Dummy(ImVec2(w, h));
}

void Counter(const char* label, const char* fmt, double value) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(label);
    ImGui::TableNextColumn();
    ImGui::Text(fmt, value);
}

/// 从环形缓冲区画曲线（offset 指向最旧的数据）
void HistoryPlot(const char* id, const char* overlay, const float* data, int count, int offset, float height) {
    float maxV = 0.0f;
    for (int i = 0; i < count; ++i) maxV = std::max(maxV, data[i]);
    ImGui::PlotLines(id, data, count, offset, overlay, 0.0f, maxV * 1.15f + 1e-3f, ImVec2(-FLT_MIN, height));
}

}  // namespace

void StatsPanel::OnImGui(Editor& ed) {
    if (!ImGui::Begin(WindowName().c_str(), &open)) {
        ImGui::End();
        return;
    }

    const kinema::WorldStats& st = ed.GetWorld().GetStats();
    const Editor::History& h = ed.GetHistory();
    const ImGuiIO& io = ImGui::GetIO();
    const float plotH = 46.0f * ed.DpiScale();

    // 三列布局：计数 | 阶段耗时 | 曲线
    if (ImGui::BeginTable("##statslayout", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableNextColumn();
        if (ImGui::BeginTable("##counters", 2, ImGuiTableFlags_SizingStretchProp)) {
            Counter(Tr("FPS"), "%.0f", io.Framerate);
            Counter(Tr("Frame time"), "%.2f ms", 1000.0 / std::max(io.Framerate, 1.0f));
            Counter(Tr("Bodies"), "%.0f", st.bodyCount);
            Counter(Tr("Awake bodies"), "%.0f", st.awakeBodyCount);
            Counter(Tr("Islands"), "%.0f", st.islandCount);
            Counter(Tr("Joints"), "%.0f", st.jointCount);
            Counter(Tr("Broad-phase pairs"), "%.0f", st.pairCount);
            Counter(Tr("Contacts"), "%.0f", st.contactCount);
            Counter(Tr("Contact points"), "%.0f", st.pointCount);
            Counter(Tr("Draw calls"), "%.0f", ed.GetRenderer().DrawCalls());
            Counter(Tr("Vertices"), "%.0f", static_cast<double>(ed.GetRenderer().VertexCount()));
            ImGui::EndTable();
        }

        ImGui::TableNextColumn();
        ImGui::TextDisabled("%s", Tr("Last physics step"));
        if (ImGui::BeginTable("##timings", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("a", ImGuiTableColumnFlags_WidthStretch, 0.45f);
            ImGui::TableSetupColumn("b", ImGuiTableColumnFlags_WidthStretch, 0.55f);
            const float total = std::max(st.totalMs, 1e-4f);
            TimingBar(Tr("Broad phase"), st.broadPhaseMs, total, IM_COL32(79, 140, 255, 200));
            TimingBar(Tr("Narrow phase"), st.narrowPhaseMs, total, IM_COL32(255, 107, 107, 200));
            TimingBar(Tr("Integrate"), st.integrateMs, total, IM_COL32(255, 209, 102, 200));
            TimingBar(Tr("Solver"), st.solverMs, total, IM_COL32(6, 214, 160, 200));
            TimingBar(Tr("Sleep / islands"), st.sleepMs, total, IM_COL32(167, 139, 250, 200));
            TimingBar(Tr("Total"), st.totalMs, total, IM_COL32(140, 146, 166, 160));
            ImGui::EndTable();
        }
        ImGui::Spacing();
        ImGui::TextDisabled("%s: %llu", Tr("Steps"), static_cast<unsigned long long>(ed.Sim().GetStepCount()));

        ImGui::TableNextColumn();
        char overlay[64];
        const int last = (h.offset + Editor::History::kSize - 1) % Editor::History::kSize;
        std::snprintf(overlay, sizeof(overlay), "%s %.3f ms", Tr("Step"), h.stepMs[last]);
        HistoryPlot("##step", overlay, h.stepMs, Editor::History::kSize, h.offset, plotH);
        std::snprintf(overlay, sizeof(overlay), "%s %.0f", Tr("Awake"), h.awake[last]);
        HistoryPlot("##awake", overlay, h.awake, Editor::History::kSize, h.offset, plotH);
        std::snprintf(overlay, sizeof(overlay), "%s %.1f J", Tr("Kinetic energy"), h.energy[last]);
        HistoryPlot("##energy", overlay, h.energy, Editor::History::kSize, h.offset, plotH);

        ImGui::EndTable();
    }

    ImGui::End();
}

}  // namespace app
