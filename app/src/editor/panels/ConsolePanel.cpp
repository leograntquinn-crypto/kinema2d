// =============================================================================
// Console 面板：显示日志
// =============================================================================
#include <imgui.h>

#include "editor/Editor.h"
#include "editor/panels/Panel.h"
#include "util/Log.h"

namespace app {

void ConsolePanel::OnImGui(Editor& /*ed*/) {
    if (!ImGui::Begin(WindowName().c_str(), &open)) {
        ImGui::End();
        return;
    }

    if (ImGui::SmallButton(Tr("Clear"))) Log::Clear();
    ImGui::SameLine();
    ImGui::Checkbox(Tr("Auto-scroll"), &m_autoScroll);
    ImGui::Separator();

    if (ImGui::BeginChild("##log", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar)) {
        const auto& entries = Log::Entries();
        for (const LogEntry& e : entries) {
            ImGui::TextDisabled("[%7.2f]", e.time);
            ImGui::SameLine();
            ImVec4 color = ImGui::GetStyleColorVec4(ImGuiCol_Text);
            if (e.level == LogLevel::Warning) color = ImVec4(1.0f, 0.82f, 0.4f, 1.0f);
            if (e.level == LogLevel::Error) color = ImVec4(1.0f, 0.42f, 0.42f, 1.0f);
            ImGui::TextColored(color, "%s", e.text.c_str());
        }
        if (m_autoScroll && entries.size() != m_lastCount) ImGui::SetScrollHereY(1.0f);
        m_lastCount = entries.size();
    }
    ImGui::EndChild();
    ImGui::End();
}

}  // namespace app
