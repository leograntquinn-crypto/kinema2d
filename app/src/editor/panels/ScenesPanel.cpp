// =============================================================================
// Scenes 面板：内置示例场景列表
// =============================================================================
#include <imgui.h>
#include <imgui_internal.h>  // RenderTextEllipsis
#include <kinema_scene/SceneLibrary.h>

#include "editor/Editor.h"
#include "editor/Theme.h"
#include "editor/panels/Panel.h"

namespace app {

void ScenesPanel::OnImGui(Editor& ed) {
    if (!ImGui::Begin(WindowName().c_str(), &open)) {
        ImGui::End();
        return;
    }

    const auto& scenes = kinema::scene::GetScenes();
    const int current = ed.Sim().GetSceneIndex();
    const float dpi = ed.DpiScale();

    // 每个场景一张"卡片"：左侧色条 + 名称 + 描述
    for (size_t i = 0; i < scenes.size(); ++i) {
        const auto& sc = scenes[i];
        const bool selected = static_cast<int>(i) == current;
        ImGui::PushID(static_cast<int>(i));

        const ImVec2 start = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float lineH = ImGui::GetTextLineHeight();
        const float height = lineH * 2.0f + 14.0f * dpi;

        if (ImGui::Selectable("##scene", selected, ImGuiSelectableFlags_None, ImVec2(width, height))) {
            ed.LoadScene(static_cast<int>(i));
        }
        const bool hovered = ImGui::IsItemHovered();
        if (hovered) ImGui::SetItemTooltip("%s", Tr(sc.description.c_str()));

        ImDrawList* dl = ImGui::GetWindowDrawList();
        if (selected) {
            dl->AddRectFilled(start, ImVec2(start.x + 3.0f * dpi, start.y + height), theme::AccentU32(), 2.0f);
        }
        const float textX = start.x + 12.0f * dpi;
        dl->AddText(ImVec2(textX, start.y + 5.0f * dpi), ImGui::GetColorU32(ImGuiCol_Text), Tr(sc.name.c_str()));

        // 描述文字过长时截断并加省略号
        const char* desc = Tr(sc.description.c_str());
        const ImVec2 descPos(textX, start.y + 7.0f * dpi + lineH);
        const float maxX = start.x + width - 6.0f * dpi;
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::RenderTextEllipsis(dl, descPos, ImVec2(maxX, descPos.y + lineH), maxX, maxX, desc, nullptr, nullptr);
        ImGui::PopStyleColor();

        ImGui::PopID();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    if (ImGui::Button(Tr("Open Scene..."), ImVec2(-FLT_MIN, 0))) ed.OpenSceneDialog();
    if (ImGui::Button(Tr("Save Scene As..."), ImVec2(-FLT_MIN, 0))) ed.SaveSceneDialog();

    ImGui::End();
}

}  // namespace app
