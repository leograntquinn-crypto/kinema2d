// =============================================================================
// Outliner 面板：列出世界中的所有物体与关节
// 使用 ImGuiListClipper：只为可见的行生成控件，上千个物体也不卡。
// =============================================================================
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <vector>

#include <imgui.h>

#include "editor/DebugDraw.h"
#include "editor/Editor.h"
#include "editor/Theme.h"
#include "editor/panels/Panel.h"

namespace app {

void OutlinerPanel::OnImGui(Editor& ed) {
    if (!ImGui::Begin(WindowName().c_str(), &open)) {
        ImGui::End();
        return;
    }

    kinema::World& world = ed.GetWorld();
    const float dpi = ed.DpiScale();

    // ---- 过滤 ----
    ImGui::SetNextItemWidth(-90.0f * dpi);
    ImGui::InputTextWithHint("##filter", Tr("Search (#id or shape)"), m_filter, sizeof(m_filter));
    ImGui::SameLine();
    const char* types[] = {Tr("All"), Tr("Dynamic"), Tr("Static"), Tr("Kinematic"), Tr("Awake")};
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::Combo("##type", &m_typeFilter, types, IM_ARRAYSIZE(types));

    // 收集通过过滤的物体
    std::vector<const kinema::Body*> rows;
    rows.reserve(world.GetBodies().size());
    for (const auto& b : world.GetBodies()) {
        switch (m_typeFilter) {
            case 1: if (!b->IsDynamic()) continue; break;
            case 2: if (!b->IsStatic()) continue; break;
            case 3: if (b->GetType() != kinema::BodyType::Kinematic) continue; break;
            case 4: if (!b->IsAwake()) continue; break;
            default: break;
        }
        if (m_filter[0]) {
            char label[64];
            std::snprintf(label, sizeof(label), "#%u %s", b->GetId(), kinema::ToString(b->GetShapeType()));
            // 不区分大小写的子串匹配
            std::string hay(label), needle(m_filter);
            for (auto& c : hay) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            for (auto& c : needle) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (hay.find(needle) == std::string::npos) continue;
        }
        rows.push_back(b.get());
    }

    ImGui::TextDisabled("%zu / %zu %s, %zu %s", rows.size(), world.GetBodies().size(), Tr("bodies"),
                        world.GetJoints().size(), Tr("joints"));

    // 选择变化时（例如在视口中点选）自动滚动到该行
    if (ed.GetSelection() != m_lastSelection) {
        m_scrollToSelection = true;
        m_lastSelection = ed.GetSelection();
    }

    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerV;
    if (ImGui::BeginTable("##bodies", 3, flags, ImVec2(0, 0))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        // 类型、状态列按最长文字定宽，剩余宽度给名称列
        const float typeW = std::max(ImGui::CalcTextSize(Tr("Kinematic")).x, ImGui::CalcTextSize(Tr("Dynamic")).x);
        const float stateW = std::max(ImGui::CalcTextSize(Tr("sleeping")).x, ImGui::CalcTextSize(Tr("State")).x);
        ImGui::TableSetupColumn(Tr("Body"), ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(Tr("Type"), ImGuiTableColumnFlags_WidthFixed, typeW);
        ImGui::TableSetupColumn(Tr("State"), ImGuiTableColumnFlags_WidthFixed, stateW);
        ImGui::TableHeadersRow();

        // 找到选中行的索引，用于滚动
        int selectedRow = -1;
        for (size_t i = 0; i < rows.size(); ++i) {
            if (rows[i]->GetId() == ed.GetSelection()) selectedRow = static_cast<int>(i);
        }
        if (m_scrollToSelection && selectedRow >= 0) {
            ImGui::SetScrollY(static_cast<float>(selectedRow) * ImGui::GetTextLineHeightWithSpacing() -
                              ImGui::GetWindowHeight() * 0.4f);
        }
        m_scrollToSelection = false;

        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(rows.size()));
        while (clipper.Step()) {
            for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row) {
                const kinema::Body* b = rows[static_cast<size_t>(row)];
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::PushID(static_cast<int>(b->GetId()));

                // 颜色小方块
                const Color c = BodyColor(*b);
                const float sz = ImGui::GetTextLineHeight() * 0.7f;
                const ImVec2 p = ImGui::GetCursorScreenPos();
                const float off = (ImGui::GetTextLineHeight() - sz) * 0.5f;
                ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(p.x, p.y + off), ImVec2(p.x + sz, p.y + off + sz),
                                                          IM_COL32(c.r, c.g, c.b, 255), 2.0f);
                ImGui::Dummy(ImVec2(sz, 0));
                ImGui::SameLine();

                char label[64];
                std::snprintf(label, sizeof(label), "#%u  %s", b->GetId(), Tr(kinema::ToString(b->GetShapeType())));
                const bool selected = b->GetId() == ed.GetSelection();
                if (ImGui::Selectable(label, selected,
                                      ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick)) {
                    ed.Select(b->GetId());
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) ed.FocusOn(*b);
                }
                if (ImGui::IsItemHovered()) ed.hoveredBody = b->GetId();

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(Tr(kinema::ToString(b->GetType())));
                ImGui::TableNextColumn();
                if (b->IsStatic()) ImGui::TextDisabled("-");
                else if (b->IsAwake()) ImGui::TextColored(ImVec4(0.02f, 0.84f, 0.63f, 1.0f), "%s", Tr("awake"));
                else ImGui::TextDisabled("%s", Tr("sleeping"));

                ImGui::PopID();
            }
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

}  // namespace app
