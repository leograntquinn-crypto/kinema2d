// =============================================================================
// Inspector 面板：查看并实时编辑选中物体的所有属性
// =============================================================================
#include <algorithm>
#include <cstdio>
#include <vector>

#include <imgui.h>
#include <kinema_scene/Color.h>

#include "editor/DebugDraw.h"
#include "editor/Editor.h"
#include "editor/Theme.h"
#include "editor/panels/Panel.h"

namespace app {

namespace {

/// 属性表的一行：左侧标签，右侧控件占满剩余宽度
void Row(const char* label, const char* tooltip = nullptr) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    if (tooltip && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", tooltip);
    ImGui::TableNextColumn();
    ImGui::SetNextItemWidth(-FLT_MIN);
}

bool BeginProps(const char* id) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp)) return false;
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthStretch, 0.42f);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 0.58f);
    return true;
}

void ReadOnly(const char* label, const char* fmt, double value) {
    Row(label);
    ImGui::TextDisabled(fmt, value);
}

void DrawJointEditor(Editor& ed, kinema::Joint& joint, const kinema::Body& self) {
    const kinema::Body* other = joint.GetBodyA() == &self ? joint.GetBodyB() : joint.GetBodyA();
    char header[96];
    std::snprintf(header, sizeof(header), "%s #%u  ->  #%u###joint%u", Tr(kinema::ToString(joint.GetType())), joint.GetId(),
                  other ? other->GetId() : 0u, joint.GetId());

    if (!ImGui::TreeNodeEx(header, ImGuiTreeNodeFlags_SpanAvailWidth)) return;

    if (BeginProps("##jointprops")) {
        if (joint.GetType() == kinema::JointType::Revolute) {
            auto& rj = static_cast<kinema::RevoluteJoint&>(joint);
            bool motor = rj.IsMotorEnabled();
            Row(Tr("Motor"));
            if (ImGui::Checkbox("##motor", &motor)) rj.EnableMotor(motor);
            float speed = rj.GetMotorSpeed();
            Row(Tr("Motor speed"), Tr("Target relative angular velocity (rad/s)"));
            if (ImGui::DragFloat("##speed", &speed, 0.05f, -50.0f, 50.0f, "%.2f rad/s")) rj.SetMotorSpeed(speed);
            float torque = rj.GetMaxMotorTorque();
            Row(Tr("Max torque"));
            if (ImGui::DragFloat("##torque", &torque, 10.0f, 0.0f, 1e6f, "%.0f N·m")) rj.SetMaxMotorTorque(torque);
        } else if (joint.GetType() == kinema::JointType::Distance) {
            auto& dj = static_cast<kinema::DistanceJoint&>(joint);
            float length = dj.GetLength();
            Row(Tr("Length"));
            if (ImGui::DragFloat("##len", &length, 0.02f, 0.01f, 100.0f, "%.2f m")) dj.SetLength(length);
            float hz = dj.GetFrequency();
            Row(Tr("Frequency"), Tr("0 = rigid rod, > 0 = spring stiffness (Hz)"));
            if (ImGui::DragFloat("##hz", &hz, 0.05f, 0.0f, 30.0f, "%.2f Hz")) dj.SetFrequency(hz);
            float damping = dj.GetDampingRatio();
            Row(Tr("Damping ratio"));
            if (ImGui::SliderFloat("##damp", &damping, 0.0f, 2.0f, "%.2f")) dj.SetDampingRatio(damping);
        }
        Row(Tr("Reaction force"));
        const float invDt = ed.Sim().hz;
        ImGui::TextDisabled("%.1f N", joint.GetReactionForce(invDt).Length());
        ImGui::EndTable();
    }
    if (joint.GetType() != kinema::JointType::Mouse && ImGui::SmallButton(Tr("Delete joint"))) {
        ed.GetWorld().DestroyJoint(joint.GetId());
    }
    ImGui::TreePop();
}

}  // namespace

void InspectorPanel::OnImGui(Editor& ed) {
    if (!ImGui::Begin(WindowName().c_str(), &open)) {
        ImGui::End();
        return;
    }

    kinema::Body* body = ed.GetSelectedBody();
    if (!body) {
        ImGui::Spacing();
        ImGui::TextDisabled("%s", Tr("No body selected."));
        ImGui::TextWrapped("%s", Tr("Click a body in the viewport (Select tool) or in the Outliner to inspect it."));
        ImGui::End();
        return;
    }

    kinema::Body& b = *body;
    const float dpi = ed.DpiScale();

    // ---- 标题：颜色 + 编号 + 类型 ----
    {
        Color c = BodyColor(b);
        float col[3] = {c.Rf(), c.Gf(), c.Bf()};
        if (ImGui::ColorEdit3("##color", col, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel)) {
            b.SetUserData(kinema::scene::PackColor(static_cast<uint8_t>(col[0] * 255.0f), static_cast<uint8_t>(col[1] * 255.0f),
                                                   static_cast<uint8_t>(col[2] * 255.0f)));
        }
        ImGui::SameLine();
        if (theme::LargeFont()) ImGui::PushFont(theme::LargeFont());
        ImGui::Text("%s #%u", Tr(kinema::ToString(b.GetShapeType())), b.GetId());
        if (theme::LargeFont()) ImGui::PopFont();
        ImGui::SameLine();
        ImGui::TextDisabled("%s", b.IsStatic() ? "" : b.IsAwake() ? Tr("awake") : Tr("sleeping"));
    }
    ImGui::Spacing();

    // ---- 快捷操作 ----
    const float bw = (ImGui::GetContentRegionAvail().x - 2.0f * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
    if (ImGui::Button(Tr("Focus"), ImVec2(bw, 0))) ed.FocusOn(b);
    ImGui::SameLine();
    if (ImGui::Button(ed.followSelection ? Tr("Unfollow") : Tr("Follow"), ImVec2(bw, 0))) ed.followSelection = !ed.followSelection;
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.25f, 0.3f, 0.6f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.3f, 0.35f, 0.8f));
    if (ImGui::Button(Tr("Delete"), ImVec2(bw, 0))) {
        ImGui::PopStyleColor(2);
        ed.DeleteSelection();
        ImGui::End();
        return;
    }
    ImGui::PopStyleColor(2);
    ImGui::Spacing();

    // ---- 变换 ----
    if (ImGui::CollapsingHeader(Tr("Transform"), ImGuiTreeNodeFlags_DefaultOpen) && BeginProps("##xf")) {
        kinema::Vec2 p = b.GetPosition();
        float angleDeg = kinema::RadToDeg(b.GetAngle());
        Row(Tr("Position"));
        if (ImGui::DragFloat2("##pos", &p.x, 0.02f, 0.0f, 0.0f, "%.3f")) b.SetTransform(p, b.GetAngle());
        Row(Tr("Angle"));
        if (ImGui::DragFloat("##angle", &angleDeg, 0.5f, 0.0f, 0.0f, "%.1f°")) b.SetTransform(p, kinema::DegToRad(angleDeg));
        ImGui::EndTable();
    }

    // ---- 速度 ----
    if (ImGui::CollapsingHeader(Tr("Velocity"), ImGuiTreeNodeFlags_DefaultOpen) && BeginProps("##vel")) {
        kinema::Vec2 v = b.GetLinearVelocity();
        float w = b.GetAngularVelocity();
        ImGui::BeginDisabled(b.IsStatic());
        Row(Tr("Linear"), "m/s");
        if (ImGui::DragFloat2("##v", &v.x, 0.05f, 0.0f, 0.0f, "%.2f")) b.SetLinearVelocity(v);
        Row(Tr("Angular"), "rad/s");
        if (ImGui::DragFloat("##w", &w, 0.05f, 0.0f, 0.0f, "%.2f")) b.SetAngularVelocity(w);
        ImGui::EndDisabled();
        ImGui::EndTable();
    }

    // ---- 形状 ----
    if (ImGui::CollapsingHeader(Tr("Shape"), ImGuiTreeNodeFlags_DefaultOpen) && BeginProps("##shape")) {
        if (b.GetShapeType() == kinema::ShapeType::Circle) {
            float r = static_cast<const kinema::CircleShape&>(b.GetShape()).GetRadius();
            Row(Tr("Radius"));
            if (ImGui::DragFloat("##radius", &r, 0.01f, 0.05f, 20.0f, "%.2f m")) {
                b.SetShape(kinema::CircleShape(std::max(r, 0.05f)));
            }
        } else {
            const auto& poly = static_cast<const kinema::PolygonShape&>(b.GetShape());
            Row(Tr("Vertices"));
            ImGui::Text("%d", poly.GetVertexCount());
            Row(Tr("Size"));
            const kinema::AABB local = poly.ComputeAABB(kinema::Transform());
            ImGui::TextDisabled("%.2f x %.2f m", local.Width(), local.Height());
            static float scale = 1.0f;
            Row(Tr("Scale"), Tr("Uniformly scale the polygon"));
            scale = 1.0f;
            if (ImGui::DragFloat("##scale", &scale, 0.005f, 0.5f, 2.0f, "drag ↔") && scale != 1.0f) {
                std::vector<kinema::Vec2> pts;
                for (int i = 0; i < poly.GetVertexCount(); ++i) pts.push_back(scale * poly.GetVertex(i));
                kinema::PolygonShape scaled;
                if (scaled.Set(pts) && local.Width() * scale > 0.05f) b.SetShape(scaled);
            }
        }
        ImGui::EndTable();
    }

    // ---- 材质 ----
    if (ImGui::CollapsingHeader(Tr("Material"), ImGuiTreeNodeFlags_DefaultOpen) && BeginProps("##mat")) {
        kinema::Material m = b.GetMaterial();
        Row(Tr("Density"), "kg/m²");
        if (ImGui::DragFloat("##density", &m.density, 0.02f, 0.01f, 100.0f, "%.2f")) b.SetDensity(std::max(m.density, 0.01f));
        Row(Tr("Friction"), Tr("Coulomb friction coefficient"));
        if (ImGui::SliderFloat("##friction", &m.friction, 0.0f, 1.5f, "%.2f")) b.SetFriction(m.friction);
        Row(Tr("Restitution"), Tr("Bounciness: 0 = no bounce, 1 = perfectly elastic"));
        if (ImGui::SliderFloat("##restitution", &m.restitution, 0.0f, 1.0f, "%.2f")) b.SetRestitution(m.restitution);
        ImGui::EndTable();
    }

    // ---- 动力学 ----
    if (ImGui::CollapsingHeader(Tr("Dynamics"), ImGuiTreeNodeFlags_DefaultOpen) && BeginProps("##dyn")) {
        int type = static_cast<int>(b.GetType());
        const char* types[] = {Tr("Static"), Tr("Kinematic"), Tr("Dynamic")};
        Row(Tr("Body type"));
        if (ImGui::Combo("##type", &type, types, 3)) b.SetType(static_cast<kinema::BodyType>(type));

        float ld = b.GetLinearDamping(), ad = b.GetAngularDamping(), gs = b.GetGravityScale();
        Row(Tr("Linear damping"));
        if (ImGui::DragFloat("##ld", &ld, 0.01f, 0.0f, 10.0f, "%.2f")) b.SetLinearDamping(ld);
        Row(Tr("Angular damping"));
        if (ImGui::DragFloat("##ad", &ad, 0.01f, 0.0f, 10.0f, "%.2f")) b.SetAngularDamping(ad);
        Row(Tr("Gravity scale"));
        if (ImGui::DragFloat("##gs", &gs, 0.01f, -5.0f, 5.0f, "%.2f")) b.SetGravityScale(gs);

        bool fixedRot = b.IsFixedRotation();
        Row(Tr("Fixed rotation"));
        if (ImGui::Checkbox("##fr", &fixedRot)) b.SetFixedRotation(fixedRot);
        bool canSleep = b.IsSleepingAllowed();
        Row(Tr("Allow sleep"));
        if (ImGui::Checkbox("##cs", &canSleep)) b.SetSleepingAllowed(canSleep);
        bool awake = b.IsAwake();
        Row(Tr("Awake"));
        ImGui::BeginDisabled(b.IsStatic());
        if (ImGui::Checkbox("##awake", &awake)) b.SetAwake(awake);
        ImGui::EndDisabled();
        ImGui::EndTable();
    }

    // ---- 碰撞过滤 ----
    if (ImGui::CollapsingHeader(Tr("Collision Filter")) && BeginProps("##filter")) {
        uint16_t cat = b.GetCategoryBits(), mask = b.GetMaskBits();
        Row(Tr("Category bits"));
        bool changed = ImGui::InputScalar("##cat", ImGuiDataType_U16, &cat, nullptr, nullptr, "%04X",
                                          ImGuiInputTextFlags_CharsHexadecimal);
        Row(Tr("Mask bits"));
        changed |= ImGui::InputScalar("##mask", ImGuiDataType_U16, &mask, nullptr, nullptr, "%04X",
                                      ImGuiInputTextFlags_CharsHexadecimal);
        if (changed) b.SetFilter(cat, mask);
        ImGui::EndTable();
    }

    // ---- 质量属性（只读） ----
    if (ImGui::CollapsingHeader(Tr("Mass Properties"), ImGuiTreeNodeFlags_DefaultOpen) && BeginProps("##mass")) {
        if (b.IsDynamic()) {
            ReadOnly(Tr("Mass"), "%.3f kg", b.GetMass());
            ReadOnly(Tr("Inertia"), "%.3f kg·m²", b.GetInertia());
            ReadOnly(Tr("Kinetic energy"), "%.2f J", b.GetKineticEnergy());
            ReadOnly(Tr("Momentum"), "%.2f kg·m/s", b.GetMass() * b.GetLinearVelocity().Length());
            ReadOnly(Tr("Sleep timer"), "%.2f s", b.GetSleepTime());
        } else {
            Row(Tr("Mass"));
            ImGui::TextDisabled("∞");
        }
        ImGui::EndTable();
    }

    // ---- 关节 ----
    std::vector<kinema::Joint*> joints;
    for (const auto& j : ed.GetWorld().GetJoints()) {
        if (j->GetBodyA() == &b || j->GetBodyB() == &b) joints.push_back(j.get());
    }
    char jointsHeader[64];
    std::snprintf(jointsHeader, sizeof(jointsHeader), "%s (%zu)###joints", Tr("Joints"), joints.size());
    if (ImGui::CollapsingHeader(jointsHeader, ImGuiTreeNodeFlags_DefaultOpen)) {
        if (joints.empty()) ImGui::TextDisabled("%s", Tr("Use the Joint tool to connect bodies."));
        for (kinema::Joint* j : joints) {
            DrawJointEditor(ed, *j, b);
            if (!ed.GetWorld().FindBody(b.GetId())) break;
        }
    }

    // ---- 施加冲量 ----
    if (b.IsDynamic()) {
        ImGui::Spacing();
        if (ImGui::Button(Tr("Kick upward"), ImVec2(-FLT_MIN, 0))) {
            b.ApplyLinearImpulseToCenter({0.0f, 8.0f * b.GetMass()});
        }
        if (ImGui::Button(Tr("Spin"), ImVec2(-FLT_MIN, 0))) b.ApplyAngularImpulse(6.0f * b.GetInertia());
    }
    (void)dpi;

    ImGui::End();
}

}  // namespace app
