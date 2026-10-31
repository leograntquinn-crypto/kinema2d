// =============================================================================
// Viewport 面板：显示物理场景、处理鼠标交互、绘制工具栏与叠加信息
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include <imgui.h>

#include "editor/DebugDraw.h"
#include "editor/Editor.h"
#include "editor/Theme.h"
#include "editor/panels/Panel.h"

namespace app {

using kinema::Vec2;

namespace {

Vec2 FromIm(ImVec2 v) { return {v.x, v.y}; }

// -----------------------------------------------------------------------------
// 图标：全部用 ImDrawList 现场绘制，无需任何图片或图标字体
// -----------------------------------------------------------------------------
enum class Icon { Play, Pause, Step, Reset, Select, Circle, Box, Polygon, Static, Joint, Explode, Ray, Delete };

void DrawIcon(ImDrawList* dl, Icon icon, ImVec2 c, float s, ImU32 col) {
    const float t = std::max(1.5f, s * 0.11f);  // 线宽
    switch (icon) {
        case Icon::Play:
            dl->AddTriangleFilled(ImVec2(c.x - 0.32f * s, c.y - 0.38f * s), ImVec2(c.x - 0.32f * s, c.y + 0.38f * s),
                                  ImVec2(c.x + 0.4f * s, c.y), col);
            break;
        case Icon::Pause:
            dl->AddRectFilled(ImVec2(c.x - 0.3f * s, c.y - 0.36f * s), ImVec2(c.x - 0.08f * s, c.y + 0.36f * s), col, 1.0f);
            dl->AddRectFilled(ImVec2(c.x + 0.08f * s, c.y - 0.36f * s), ImVec2(c.x + 0.3f * s, c.y + 0.36f * s), col, 1.0f);
            break;
        case Icon::Step:
            dl->AddTriangleFilled(ImVec2(c.x - 0.36f * s, c.y - 0.34f * s), ImVec2(c.x - 0.36f * s, c.y + 0.34f * s),
                                  ImVec2(c.x + 0.18f * s, c.y), col);
            dl->AddRectFilled(ImVec2(c.x + 0.2f * s, c.y - 0.34f * s), ImVec2(c.x + 0.36f * s, c.y + 0.34f * s), col, 1.0f);
            break;
        case Icon::Reset: {
            const float r = 0.32f * s;
            dl->PathArcTo(c, r, 0.9f, 2.0f * kinema::kPi - 0.3f, 20);
            dl->PathStroke(col, 0, t);
            const ImVec2 tip(c.x + r * std::cos(0.9f), c.y + r * std::sin(0.9f));
            dl->AddTriangleFilled(ImVec2(tip.x + 0.18f * s, tip.y - 0.02f * s), ImVec2(tip.x - 0.12f * s, tip.y - 0.12f * s),
                                  ImVec2(tip.x - 0.02f * s, tip.y + 0.2f * s), col);
            break;
        }
        case Icon::Select: {
            const ImVec2 p[] = {{c.x - 0.25f * s, c.y - 0.38f * s}, {c.x - 0.25f * s, c.y + 0.28f * s},
                                {c.x - 0.08f * s, c.y + 0.12f * s}, {c.x + 0.06f * s, c.y + 0.4f * s},
                                {c.x + 0.16f * s, c.y + 0.35f * s}, {c.x + 0.03f * s, c.y + 0.08f * s},
                                {c.x + 0.27f * s, c.y + 0.06f * s}};
            dl->AddConcavePolyFilled(p, 7, col);
            break;
        }
        case Icon::Circle:
            dl->AddCircle(c, 0.34f * s, col, 24, t);
            dl->AddLine(c, ImVec2(c.x + 0.34f * s, c.y), col, t);
            break;
        case Icon::Box:
            dl->AddRect(ImVec2(c.x - 0.32f * s, c.y - 0.32f * s), ImVec2(c.x + 0.32f * s, c.y + 0.32f * s), col, 2.0f, 0, t);
            break;
        case Icon::Polygon: {
            ImVec2 p[5];
            for (int i = 0; i < 5; ++i) {
                const float a = -0.5f * kinema::kPi + 2.0f * kinema::kPi * static_cast<float>(i) / 5.0f;
                p[i] = ImVec2(c.x + 0.36f * s * std::cos(a), c.y + 0.36f * s * std::sin(a));
            }
            dl->AddPolyline(p, 5, col, ImDrawFlags_Closed, t);
            break;
        }
        case Icon::Static: {
            const ImVec2 a(c.x - 0.38f * s, c.y + 0.05f * s), b(c.x + 0.38f * s, c.y + 0.3f * s);
            dl->AddRectFilled(a, b, col, 1.0f);
            for (int i = 0; i < 4; ++i) {
                const float x = a.x + 0.05f * s + 0.2f * s * static_cast<float>(i);
                dl->AddLine(ImVec2(x, c.y - 0.05f * s), ImVec2(x + 0.12f * s, c.y - 0.28f * s), col, t * 0.8f);
            }
            break;
        }
        case Icon::Joint:
            dl->AddLine(ImVec2(c.x - 0.25f * s, c.y + 0.25f * s), ImVec2(c.x + 0.25f * s, c.y - 0.25f * s), col, t);
            dl->AddCircleFilled(ImVec2(c.x - 0.25f * s, c.y + 0.25f * s), 0.13f * s, col);
            dl->AddCircleFilled(ImVec2(c.x + 0.25f * s, c.y - 0.25f * s), 0.13f * s, col);
            break;
        case Icon::Explode:
            for (int i = 0; i < 8; ++i) {
                const float a = 2.0f * kinema::kPi * static_cast<float>(i) / 8.0f;
                const float r0 = 0.12f * s, r1 = (i % 2 ? 0.28f : 0.4f) * s;
                dl->AddLine(ImVec2(c.x + r0 * std::cos(a), c.y + r0 * std::sin(a)),
                            ImVec2(c.x + r1 * std::cos(a), c.y + r1 * std::sin(a)), col, t);
            }
            break;
        case Icon::Ray:
            dl->AddCircleFilled(ImVec2(c.x - 0.32f * s, c.y + 0.25f * s), 0.09f * s, col);
            dl->AddLine(ImVec2(c.x - 0.32f * s, c.y + 0.25f * s), ImVec2(c.x + 0.22f * s, c.y - 0.18f * s), col, t);
            dl->AddTriangleFilled(ImVec2(c.x + 0.36f * s, c.y - 0.3f * s), ImVec2(c.x + 0.1f * s, c.y - 0.25f * s),
                                  ImVec2(c.x + 0.3f * s, c.y - 0.04f * s), col);
            break;
        case Icon::Delete:
            dl->AddLine(ImVec2(c.x - 0.28f * s, c.y - 0.28f * s), ImVec2(c.x + 0.28f * s, c.y + 0.28f * s), col, t * 1.2f);
            dl->AddLine(ImVec2(c.x + 0.28f * s, c.y - 0.28f * s), ImVec2(c.x - 0.28f * s, c.y + 0.28f * s), col, t * 1.2f);
            break;
    }
}

/// 带图标的方形按钮。active 时用强调色高亮。
bool IconButton(const char* id, Icon icon, bool active, const char* tooltip, ImU32 iconColor = 0) {
    const float size = ImGui::GetFrameHeight();
    const bool pressed = ImGui::InvisibleButton(id, ImVec2(size, size));
    const bool hovered = ImGui::IsItemHovered();
    const bool held = ImGui::IsItemActive();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 a = ImGui::GetItemRectMin();
    const ImVec2 b = ImGui::GetItemRectMax();
    const float rounding = ImGui::GetStyle().FrameRounding;

    ImU32 bg = ImGui::GetColorU32(held ? ImGuiCol_ButtonActive : hovered ? ImGuiCol_ButtonHovered : ImGuiCol_Button);
    if (active) bg = theme::AccentU32(held ? 0.9f : hovered ? 0.8f : 0.7f);
    dl->AddRectFilled(a, b, bg, rounding);

    const ImU32 col = iconColor ? iconColor : active ? IM_COL32(255, 255, 255, 255) : ImGui::GetColorU32(ImGuiCol_Text);
    DrawIcon(dl, icon, ImVec2(0.5f * (a.x + b.x), 0.5f * (a.y + b.y)), size * 0.62f, col);

    if (tooltip && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", tooltip);
    return pressed;
}

Icon ToolIcon(Tool t) {
    switch (t) {
        case Tool::Select: return Icon::Select;
        case Tool::Circle: return Icon::Circle;
        case Tool::Box: return Icon::Box;
        case Tool::Polygon: return Icon::Polygon;
        case Tool::StaticBox: return Icon::Static;
        case Tool::Joint: return Icon::Joint;
        case Tool::Explode: return Icon::Explode;
        case Tool::Ray: return Icon::Ray;
        case Tool::Delete: return Icon::Delete;
        default: return Icon::Select;
    }
}

/// 生成一个随机凸多边形（在圆上取随机角度的点，再求凸包）
kinema::PolygonShape RandomPolygon(float radius) {
    static std::mt19937 rng(1234);
    std::uniform_int_distribution<int> countDist(5, 8);
    std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * kinema::kPi);
    std::uniform_real_distribution<float> radiusDist(0.7f, 1.0f);
    for (int attempt = 0; attempt < 10; ++attempt) {
        std::vector<Vec2> pts;
        const int n = countDist(rng);
        for (int i = 0; i < n; ++i) {
            const float a = angleDist(rng);
            const float r = radius * radiusDist(rng);
            pts.push_back({r * std::cos(a), r * std::sin(a)});
        }
        kinema::PolygonShape p;
        if (p.Set(pts) && p.ComputeMass(1.0f).mass > 0.3f * radius * radius) return p;
    }
    return kinema::PolygonShape::MakeRegular(6, radius);
}

/// 根据当前生成设置构造形状
std::unique_ptr<kinema::Shape> MakeSpawnShape(const SpawnSettings& s, Tool tool) {
    switch (tool) {
        case Tool::Circle: return std::make_unique<kinema::CircleShape>(s.circleRadius);
        case Tool::Box: {
            auto p = std::make_unique<kinema::PolygonShape>();
            p->SetAsBox(s.boxHalfWidth, s.boxHalfHeight);
            return p;
        }
        case Tool::Polygon: {
            if (s.polygonSides >= 3) {
                auto p = std::make_unique<kinema::PolygonShape>();
                p->SetAsRegular(s.polygonSides, s.polygonRadius);
                return p;
            }
            return std::make_unique<kinema::PolygonShape>(RandomPolygon(s.polygonRadius));
        }
        default: return nullptr;
    }
}

bool IsSpawnTool(Tool t) { return t == Tool::Circle || t == Tool::Box || t == Tool::Polygon; }

}  // namespace

// =============================================================================
void ViewportPanel::OnImGui(Editor& ed) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    const bool visible = ImGui::Begin(WindowName().c_str(), &open,
                                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    if (!visible) {
        ImGui::End();
        return;
    }

    DrawToolbar(ed);

    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.x = std::max(avail.x, 16.0f);
    avail.y = std::max(avail.y, 16.0f);
    const ImVec2 pos = ImGui::GetCursorScreenPos();

    // 透明按钮覆盖整个画布，用来接收鼠标事件（左/右/中键）
    ImGui::InvisibleButton("##canvas", avail,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();

    HandleInput(ed, FromIm(pos), FromIm(avail), hovered);
    RenderScene(ed);

    // 把离屏渲染的纹理贴到画布上（OpenGL 纹理原点在左下，所以 v 方向翻转）
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const auto texId = static_cast<ImTextureID>(static_cast<intptr_t>(ed.GetFramebuffer().GetTexture()));
    dl->AddImage(texId, pos, ImVec2(pos.x + avail.x, pos.y + avail.y), ImVec2(0, 1), ImVec2(1, 0));

    DrawOverlay(ed, FromIm(pos), FromIm(avail));
    ImGui::End();
}

// =============================================================================
// 工具栏
// =============================================================================
void ViewportPanel::DrawToolbar(Editor& ed) {
    const float pad = 6.0f * ed.DpiScale();
    const float height = ImGui::GetFrameHeight() + 2.0f * pad;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(pad * 1.5f, pad));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f * ed.DpiScale(), 4.0f * ed.DpiScale()));
    ImGui::BeginChild("##toolbar", ImVec2(0, height), ImGuiChildFlags_AlwaysUseWindowPadding,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    Simulation& sim = ed.Sim();

    // ---- 播放控制 ----
    const ImU32 playColor = sim.paused ? IM_COL32(6, 214, 160, 255) : IM_COL32(255, 176, 32, 255);
    if (IconButton("##play", sim.paused ? Icon::Play : Icon::Pause, false,
                   sim.paused ? Tr("Play (Space)") : Tr("Pause (Space)"), playColor)) {
        ed.TogglePause();
    }
    ImGui::SameLine();
    if (IconButton("##step", Icon::Step, false, Tr("Step one frame (N)"))) ed.StepOnce();
    ImGui::SameLine();
    if (IconButton("##reset", Icon::Reset, false, Tr("Reset scene (R)"))) ed.ResetScene();

    ImGui::SameLine(0, 14.0f * ed.DpiScale());
    ImGui::TextDisabled("|");
    ImGui::SameLine(0, 14.0f * ed.DpiScale());

    // ---- 工具 ----
    for (int i = 0; i < static_cast<int>(Tool::Count); ++i) {
        const Tool tool = static_cast<Tool>(i);
        char id[16];
        std::snprintf(id, sizeof(id), "##tool%d", i);
        char tip[256];
        std::snprintf(tip, sizeof(tip), "%s  [%d]\n%s", Tr(ToolName(tool)), i + 1, Tr(ToolHint(tool)));
        if (IconButton(id, ToolIcon(tool), ed.GetTool() == tool, tip)) ed.SetTool(tool);
        ImGui::SameLine();
    }

    // ---- 右侧：时间缩放 + 跟随 ----
    const float rightWidth = 260.0f * ed.DpiScale();
    const float remaining = ImGui::GetContentRegionAvail().x;  // 光标仍在工具按钮所在行
    if (remaining > rightWidth + 10.0f) {
        ImGui::SameLine(ImGui::GetCursorPosX() + remaining - rightWidth);
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("%s", Tr("Speed"));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f * ed.DpiScale());
        ImGui::SliderFloat("##timescale", &sim.timeScale, 0.05f, 2.0f, "%.2fx", ImGuiSliderFlags_Logarithmic);
        ImGui::SameLine();
        ImGui::BeginDisabled(ed.GetSelection() == 0);
        ImGui::Checkbox(Tr("Follow"), &ed.followSelection);
        ImGui::EndDisabled();
    }

    ImGui::EndChild();
    ImGui::PopStyleVar(2);
}

// =============================================================================
// 输入
// =============================================================================
void ViewportPanel::HandleInput(Editor& ed, Vec2 canvasPos, Vec2 canvasSize, bool hovered) {
    const ImGuiIO& io = ImGui::GetIO();
    Camera2D& cam = ed.GetCamera();
    kinema::World& world = ed.GetWorld();

    cam.viewport = canvasSize;
    // 停靠布局在头一两帧内才会稳定下来：画布尺寸连续两帧相同后再应用"适配视野"
    if (canvasSize == m_lastCanvasSize) cam.ApplyPendingFit();
    m_lastCanvasSize = canvasSize;
    m_mouseScreen = FromIm(io.MousePos) - canvasPos;
    m_mouseWorld = cam.ScreenToWorld(m_mouseScreen);
    m_mouseInside = hovered;

    // ---- 缩放：滚轮，以光标为中心 ----
    if (hovered && io.MouseWheel != 0.0f) cam.ZoomAt(m_mouseScreen, std::pow(1.15f, io.MouseWheel));

    // ---- 平移：右键或中键拖拽 ----
    const bool panButton = ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle);
    if (ImGui::IsItemActive() && panButton && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)) {
        cam.center.x -= io.MouseDelta.x / cam.zoom;
        cam.center.y += io.MouseDelta.y / cam.zoom;
        ed.followSelection = false;
    }

    // ---- 悬停 ----
    ed.hoveredBody = kinema::kInvalidBodyId;
    if (hovered && !m_dragging) {
        if (const kinema::Body* b = world.QueryPoint(m_mouseWorld)) ed.hoveredBody = b->GetId();
    }

    const Tool tool = ed.GetTool();
    const SpawnSettings& spawn = ed.Spawn();

    // ======================= 按下 =======================
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_dragging = true;
        m_dragButton = ImGuiMouseButton_Left;
        m_dragStartWorld = m_mouseWorld;
        m_dragStartScreen = m_mouseScreen;
        kinema::Body* body = world.QueryPoint(m_mouseWorld);

        switch (tool) {
            case Tool::Select:
                if (body) {
                    ed.Select(body->GetId());
                    if (body->IsDynamic() && !ed.Sim().paused) {
                        kinema::MouseJointDef def;
                        def.bodyB = body;
                        def.target = m_mouseWorld;
                        def.maxForce = 1000.0f * body->GetMass();
                        def.frequencyHz = 5.0f;
                        def.dampingRatio = 0.7f;
                        ed.mouseJoint = world.CreateJoint(def)->GetId();
                    } else {
                        m_moveOffset = body->GetPosition() - m_mouseWorld;
                    }
                } else {
                    ed.Select(kinema::kInvalidBodyId);
                }
                break;
            case Tool::Joint:
                m_jointStartBody = body ? body->GetId() : 0;
                if (!body) m_dragging = false;
                break;
            case Tool::Explode:
                world.ApplyExplosion(m_mouseWorld, spawn.explosionRadius, spawn.explosionStrength);
                ed.AddEffect({m_mouseWorld, spawn.explosionRadius, 0.0f, 0.45f});
                m_dragging = false;
                break;
            case Tool::Ray:
                m_rayActive = true;
                m_rayStart = m_rayEnd = m_mouseWorld;
                break;
            case Tool::Delete:
                if (body) {
                    if (body->GetId() == ed.GetSelection()) ed.Select(kinema::kInvalidBodyId);
                    world.DestroyBody(body->GetId());
                    ed.hoveredBody = kinema::kInvalidBodyId;
                }
                m_dragging = false;
                break;
            default: break;
        }
    }

    // ======================= 拖动 =======================
    if (m_dragging && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (tool == Tool::Select) {
            if (ed.mouseJoint) {
                if (auto* j = static_cast<kinema::MouseJoint*>(world.FindJoint(ed.mouseJoint))) j->SetTarget(m_mouseWorld);
            } else if (kinema::Body* b = ed.GetSelectedBody()) {
                // 暂停（或静态物体）时直接移动物体
                b->SetTransform(m_mouseWorld + m_moveOffset, b->GetAngle());
                if (b->IsDynamic()) {
                    b->SetLinearVelocity({0.0f, 0.0f});
                    b->SetAngularVelocity(0.0f);
                }
            } else if (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f) {
                // 空白处左键拖动 = 平移相机
                cam.center.x -= io.MouseDelta.x / cam.zoom;
                cam.center.y += io.MouseDelta.y / cam.zoom;
                ed.followSelection = false;
            }
        } else if (tool == Tool::Ray) {
            m_rayEnd = m_mouseWorld;
        }
    }

    // ======================= 松开 =======================
    if (m_dragging && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        m_dragging = false;
        const float dragPixels = (m_mouseScreen - m_dragStartScreen).Length();

        if (tool == Tool::Select && ed.mouseJoint) {
            world.DestroyJoint(ed.mouseJoint);
            ed.mouseJoint = kinema::kInvalidJointId;
        } else if (IsSpawnTool(tool)) {
            kinema::BodyDef def;
            def.type = static_cast<kinema::BodyType>(spawn.bodyType);
            def.position = m_dragStartWorld;
            def.material = spawn.material;
            def.userData = ed.NextSpawnColor();
            if (dragPixels > 6.0f && def.type != kinema::BodyType::Static) {
                // 弹弓：反方向发射
                def.linearVelocity = spawn.throwScale * (m_dragStartWorld - m_mouseWorld);
            }
            UpdateGhost(spawn, tool, false);
            kinema::Body* b = world.CreateBody(def, *m_ghost);
            ed.Select(b->GetId());
            UpdateGhost(spawn, tool, true);  // 随机多边形：下一个换一个形状
        } else if (tool == Tool::StaticBox) {
            const Vec2 lo = kinema::Min(m_dragStartWorld, m_mouseWorld);
            const Vec2 hi = kinema::Max(m_dragStartWorld, m_mouseWorld);
            if (hi.x - lo.x > 0.1f && hi.y - lo.y > 0.1f) {
                kinema::BodyDef def;
                def.type = kinema::BodyType::Static;
                def.position = 0.5f * (lo + hi);
                def.material = spawn.material;
                kinema::Body* b = world.CreateBody(def, kinema::PolygonShape::MakeBox(0.5f * (hi.x - lo.x), 0.5f * (hi.y - lo.y)));
                ed.Select(b->GetId());
            }
        } else if (tool == Tool::Joint && m_jointStartBody) {
            kinema::Body* a = world.FindBody(m_jointStartBody);
            kinema::Body* b = world.QueryPoint(m_mouseWorld);
            if (a && b && a != b && (a->IsDynamic() || b->IsDynamic())) {
                if (spawn.jointKind == JointKind::Hinge) {
                    kinema::RevoluteJointDef jd;
                    jd.Initialize(a, b, m_mouseWorld);
                    jd.collideConnected = spawn.jointCollide;
                    world.CreateJoint(jd);
                } else {
                    kinema::DistanceJointDef jd;
                    jd.Initialize(a, b, m_dragStartWorld, m_mouseWorld);
                    jd.collideConnected = spawn.jointCollide;
                    if (spawn.jointKind == JointKind::Spring) {
                        jd.frequencyHz = spawn.springHz;
                        jd.dampingRatio = spawn.springDamping;
                    }
                    world.CreateJoint(jd);
                }
                ed.Notify(Tr("Joint created"));
            }
            m_jointStartBody = 0;
        }
    }
}

void ViewportPanel::UpdateGhost(const SpawnSettings& spawn, Tool tool, bool force) {
    const float key[5] = {spawn.circleRadius, spawn.boxHalfWidth, spawn.boxHalfHeight,
                          static_cast<float>(spawn.polygonSides), spawn.polygonRadius};
    if (force || !m_ghost || m_ghostTool != tool || !std::equal(key, key + 5, m_ghostKey)) {
        m_ghost = MakeSpawnShape(spawn, tool);
        m_ghostTool = tool;
        std::copy(key, key + 5, m_ghostKey);
    }
}

// =============================================================================
// 渲染
// =============================================================================
void ViewportPanel::RenderScene(Editor& ed) {
    const ImGuiIO& io = ImGui::GetIO();
    Camera2D& cam = ed.GetCamera();
    Framebuffer& fb = ed.GetFramebuffer();
    Renderer2D& r = ed.GetRenderer();
    const ViewportPalette& pal = theme::Palette();
    const DebugDrawSettings& dbg = ed.Debug();
    kinema::World& world = ed.GetWorld();

    // 高 DPI 屏幕（如 Retina）上帧缓冲像素比逻辑像素多
    const int w = static_cast<int>(cam.viewport.x * io.DisplayFramebufferScale.x);
    const int h = static_cast<int>(cam.viewport.y * io.DisplayFramebufferScale.y);
    fb.Resize(w, h, dbg.msaaSamples);
    fb.Bind();
    gl::ClearColor(pal.background.Rf(), pal.background.Gf(), pal.background.Bf(), 1.0f);
    gl::Clear(gl::COLOR_BUFFER_BIT);

    r.Begin(cam);
    if (dbg.grid) DrawGrid(r, cam, pal);
    DrawWorld(r, cam, world, dbg, pal, ed.GetSelection(), ed.hoveredBody);

    // ---- 工具预览 ----
    const Tool tool = ed.GetTool();
    const SpawnSettings& spawn = ed.Spawn();
    if (IsSpawnTool(tool) && (m_mouseInside || m_dragging)) {
        UpdateGhost(spawn, tool, false);
        const Vec2 at = m_dragging ? m_dragStartWorld : m_mouseWorld;
        DrawShape(r, *m_ghost, kinema::Transform(at, kinema::Rot(0.0f)), pal.toolPreview.WithAlpha(40), pal.toolPreview,
                  1.5f, tool == Tool::Circle);

        if (m_dragging && spawn.bodyType != static_cast<int>(kinema::BodyType::Static)) {
            const Vec2 v = spawn.throwScale * (m_dragStartWorld - m_mouseWorld);
            if (v.LengthSquared() > 0.01f) {
                r.Line(m_dragStartWorld, m_mouseWorld, pal.toolPreview.WithAlpha(90), 1.0f);
                // 抛物线轨迹预测：p(t) = p0 + v t + ½ g t²
                const Vec2 g = world.GetSettings().gravity;
                for (int i = 1; i <= 30; ++i) {
                    const float t = 0.06f * static_cast<float>(i);
                    const Vec2 p = m_dragStartWorld + t * v + (0.5f * t * t) * g;
                    r.Point(p, 4.0f, pal.selection.WithAlpha(static_cast<uint8_t>(230 - 6 * i)));
                }
            }
        }
    } else if (tool == Tool::StaticBox && m_dragging) {
        const Vec2 lo = kinema::Min(m_dragStartWorld, m_mouseWorld);
        const Vec2 hi = kinema::Max(m_dragStartWorld, m_mouseWorld);
        r.RectFilled(lo, hi, pal.staticFill.WithAlpha(140));
        r.RectOutline(lo, hi, pal.toolPreview, 1.5f);
    } else if (tool == Tool::Joint && m_dragging && m_jointStartBody) {
        const Color c = pal.joint;
        if (spawn.jointKind == JointKind::Hinge) {
            r.Line(m_dragStartWorld, m_mouseWorld, c.WithAlpha(120), 1.0f);
            r.CircleOutline(m_mouseWorld, 6.0f * r.PixelSize(), c, 2.0f);
        } else {
            r.Line(m_dragStartWorld, m_mouseWorld, c, 1.5f);
        }
        r.Point(m_dragStartWorld, 7.0f, c);
        r.Point(m_mouseWorld, 7.0f, c);
    } else if (tool == Tool::Explode && m_mouseInside) {
        r.CircleOutline(m_mouseWorld, spawn.explosionRadius, pal.contactPoint.WithAlpha(120), 1.0f);
    }

    // ---- 射线 ----
    if (m_rayActive && (m_rayEnd - m_rayStart).LengthSquared() > 1e-6f) {
        kinema::RayHit hit;
        if (world.RayCast(m_rayStart, m_rayEnd, hit)) {
            r.Line(m_rayStart, hit.point, pal.contactNormal, 2.0f);
            r.Line(hit.point, m_rayEnd, pal.contactNormal.WithAlpha(60), 1.0f);
            r.Point(hit.point, 9.0f, pal.contactPoint);
            r.Arrow(hit.point, hit.point + 1.0f * hit.normal, pal.impulse, 2.0f);
        } else {
            r.Line(m_rayStart, m_rayEnd, pal.contactNormal.WithAlpha(150), 1.5f);
        }
        r.Point(m_rayStart, 7.0f, pal.contactNormal);
    }
    if (tool != Tool::Ray) m_rayActive = false;

    // ---- 特效：爆炸冲击波 ----
    for (const Effect& e : ed.GetEffects()) {
        const float t = e.age / e.life;
        const auto alpha = static_cast<uint8_t>(255.0f * (1.0f - t));
        r.CircleOutline(e.position, e.radius * std::sqrt(t), pal.selection.WithAlpha(alpha), 3.0f);
        r.CircleFilled(e.position, e.radius * std::sqrt(t), pal.selection.WithAlpha(static_cast<uint8_t>(alpha / 6)));
    }

    r.End();
    fb.Resolve();
    Framebuffer::BindDefault();
}

// =============================================================================
// 叠加层：坐标、暂停徽章、提示消息、悬停信息
// =============================================================================
void ViewportPanel::DrawOverlay(Editor& ed, Vec2 canvasPos, Vec2 canvasSize) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float dpi = ed.DpiScale();
    const float pad = 10.0f * dpi;
    const ImU32 chipBg = IM_COL32(15, 17, 22, 190);
    const ImU32 chipText = IM_COL32(230, 233, 239, 255);

    auto chip = [&](ImVec2 p, const char* text, ImU32 bg, ImU32 fg) {
        const ImVec2 sz = ImGui::CalcTextSize(text);
        const ImVec2 inner(8.0f * dpi, 4.0f * dpi);
        dl->AddRectFilled(p, ImVec2(p.x + sz.x + 2 * inner.x, p.y + sz.y + 2 * inner.y), bg, 6.0f * dpi);
        dl->AddText(ImVec2(p.x + inner.x, p.y + inner.y), fg, text);
        return ImVec2(sz.x + 2 * inner.x, sz.y + 2 * inner.y);
    };

    // 左上：当前工具与提示
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s  -  %s", Tr(ToolName(ed.GetTool())), Tr(ToolHint(ed.GetTool())));
        chip(ImVec2(canvasPos.x + pad, canvasPos.y + pad), buf, chipBg, chipText);
    }

    // 右上：暂停徽章
    if (ed.Sim().paused) {
        const char* text = Tr("PAUSED");
        const ImVec2 sz = ImGui::CalcTextSize(text);
        const float x = canvasPos.x + canvasSize.x - sz.x - 16.0f * dpi - pad;
        chip(ImVec2(x, canvasPos.y + pad), text, IM_COL32(255, 176, 32, 230), IM_COL32(20, 20, 20, 255));
    }

    // 左下：光标坐标与缩放
    if (m_mouseInside) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "x %.2f   y %.2f   |   %.0f px/m", m_mouseWorld.x, m_mouseWorld.y,
                      ed.GetCamera().zoom);
        const float h = ImGui::GetTextLineHeight() + 8.0f * dpi;
        chip(ImVec2(canvasPos.x + pad, canvasPos.y + canvasSize.y - h - pad), buf, chipBg, chipText);
    }

    // 顶部居中：提示消息（淡出）
    float y = canvasPos.y + pad + 34.0f * dpi;
    for (const Toast& t : ed.GetToasts()) {
        const float alpha = std::min(1.0f, t.timeLeft / 0.4f);
        const ImVec2 sz = ImGui::CalcTextSize(t.text.c_str());
        const float x = canvasPos.x + 0.5f * (canvasSize.x - sz.x) - 12.0f * dpi;
        const ImVec2 s = chip(ImVec2(x, y), t.text.c_str(), theme::AccentU32(0.85f * alpha),
                              IM_COL32(255, 255, 255, static_cast<int>(255 * alpha)));
        y += s.y + 6.0f * dpi;
    }

    // 悬停物体的信息提示
    if (m_mouseInside && !m_dragging && ed.hoveredBody && ed.GetTool() == Tool::Select) {
        if (const kinema::Body* b = ed.GetWorld().FindBody(ed.hoveredBody)) {
            ImGui::BeginTooltip();
            ImGui::TextColored(theme::Accent(), "#%u  %s", b->GetId(), Tr(kinema::ToString(b->GetShapeType())));
            ImGui::TextDisabled("%s", Tr(kinema::ToString(b->GetType())));
            if (b->IsDynamic()) {
                ImGui::Text("%s: %.2f kg", Tr("Mass"), b->GetMass());
                ImGui::Text("%s: %.2f m/s", Tr("Speed"), b->GetLinearVelocity().Length());
            }
            ImGui::EndTooltip();
        }
    }
}

}  // namespace app
