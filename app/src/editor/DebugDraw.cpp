#include "editor/DebugDraw.h"

#include <algorithm>
#include <cmath>

#include <kinema_scene/Color.h>

namespace app {

using kinema::Vec2;

Color BodyColor(const kinema::Body& body) {
    kinema::scene::Rgba c;
    if (kinema::scene::UnpackColor(body.GetUserData(), c)) return {c.r, c.g, c.b, c.a};
    if (body.IsStatic()) return theme::Palette().staticFill;
    kinema::scene::UnpackColor(kinema::scene::palette::Pick(static_cast<int>(body.GetId())), c);
    return {c.r, c.g, c.b, c.a};
}

void DrawGrid(Renderer2D& r, const Camera2D& camera, const ViewportPalette& palette) {
    const kinema::AABB view = camera.VisibleBounds();

    // 根据缩放自动选择网格间距：保证屏幕上的格子不会太密
    float spacing = 1.0f;
    while (spacing * camera.zoom < 18.0f) spacing *= 5.0f;
    while (spacing * camera.zoom > 120.0f) spacing /= 5.0f;
    const float major = spacing * 5.0f;

    const auto drawLines = [&](float step, Color color, float width) {
        const float x0 = std::floor(view.lower.x / step) * step;
        for (float x = x0; x <= view.upper.x; x += step) r.Line({x, view.lower.y}, {x, view.upper.y}, color, width);
        const float y0 = std::floor(view.lower.y / step) * step;
        for (float y = y0; y <= view.upper.y; y += step) r.Line({view.lower.x, y}, {view.upper.x, y}, color, width);
    };

    drawLines(spacing, palette.gridMinor, 1.0f);
    drawLines(major, palette.gridMajor, 1.0f);

    // 坐标轴
    r.Line({view.lower.x, 0.0f}, {view.upper.x, 0.0f}, palette.axisX, 1.5f);
    r.Line({0.0f, view.lower.y}, {0.0f, view.upper.y}, palette.axisY, 1.5f);
}

void DrawShape(Renderer2D& r, const kinema::Shape& shape, const kinema::Transform& xf, Color fill, Color outline,
               float outlineWidth, bool rotationMarker) {
    if (shape.GetType() == kinema::ShapeType::Circle) {
        const auto& circle = static_cast<const kinema::CircleShape&>(shape);
        const float radius = circle.GetRadius();
        if (fill.a > 0) r.CircleFilled(xf.p, radius, fill);
        if (outline.a > 0) r.CircleOutline(xf.p, radius, outline, outlineWidth);
        // 半径线：让圆的旋转可见
        if (rotationMarker) r.Line(xf.p, xf.p + radius * xf.q.XAxis(), outline, outlineWidth);
    } else {
        const auto& poly = static_cast<const kinema::PolygonShape&>(shape);
        Vec2 verts[kinema::kMaxPolygonVertices];
        const int n = poly.GetVertexCount();
        for (int i = 0; i < n; ++i) verts[i] = xf.Apply(poly.GetVertex(i));
        if (fill.a > 0) r.ConvexPolyFilled(verts, n, fill);
        if (outline.a > 0) r.PolyOutline(verts, n, outline, outlineWidth);
    }
}

void DrawWorld(Renderer2D& r, const Camera2D& camera, const kinema::World& world, const DebugDrawSettings& s,
               const ViewportPalette& palette, kinema::BodyId selected, kinema::BodyId hovered) {
    const Color gray(128, 132, 140, 255);
    // 视锥剔除：完全在屏幕外的物体不绘制
    const kinema::AABB view = camera.VisibleBounds();

    // ---- 物体 ----
    if (s.shapes) {
        for (const auto& bp : world.GetBodies()) {
            const kinema::Body& b = *bp;
            if (!kinema::Overlaps(b.GetAABB(), view)) continue;
            Color base = BodyColor(b);
            if (s.sleepTint && !b.IsStatic() && !b.IsAwake()) base = base.Mix(gray, 0.35f);

            const auto alpha = static_cast<uint8_t>(255.0f * (b.IsStatic() ? 1.0f : s.fillAlpha));
            const Color fill = base.WithAlpha(alpha);
            Color outline = s.outlines ? base.Lighter(0.25f) : Color(0, 0, 0, 0);
            float width = 1.5f;
            if (b.GetId() == hovered) {
                outline = palette.hover;
                width = 2.0f;
            }
            if (b.GetId() == selected) {
                outline = palette.selection;
                width = 2.5f;
            }
            DrawShape(r, b.GetShape(), b.GetTransform(), fill, outline, width,
                      s.rotationMarkers && b.GetShapeType() == kinema::ShapeType::Circle);
        }
    }

    // ---- AABB ----
    if (s.aabbs) {
        for (const auto& b : world.GetBodies()) {
            r.RectOutline(b->GetAABB().lower, b->GetAABB().upper, palette.aabb, 1.0f);
        }
    }

    // ---- 质心 ----
    if (s.centers) {
        for (const auto& b : world.GetBodies()) {
            const Vec2 p = b->GetPosition();
            const float len = 8.0f * r.PixelSize();
            r.Line(p - len * b->GetTransform().q.XAxis(), p + len * b->GetTransform().q.XAxis(), palette.axisX, 1.5f);
            r.Line(p - len * b->GetTransform().q.YAxis(), p + len * b->GetTransform().q.YAxis(), palette.axisY, 1.5f);
        }
    }

    // ---- 速度向量 ----
    if (s.velocities) {
        for (const auto& b : world.GetBodies()) {
            if (!b->IsAwake()) continue;
            const Vec2 v = b->GetLinearVelocity();
            if (v.LengthSquared() < 1e-4f) continue;
            r.Arrow(b->GetPosition(), b->GetPosition() + 0.2f * v, palette.velocity, 1.5f);
        }
    }

    // ---- 关节 ----
    if (s.joints) {
        for (const auto& jp : world.GetJoints()) {
            const kinema::Joint& j = *jp;
            const Vec2 a = j.GetAnchorA();
            const Vec2 b = j.GetAnchorB();
            Color c = palette.joint.WithAlpha(190);
            if (j.GetType() == kinema::JointType::Mouse) c = palette.selection;

            if (j.GetType() == kinema::JointType::Distance &&
                static_cast<const kinema::DistanceJoint&>(j).GetFrequency() > 0.0f) {
                // 弹簧画成锯齿线
                Vec2 d = b - a;
                const float len = d.Normalize();
                const Vec2 n = kinema::LeftPerp(d) * (std::min(4.0f * r.PixelSize(), 0.12f * len));
                constexpr int kCoils = 8;
                Vec2 prev = a;
                for (int i = 1; i <= kCoils; ++i) {
                    const float t = static_cast<float>(i) / static_cast<float>(kCoils + 1);
                    const Vec2 p = a + (t * len) * d + ((i % 2) ? n : -n);
                    r.Line(prev, p, c.WithAlpha(120), 1.0f);
                    prev = p;
                }
                r.Line(prev, b, c.WithAlpha(120), 1.0f);
                continue;  // 软体中弹簧很多，不画端点以免遮挡物体
            } else if (j.GetType() == kinema::JointType::Revolute) {
                // 从质心连到锚点，能直观看出关节作用在物体的哪个位置（静态物体不画）
                if (j.GetBodyA() && !j.GetBodyA()->IsStatic()) r.Line(j.GetBodyA()->GetPosition(), a, c.WithAlpha(90), 1.0f);
                if (j.GetBodyB() && !j.GetBodyB()->IsStatic()) r.Line(j.GetBodyB()->GetPosition(), b, c.WithAlpha(90), 1.0f);
            } else {
                r.Line(a, b, c, 1.5f);
            }
            r.Point(a, 6.0f, c);
            r.Point(b, 6.0f, c);
        }
    }

    // ---- 接触 ----
    if (s.contacts || s.normals || s.impulses) {
        world.ForEachContact([&](const kinema::Contact& c) {
            const kinema::Manifold& m = c.GetManifold();
            for (int i = 0; i < m.pointCount; ++i) {
                const kinema::ManifoldPoint& mp = m.points[i];
                if (s.contacts) r.Point(mp.point, 6.0f, palette.contactPoint);
                if (s.normals) r.Arrow(mp.point, mp.point + 0.4f * m.normal, palette.contactNormal, 1.2f);
                if (s.impulses) {
                    r.Line(mp.point, mp.point + 0.1f * mp.normalImpulse * m.normal, palette.impulse, 2.0f);
                }
            }
        });
    }
}

}  // namespace app
