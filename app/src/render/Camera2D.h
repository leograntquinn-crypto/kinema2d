// =============================================================================
// Camera2D：世界坐标（米，y 向上）<-> 屏幕坐标（像素，y 向下）
// =============================================================================
#pragma once

#include <kinema/math/AABB.h>
#include <kinema/math/Math.h>

namespace app {

using kinema::AABB;
using kinema::Vec2;

class Camera2D {
public:
    Vec2 center{0.0f, 10.0f};  // 视口中心对应的世界坐标
    float zoom = 30.0f;        // 每米多少像素
    Vec2 viewport{1280.0f, 720.0f};

    /// 期望的视野高度（米）。视口尺寸确定前无法换算 zoom，所以先记下来，
    /// 等 Viewport 面板知道自己的像素大小后再应用（见 ApplyPendingFit）。
    float pendingViewHeight = 0.0f;

    static constexpr float kMinZoom = 2.0f;
    static constexpr float kMaxZoom = 800.0f;

    /// 屏幕坐标（相对视口左上角）-> 世界坐标
    [[nodiscard]] Vec2 ScreenToWorld(Vec2 screen) const {
        return {center.x + (screen.x - 0.5f * viewport.x) / zoom, center.y - (screen.y - 0.5f * viewport.y) / zoom};
    }

    [[nodiscard]] Vec2 WorldToScreen(Vec2 world) const {
        return {(world.x - center.x) * zoom + 0.5f * viewport.x, (center.y - world.y) * zoom + 0.5f * viewport.y};
    }

    /// 一个像素对应多少米
    [[nodiscard]] float PixelSize() const { return 1.0f / zoom; }

    [[nodiscard]] AABB VisibleBounds() const {
        const Vec2 half{0.5f * viewport.x / zoom, 0.5f * viewport.y / zoom};
        return {center - half, center + half};
    }

    /// 以屏幕上某点为中心缩放：保持该点下的世界坐标不变
    void ZoomAt(Vec2 screen, float factor) {
        const Vec2 before = ScreenToWorld(screen);
        zoom = kinema::Clamp(zoom * factor, kMinZoom, kMaxZoom);
        const Vec2 after = ScreenToWorld(screen);
        center += before - after;
    }

    void SetViewHeight(float meters) {
        if (meters > 0.0f && viewport.y > 0.0f) zoom = kinema::Clamp(viewport.y / meters, kMinZoom, kMaxZoom);
        pendingViewHeight = meters;
    }

    void ApplyPendingFit() {
        if (pendingViewHeight > 0.0f && viewport.y > 0.0f) {
            zoom = kinema::Clamp(viewport.y / pendingViewHeight, kMinZoom, kMaxZoom);
            pendingViewHeight = 0.0f;
        }
    }

    /// 正交投影矩阵（列主序，直接传给 glUniformMatrix4fv）
    void BuildProjection(float m[16]) const {
        const AABB b = VisibleBounds();
        const float l = b.lower.x, r = b.upper.x, bt = b.lower.y, t = b.upper.y;
        for (int i = 0; i < 16; ++i) m[i] = 0.0f;
        m[0] = 2.0f / (r - l);
        m[5] = 2.0f / (t - bt);
        m[10] = -1.0f;
        m[12] = -(r + l) / (r - l);
        m[13] = -(t + bt) / (t - bt);
        m[15] = 1.0f;
    }
};

}  // namespace app
