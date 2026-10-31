// =============================================================================
// Color：8 位 RGBA 颜色 + 常用的颜色运算
// =============================================================================
#pragma once

#include <algorithm>
#include <cstdint>

namespace app {

struct Color {
    uint8_t r = 255, g = 255, b = 255, a = 255;

    constexpr Color() = default;
    constexpr Color(uint8_t r_, uint8_t g_, uint8_t b_, uint8_t a_ = 255) : r(r_), g(g_), b(b_), a(a_) {}

    static constexpr Color Hex(uint32_t rgb, uint8_t alpha = 255) {
        return {static_cast<uint8_t>((rgb >> 16) & 0xFF), static_cast<uint8_t>((rgb >> 8) & 0xFF),
                static_cast<uint8_t>(rgb & 0xFF), alpha};
    }

    static Color FromFloat(float fr, float fg, float fb, float fa = 1.0f) {
        auto c = [](float v) { return static_cast<uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
        return {c(fr), c(fg), c(fb), c(fa)};
    }

    [[nodiscard]] constexpr Color WithAlpha(uint8_t alpha) const { return {r, g, b, alpha}; }

    /// 与另一颜色线性混合（t=0 返回自身，t=1 返回 other）
    [[nodiscard]] Color Mix(Color o, float t) const {
        auto m = [t](uint8_t x, uint8_t y) {
            return static_cast<uint8_t>(static_cast<float>(x) + (static_cast<float>(y) - static_cast<float>(x)) * t);
        };
        return {m(r, o.r), m(g, o.g), m(b, o.b), m(a, o.a)};
    }

    [[nodiscard]] Color Lighter(float t) const { return Mix(Color(255, 255, 255, a), t); }
    [[nodiscard]] Color Darker(float t) const { return Mix(Color(0, 0, 0, a), t); }

    /// 打包成 GPU 顶点格式（内存中依次为 r,g,b,a）
    [[nodiscard]] constexpr uint32_t Packed() const {
        return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) | (static_cast<uint32_t>(b) << 16) |
               (static_cast<uint32_t>(a) << 24);
    }

    [[nodiscard]] float Rf() const { return static_cast<float>(r) / 255.0f; }
    [[nodiscard]] float Gf() const { return static_cast<float>(g) / 255.0f; }
    [[nodiscard]] float Bf() const { return static_cast<float>(b) / 255.0f; }
    [[nodiscard]] float Af() const { return static_cast<float>(a) / 255.0f; }
};

}  // namespace app
