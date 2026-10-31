// =============================================================================
// 颜色打包：把 RGBA 颜色存进 Body::userData（uint64_t）
//
// 引擎本身不关心颜色；但示例场景想给物体上色，于是约定：
//   userData 的第 32 位为 1 表示"包含颜色"，低 32 位是 0xRRGGBBAA。
// =============================================================================
#pragma once

#include <cstdint>

namespace kinema::scene {

struct Rgba {
    uint8_t r = 255, g = 255, b = 255, a = 255;
};

inline constexpr uint64_t kColorFlag = 1ull << 32;

constexpr uint64_t PackColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return kColorFlag | (static_cast<uint64_t>(r) << 24) | (static_cast<uint64_t>(g) << 16) |
           (static_cast<uint64_t>(b) << 8) | static_cast<uint64_t>(a);
}

/// 0xRRGGBB 十六进制写法
constexpr uint64_t HexColor(uint32_t rgb) {
    return PackColor(static_cast<uint8_t>((rgb >> 16) & 0xFF), static_cast<uint8_t>((rgb >> 8) & 0xFF),
                     static_cast<uint8_t>(rgb & 0xFF));
}

constexpr bool UnpackColor(uint64_t userData, Rgba& out) {
    if ((userData & kColorFlag) == 0) return false;
    out.r = static_cast<uint8_t>((userData >> 24) & 0xFF);
    out.g = static_cast<uint8_t>((userData >> 16) & 0xFF);
    out.b = static_cast<uint8_t>((userData >> 8) & 0xFF);
    out.a = static_cast<uint8_t>(userData & 0xFF);
    return true;
}

/// 调色板（适合深色背景的现代配色）
namespace palette {
inline constexpr uint64_t Blue = HexColor(0x4F8CFF);
inline constexpr uint64_t Coral = HexColor(0xFF6B6B);
inline constexpr uint64_t Yellow = HexColor(0xFFD166);
inline constexpr uint64_t Green = HexColor(0x06D6A0);
inline constexpr uint64_t Purple = HexColor(0xA78BFA);
inline constexpr uint64_t Orange = HexColor(0xF78C6B);
inline constexpr uint64_t Cyan = HexColor(0x4CC9F0);
inline constexpr uint64_t Pink = HexColor(0xEF476F);
inline constexpr uint64_t Ground = HexColor(0x3B4252);
inline constexpr uint64_t Steel = HexColor(0x8892A6);

inline constexpr uint64_t kCycle[] = {Blue, Coral, Yellow, Green, Purple, Orange, Cyan, Pink};
inline constexpr int kCycleCount = static_cast<int>(sizeof(kCycle) / sizeof(kCycle[0]));

constexpr uint64_t Pick(int i) { return kCycle[((i % kCycleCount) + kCycleCount) % kCycleCount]; }
}  // namespace palette

}  // namespace kinema::scene
