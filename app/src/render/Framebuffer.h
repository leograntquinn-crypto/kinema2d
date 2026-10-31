// =============================================================================
// Framebuffer：离屏渲染目标
//
// 物理场景先渲染到这里（带 4x MSAA 抗锯齿），再解析（resolve）成普通纹理，
// 最后作为一张图片显示在 ImGui 的"Viewport"面板中 —— 和 Unity / Unreal 编辑器的做法一样。
//
//   [MSAA 渲染缓冲] --BlitFramebuffer--> [解析纹理] --ImGui::Image--> 屏幕
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

#include "render/GL.h"

namespace app {

class Framebuffer {
public:
    Framebuffer() = default;
    ~Framebuffer() { Destroy(); }

    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;

    /// 尺寸或采样数变化时重新创建
    bool Resize(int width, int height, int samples);
    void Destroy();

    void Bind() const;
    static void BindDefault();

    /// 把 MSAA 结果解析到可采样纹理
    void Resolve() const;

    /// 读取解析后的像素（RGBA8，从上到下）
    bool ReadPixels(std::vector<uint8_t>& out) const;

    [[nodiscard]] gl::GLuint GetTexture() const { return m_texture; }
    [[nodiscard]] int Width() const { return m_width; }
    [[nodiscard]] int Height() const { return m_height; }
    [[nodiscard]] int Samples() const { return m_samples; }

private:
    int m_width = 0, m_height = 0, m_samples = 0;
    gl::GLuint m_msaaFbo = 0, m_msaaColor = 0;  // 多重采样
    gl::GLuint m_resolveFbo = 0, m_texture = 0;  // 解析目标
};

}  // namespace app
