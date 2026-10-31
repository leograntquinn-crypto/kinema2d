#include "render/Framebuffer.h"

#include <algorithm>
#include <cstring>

namespace app {

bool Framebuffer::Resize(int width, int height, int samples) {
    width = std::max(width, 1);
    height = std::max(height, 1);

    gl::GLint maxSamples = 0;
    gl::GetIntegerv(gl::MAX_SAMPLES, &maxSamples);
    samples = std::clamp(samples, 0, static_cast<int>(maxSamples));

    if (width == m_width && height == m_height && samples == m_samples && m_resolveFbo != 0) return true;

    Destroy();
    m_width = width;
    m_height = height;
    m_samples = samples;

    // ---- 解析目标：普通 RGBA8 纹理 ----
    gl::GenTextures(1, &m_texture);
    gl::BindTexture(gl::TEXTURE_2D, m_texture);
    gl::TexImage2D(gl::TEXTURE_2D, 0, static_cast<gl::GLint>(gl::RGBA8), width, height, 0, gl::RGBA, gl::UNSIGNED_BYTE,
                   nullptr);
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, static_cast<gl::GLint>(gl::LINEAR));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, static_cast<gl::GLint>(gl::LINEAR));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, static_cast<gl::GLint>(gl::CLAMP_TO_EDGE));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, static_cast<gl::GLint>(gl::CLAMP_TO_EDGE));
    gl::BindTexture(gl::TEXTURE_2D, 0);

    gl::GenFramebuffers(1, &m_resolveFbo);
    gl::BindFramebuffer(gl::FRAMEBUFFER, m_resolveFbo);
    gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT0, gl::TEXTURE_2D, m_texture, 0);
    bool ok = gl::CheckFramebufferStatus(gl::FRAMEBUFFER) == gl::FRAMEBUFFER_COMPLETE;

    // ---- 多重采样渲染缓冲 ----
    if (ok && samples > 0) {
        gl::GenRenderbuffers(1, &m_msaaColor);
        gl::BindRenderbuffer(gl::RENDERBUFFER, m_msaaColor);
        gl::RenderbufferStorageMultisample(gl::RENDERBUFFER, samples, gl::RGBA8, width, height);
        gl::BindRenderbuffer(gl::RENDERBUFFER, 0);

        gl::GenFramebuffers(1, &m_msaaFbo);
        gl::BindFramebuffer(gl::FRAMEBUFFER, m_msaaFbo);
        gl::FramebufferRenderbuffer(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT0, gl::RENDERBUFFER, m_msaaColor);
        if (gl::CheckFramebufferStatus(gl::FRAMEBUFFER) != gl::FRAMEBUFFER_COMPLETE) {
            // 驱动不支持：退化为无抗锯齿
            gl::DeleteFramebuffers(1, &m_msaaFbo);
            gl::DeleteRenderbuffers(1, &m_msaaColor);
            m_msaaFbo = m_msaaColor = 0;
            m_samples = 0;
        }
    }

    gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
    return ok;
}

void Framebuffer::Destroy() {
    if (m_msaaFbo) gl::DeleteFramebuffers(1, &m_msaaFbo);
    if (m_msaaColor) gl::DeleteRenderbuffers(1, &m_msaaColor);
    if (m_resolveFbo) gl::DeleteFramebuffers(1, &m_resolveFbo);
    if (m_texture) gl::DeleteTextures(1, &m_texture);
    m_msaaFbo = m_msaaColor = m_resolveFbo = m_texture = 0;
    m_width = m_height = m_samples = 0;
}

void Framebuffer::Bind() const {
    gl::BindFramebuffer(gl::FRAMEBUFFER, m_msaaFbo ? m_msaaFbo : m_resolveFbo);
    gl::Viewport(0, 0, m_width, m_height);
}

void Framebuffer::BindDefault() { gl::BindFramebuffer(gl::FRAMEBUFFER, 0); }

void Framebuffer::Resolve() const {
    if (!m_msaaFbo) return;
    gl::BindFramebuffer(gl::READ_FRAMEBUFFER, m_msaaFbo);
    gl::BindFramebuffer(gl::DRAW_FRAMEBUFFER, m_resolveFbo);
    gl::BlitFramebuffer(0, 0, m_width, m_height, 0, 0, m_width, m_height, gl::COLOR_BUFFER_BIT, gl::NEAREST);
    gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
}

bool Framebuffer::ReadPixels(std::vector<uint8_t>& out) const {
    if (!m_resolveFbo) return false;
    const size_t stride = static_cast<size_t>(m_width) * 4;
    out.resize(stride * static_cast<size_t>(m_height));

    gl::BindFramebuffer(gl::FRAMEBUFFER, m_resolveFbo);
    gl::PixelStorei(gl::PACK_ALIGNMENT, 1);
    gl::ReadPixels(0, 0, m_width, m_height, gl::RGBA, gl::UNSIGNED_BYTE, out.data());
    gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

    // OpenGL 的原点在左下角，图片文件的原点在左上角：上下翻转
    std::vector<uint8_t> row(stride);
    for (int y = 0; y < m_height / 2; ++y) {
        uint8_t* a = out.data() + static_cast<size_t>(y) * stride;
        uint8_t* b = out.data() + static_cast<size_t>(m_height - 1 - y) * stride;
        std::memcpy(row.data(), a, stride);
        std::memcpy(a, b, stride);
        std::memcpy(b, row.data(), stride);
    }
    return true;
}

}  // namespace app
