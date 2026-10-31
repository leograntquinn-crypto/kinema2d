// =============================================================================
// Renderer2D：批量 2D 几何渲染器
//
// 核心思想 —— 批处理（Batching）：
//   每次 DrawXxx() 只是把三角形顶点追加到 CPU 端数组，
//   在 End() 或缓冲区满时才一次性上传 GPU 并调用一次 DrawArrays。
//   几千个物体也只需要几次 draw call。
//
// 所有图元（包括"线"）都被展开成三角形：
//   OpenGL Core Profile 不支持宽度 > 1 的线，所以粗线用矩形（两个三角形）表示。
//   线宽以"像素"为单位，与缩放无关，看起来始终一样粗。
// =============================================================================
#pragma once

#include <string>
#include <vector>

#include "render/Camera2D.h"
#include "render/Color.h"
#include "render/Shader.h"

namespace app {

class Renderer2D {
public:
    Renderer2D() = default;
    ~Renderer2D() { Shutdown(); }

    Renderer2D(const Renderer2D&) = delete;
    Renderer2D& operator=(const Renderer2D&) = delete;

    bool Init(std::string* error);
    void Shutdown();

    /// 开始一帧：设置相机投影
    void Begin(const Camera2D& camera);
    /// 提交剩余顶点
    void End();

    // ---- 图元 ----
    void Triangle(Vec2 a, Vec2 b, Vec2 c, Color color);
    void Line(Vec2 a, Vec2 b, Color color, float widthPx = 1.5f);
    void ConvexPolyFilled(const Vec2* v, int count, Color color);
    /// 闭合多边形描边（斜接 miter 连接，拐角干净）
    void PolyOutline(const Vec2* v, int count, Color color, float widthPx = 1.5f);
    void CircleFilled(Vec2 center, float radius, Color color);
    void CircleOutline(Vec2 center, float radius, Color color, float widthPx = 1.5f);
    /// 固定像素大小的点
    void Point(Vec2 p, float sizePx, Color color);
    void Arrow(Vec2 from, Vec2 to, Color color, float widthPx = 1.5f);
    void RectFilled(Vec2 lower, Vec2 upper, Color color);
    void RectOutline(Vec2 lower, Vec2 upper, Color color, float widthPx = 1.0f);

    [[nodiscard]] float PixelSize() const { return m_pixelSize; }
    [[nodiscard]] int DrawCalls() const { return m_drawCalls; }
    [[nodiscard]] size_t VertexCount() const { return m_totalVertices; }

private:
    struct Vertex {
        float x, y;
        uint32_t color;
    };

    void Push(Vec2 p, uint32_t color);
    void Flush();
    [[nodiscard]] int CircleSegments(float radius) const;

    Shader m_shader;
    gl::GLuint m_vao = 0, m_vbo = 0;
    gl::GLint m_projLoc = -1;

    std::vector<Vertex> m_vertices;
    std::vector<Vec2> m_scratch;
    float m_pixelSize = 1.0f / 30.0f;
    int m_drawCalls = 0;
    size_t m_totalVertices = 0;

    static constexpr size_t kMaxVertices = 3 * 32768;
};

}  // namespace app
