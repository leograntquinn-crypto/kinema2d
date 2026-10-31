#include "render/Renderer2D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace app {

namespace {

// 顶点着色器：把世界坐标乘以投影矩阵；颜色直接传给片元着色器
constexpr const char* kVertexShader = R"(#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uProj;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uProj * vec4(aPos, 0.0, 1.0);
}
)";

constexpr const char* kFragmentShader = R"(#version 330 core
in vec4 vColor;
out vec4 FragColor;
void main() {
    FragColor = vColor;
}
)";

}  // namespace

bool Renderer2D::Init(std::string* error) {
    if (!m_shader.Create(kVertexShader, kFragmentShader, error)) return false;
    m_projLoc = m_shader.Uniform("uProj");

    gl::GenVertexArrays(1, &m_vao);
    gl::GenBuffers(1, &m_vbo);
    gl::BindVertexArray(m_vao);
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo);
    gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(kMaxVertices * sizeof(Vertex)), nullptr,
                   gl::DYNAMIC_DRAW);

    // 顶点布局：vec2 位置 + 4 个 unsigned byte 颜色（归一化到 0~1）
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 2, gl::FLOAT, 0, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, x)));
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 4, gl::UNSIGNED_BYTE, 1, sizeof(Vertex),
                            reinterpret_cast<const void*>(offsetof(Vertex, color)));

    gl::BindVertexArray(0);
    gl::BindBuffer(gl::ARRAY_BUFFER, 0);

    m_vertices.reserve(kMaxVertices);
    return true;
}

void Renderer2D::Shutdown() {
    if (m_vbo) gl::DeleteBuffers(1, &m_vbo);
    if (m_vao) gl::DeleteVertexArrays(1, &m_vao);
    m_vbo = m_vao = 0;
    m_shader.Destroy();
}

void Renderer2D::Begin(const Camera2D& camera) {
    m_pixelSize = camera.PixelSize();
    m_drawCalls = 0;
    m_totalVertices = 0;
    m_vertices.clear();

    float proj[16];
    camera.BuildProjection(proj);
    m_shader.Bind();
    gl::UniformMatrix4fv(m_projLoc, 1, 0, proj);

    gl::Enable(gl::BLEND);
    gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
    gl::Disable(gl::DEPTH_TEST);
    gl::Disable(gl::CULL_FACE);
    gl::Disable(gl::SCISSOR_TEST);
}

void Renderer2D::End() { Flush(); }

void Renderer2D::Flush() {
    if (m_vertices.empty()) return;
    m_shader.Bind();
    gl::BindVertexArray(m_vao);
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo);
    gl::BufferSubData(gl::ARRAY_BUFFER, 0, static_cast<gl::GLsizeiptr>(m_vertices.size() * sizeof(Vertex)),
                      m_vertices.data());
    gl::DrawArrays(gl::TRIANGLES, 0, static_cast<gl::GLsizei>(m_vertices.size()));
    gl::BindVertexArray(0);

    ++m_drawCalls;
    m_totalVertices += m_vertices.size();
    m_vertices.clear();
}

void Renderer2D::Push(Vec2 p, uint32_t color) { m_vertices.push_back({p.x, p.y, color}); }

int Renderer2D::CircleSegments(float radius) const {
    const float screenRadius = radius / m_pixelSize;
    return kinema::Clamp(static_cast<int>(std::sqrt(screenRadius) * 4.0f), 12, 96);
}

void Renderer2D::Triangle(Vec2 a, Vec2 b, Vec2 c, Color color) {
    if (m_vertices.size() + 3 > kMaxVertices) Flush();
    const uint32_t col = color.Packed();
    Push(a, col);
    Push(b, col);
    Push(c, col);
}

void Renderer2D::Line(Vec2 a, Vec2 b, Color color, float widthPx) {
    Vec2 d = b - a;
    if (d.Normalize() == 0.0f) return;
    const Vec2 n = kinema::LeftPerp(d) * (0.5f * widthPx * m_pixelSize);
    Triangle(a + n, b + n, b - n, color);
    Triangle(a + n, b - n, a - n, color);
}

void Renderer2D::ConvexPolyFilled(const Vec2* v, int count, Color color) {
    for (int i = 1; i + 1 < count; ++i) Triangle(v[0], v[i], v[i + 1], color);  // 三角扇
}

void Renderer2D::PolyOutline(const Vec2* v, int count, Color color, float widthPx) {
    if (count < 2) return;
    const float half = 0.5f * widthPx * m_pixelSize;

    // 计算每个顶点的斜接偏移（两条相邻边法线的角平分线）
    m_scratch.resize(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        const Vec2 prev = v[(i + count - 1) % count];
        const Vec2 cur = v[i];
        const Vec2 next = v[(i + 1) % count];
        const Vec2 n0 = kinema::RightPerp(cur - prev).Normalized();
        const Vec2 n1 = kinema::RightPerp(next - cur).Normalized();
        Vec2 miter = (n0 + n1).Normalized();
        if (miter.LengthSquared() < 0.5f) miter = n1;
        const float denom = std::max(kinema::Dot(miter, n1), 0.3f);  // 限制尖角处的斜接长度
        m_scratch[static_cast<size_t>(i)] = miter * (half / denom);
    }

    for (int i = 0; i < count; ++i) {
        const int j = (i + 1) % count;
        const Vec2 oi = v[i] + m_scratch[static_cast<size_t>(i)];
        const Vec2 ii = v[i] - m_scratch[static_cast<size_t>(i)];
        const Vec2 oj = v[j] + m_scratch[static_cast<size_t>(j)];
        const Vec2 ij = v[j] - m_scratch[static_cast<size_t>(j)];
        Triangle(oi, oj, ij, color);
        Triangle(oi, ij, ii, color);
    }
}

void Renderer2D::CircleFilled(Vec2 center, float radius, Color color) {
    const int segments = CircleSegments(radius);
    const float step = 2.0f * kinema::kPi / static_cast<float>(segments);
    Vec2 prev = center + Vec2{radius, 0.0f};
    for (int i = 1; i <= segments; ++i) {
        const float a = step * static_cast<float>(i);
        const Vec2 cur = center + radius * Vec2{std::cos(a), std::sin(a)};
        Triangle(center, prev, cur, color);
        prev = cur;
    }
}

void Renderer2D::CircleOutline(Vec2 center, float radius, Color color, float widthPx) {
    const int segments = CircleSegments(radius);
    std::vector<Vec2> pts(static_cast<size_t>(segments));
    const float step = 2.0f * kinema::kPi / static_cast<float>(segments);
    for (int i = 0; i < segments; ++i) {
        const float a = step * static_cast<float>(i);
        pts[static_cast<size_t>(i)] = center + radius * Vec2{std::cos(a), std::sin(a)};
    }
    PolyOutline(pts.data(), segments, color, widthPx);
}

void Renderer2D::Point(Vec2 p, float sizePx, Color color) {
    const float r = 0.5f * sizePx * m_pixelSize;
    constexpr int kSegments = 10;
    const float step = 2.0f * kinema::kPi / kSegments;
    Vec2 prev = p + Vec2{r, 0.0f};
    for (int i = 1; i <= kSegments; ++i) {
        const float a = step * static_cast<float>(i);
        const Vec2 cur = p + r * Vec2{std::cos(a), std::sin(a)};
        Triangle(p, prev, cur, color);
        prev = cur;
    }
}

void Renderer2D::Arrow(Vec2 from, Vec2 to, Color color, float widthPx) {
    Vec2 d = to - from;
    const float len = d.Normalize();
    if (len <= 0.0f) return;
    const float head = std::min(10.0f * m_pixelSize, 0.4f * len);
    const Vec2 base = to - head * d;
    Line(from, base, color, widthPx);
    const Vec2 n = kinema::LeftPerp(d) * (0.6f * head);
    Triangle(to, base + n, base - n, color);
}

void Renderer2D::RectFilled(Vec2 lower, Vec2 upper, Color color) {
    Triangle(lower, {upper.x, lower.y}, upper, color);
    Triangle(lower, upper, {lower.x, upper.y}, color);
}

void Renderer2D::RectOutline(Vec2 lower, Vec2 upper, Color color, float widthPx) {
    const Vec2 pts[4] = {lower, {upper.x, lower.y}, upper, {lower.x, upper.y}};
    PolyOutline(pts, 4, color, widthPx);
}

}  // namespace app
