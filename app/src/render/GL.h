// =============================================================================
// 极简 OpenGL 3.3 Core 函数加载器
//
// 为什么自己写而不用 glad/glew？
//   * 我们只用到 ~50 个函数，几十行宏就能搞定，没有额外依赖；
//   * 能清楚地看到"OpenGL 函数其实就是运行时从驱动里查出来的函数指针"。
//
// 用法：gl::Load(glfwGetProcAddress) 之后，调用 gl::Clear(gl::COLOR_BUFFER_BIT) 等。
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
#define KGL_APIENTRY __stdcall
#else
#define KGL_APIENTRY
#endif

namespace gl {

using GLenum = unsigned int;
using GLuint = unsigned int;
using GLint = int;
using GLsizei = int;
using GLboolean = unsigned char;
using GLbitfield = unsigned int;
using GLfloat = float;
using GLchar = char;
using GLubyte = unsigned char;
using GLsizeiptr = std::ptrdiff_t;
using GLintptr = std::ptrdiff_t;

// ---- 常量 ----
inline constexpr GLbitfield COLOR_BUFFER_BIT = 0x00004000;
inline constexpr GLbitfield DEPTH_BUFFER_BIT = 0x00000100;
inline constexpr GLenum TRIANGLES = 0x0004;
inline constexpr GLenum BLEND = 0x0BE2;
inline constexpr GLenum SCISSOR_TEST = 0x0C11;
inline constexpr GLenum DEPTH_TEST = 0x0B71;
inline constexpr GLenum CULL_FACE = 0x0B44;
inline constexpr GLenum MULTISAMPLE = 0x809D;
inline constexpr GLenum ONE = 1;
inline constexpr GLenum SRC_ALPHA = 0x0302;
inline constexpr GLenum ONE_MINUS_SRC_ALPHA = 0x0303;
inline constexpr GLenum UNSIGNED_BYTE = 0x1401;
inline constexpr GLenum FLOAT = 0x1406;
inline constexpr GLenum ARRAY_BUFFER = 0x8892;
inline constexpr GLenum STREAM_DRAW = 0x88E0;
inline constexpr GLenum DYNAMIC_DRAW = 0x88E8;
inline constexpr GLenum VERTEX_SHADER = 0x8B31;
inline constexpr GLenum FRAGMENT_SHADER = 0x8B30;
inline constexpr GLenum COMPILE_STATUS = 0x8B81;
inline constexpr GLenum LINK_STATUS = 0x8B82;
inline constexpr GLenum INFO_LOG_LENGTH = 0x8B84;
inline constexpr GLenum FRAMEBUFFER = 0x8D40;
inline constexpr GLenum READ_FRAMEBUFFER = 0x8CA8;
inline constexpr GLenum DRAW_FRAMEBUFFER = 0x8CA9;
inline constexpr GLenum FRAMEBUFFER_BINDING = 0x8CA6;
inline constexpr GLenum RENDERBUFFER = 0x8D41;
inline constexpr GLenum COLOR_ATTACHMENT0 = 0x8CE0;
inline constexpr GLenum FRAMEBUFFER_COMPLETE = 0x8CD5;
inline constexpr GLenum MAX_SAMPLES = 0x8D57;
inline constexpr GLenum TEXTURE_2D = 0x0DE1;
inline constexpr GLenum RGBA = 0x1908;
inline constexpr GLenum RGBA8 = 0x8058;
inline constexpr GLenum TEXTURE_MIN_FILTER = 0x2801;
inline constexpr GLenum TEXTURE_MAG_FILTER = 0x2800;
inline constexpr GLenum TEXTURE_WRAP_S = 0x2802;
inline constexpr GLenum TEXTURE_WRAP_T = 0x2803;
inline constexpr GLenum LINEAR = 0x2601;
inline constexpr GLenum NEAREST = 0x2600;
inline constexpr GLenum CLAMP_TO_EDGE = 0x812F;
inline constexpr GLenum PACK_ALIGNMENT = 0x0D05;
inline constexpr GLenum VENDOR = 0x1F00;
inline constexpr GLenum RENDERER = 0x1F01;
inline constexpr GLenum VERSION = 0x1F02;
inline constexpr GLenum READ_BUFFER = 0x0C02;
inline constexpr GLenum BACK = 0x0405;

// ---- 函数表：X(返回值, 名字, 参数列表) ----
#define KINEMA_GL_FUNCTIONS(X)                                                                                  \
    X(void, Clear, (GLbitfield mask))                                                                           \
    X(void, ClearColor, (GLfloat r, GLfloat g, GLfloat b, GLfloat a))                                           \
    X(void, Viewport, (GLint x, GLint y, GLsizei w, GLsizei h))                                                 \
    X(void, Enable, (GLenum cap))                                                                               \
    X(void, Disable, (GLenum cap))                                                                              \
    X(void, BlendFunc, (GLenum sfactor, GLenum dfactor))                                                        \
    X(GLenum, GetError, ())                                                                                     \
    X(void, GetIntegerv, (GLenum pname, GLint * data))                                                          \
    X(const GLubyte*, GetString, (GLenum name))                                                                 \
    X(void, ReadBuffer, (GLenum mode))                                                                          \
    X(void, ReadPixels, (GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, void* data))       \
    X(void, PixelStorei, (GLenum pname, GLint param))                                                           \
    X(GLuint, CreateShader, (GLenum type))                                                                      \
    X(void, ShaderSource, (GLuint shader, GLsizei count, const GLchar* const* str, const GLint* len))           \
    X(void, CompileShader, (GLuint shader))                                                                     \
    X(void, GetShaderiv, (GLuint shader, GLenum pname, GLint * params))                                         \
    X(void, GetShaderInfoLog, (GLuint shader, GLsizei bufSize, GLsizei * length, GLchar * infoLog))             \
    X(void, DeleteShader, (GLuint shader))                                                                      \
    X(GLuint, CreateProgram, ())                                                                                \
    X(void, AttachShader, (GLuint program, GLuint shader))                                                      \
    X(void, LinkProgram, (GLuint program))                                                                      \
    X(void, GetProgramiv, (GLuint program, GLenum pname, GLint * params))                                       \
    X(void, GetProgramInfoLog, (GLuint program, GLsizei bufSize, GLsizei * length, GLchar * infoLog))           \
    X(void, DeleteProgram, (GLuint program))                                                                    \
    X(void, UseProgram, (GLuint program))                                                                       \
    X(GLint, GetUniformLocation, (GLuint program, const GLchar* name))                                          \
    X(void, UniformMatrix4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat* value))       \
    X(void, GenVertexArrays, (GLsizei n, GLuint * arrays))                                                      \
    X(void, BindVertexArray, (GLuint array))                                                                    \
    X(void, DeleteVertexArrays, (GLsizei n, const GLuint* arrays))                                              \
    X(void, GenBuffers, (GLsizei n, GLuint * buffers))                                                          \
    X(void, BindBuffer, (GLenum target, GLuint buffer))                                                         \
    X(void, BufferData, (GLenum target, GLsizeiptr size, const void* data, GLenum usage))                       \
    X(void, BufferSubData, (GLenum target, GLintptr offset, GLsizeiptr size, const void* data))                 \
    X(void, DeleteBuffers, (GLsizei n, const GLuint* buffers))                                                  \
    X(void, EnableVertexAttribArray, (GLuint index))                                                            \
    X(void, VertexAttribPointer,                                                                                \
      (GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer))       \
    X(void, DrawArrays, (GLenum mode, GLint first, GLsizei count))                                              \
    X(void, GenFramebuffers, (GLsizei n, GLuint * ids))                                                         \
    X(void, BindFramebuffer, (GLenum target, GLuint framebuffer))                                               \
    X(void, FramebufferTexture2D, (GLenum target, GLenum attachment, GLenum textarget, GLuint tex, GLint level)) \
    X(void, FramebufferRenderbuffer, (GLenum target, GLenum attachment, GLenum rbtarget, GLuint rb))            \
    X(GLenum, CheckFramebufferStatus, (GLenum target))                                                          \
    X(void, DeleteFramebuffers, (GLsizei n, const GLuint* framebuffers))                                        \
    X(void, GenRenderbuffers, (GLsizei n, GLuint * renderbuffers))                                              \
    X(void, BindRenderbuffer, (GLenum target, GLuint renderbuffer))                                             \
    X(void, RenderbufferStorageMultisample,                                                                     \
      (GLenum target, GLsizei samples, GLenum internalformat, GLsizei width, GLsizei height))                   \
    X(void, DeleteRenderbuffers, (GLsizei n, const GLuint* renderbuffers))                                      \
    X(void, BlitFramebuffer, (GLint srcX0, GLint srcY0, GLint srcX1, GLint srcY1, GLint dstX0, GLint dstY0,     \
                              GLint dstX1, GLint dstY1, GLbitfield mask, GLenum filter))                        \
    X(void, GenTextures, (GLsizei n, GLuint * textures))                                                        \
    X(void, BindTexture, (GLenum target, GLuint texture))                                                       \
    X(void, TexImage2D, (GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height,       \
                         GLint border, GLenum format, GLenum type, const void* pixels))                         \
    X(void, TexParameteri, (GLenum target, GLenum pname, GLint param))                                          \
    X(void, DeleteTextures, (GLsizei n, const GLuint* textures))

#define KINEMA_GL_DECLARE(ret, name, args) \
    using PFN_##name = ret(KGL_APIENTRY*) args; \
    extern PFN_##name name;
KINEMA_GL_FUNCTIONS(KINEMA_GL_DECLARE)
#undef KINEMA_GL_DECLARE

using GenericProc = void (*)();
using LoadProc = GenericProc (*)(const char* name);

/// 加载所有函数指针。失败时返回 false，missing 指向第一个缺失的函数名。
bool Load(LoadProc loader, const char** missing = nullptr);

}  // namespace gl
