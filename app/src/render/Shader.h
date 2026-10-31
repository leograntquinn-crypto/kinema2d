// =============================================================================
// Shader：GLSL 程序的 RAII 封装
// =============================================================================
#pragma once

#include <string>

#include "render/GL.h"

namespace app {

class Shader {
public:
    Shader() = default;
    ~Shader() { Destroy(); }

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    /// 编译并链接；失败时返回 false，并把编译日志写入 error
    bool Create(const char* vertexSrc, const char* fragmentSrc, std::string* error);
    void Destroy();

    void Bind() const { gl::UseProgram(m_program); }
    [[nodiscard]] gl::GLint Uniform(const char* name) const { return gl::GetUniformLocation(m_program, name); }
    [[nodiscard]] bool IsValid() const { return m_program != 0; }

private:
    gl::GLuint m_program = 0;
};

}  // namespace app
