#include "render/Shader.h"

#include <vector>

namespace app {

namespace {

gl::GLuint Compile(gl::GLenum type, const char* src, std::string* error) {
    const gl::GLuint shader = gl::CreateShader(type);
    gl::ShaderSource(shader, 1, &src, nullptr);
    gl::CompileShader(shader);

    gl::GLint ok = 0;
    gl::GetShaderiv(shader, gl::COMPILE_STATUS, &ok);
    if (!ok) {
        gl::GLint len = 0;
        gl::GetShaderiv(shader, gl::INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<size_t>(len > 1 ? len : 1));
        gl::GetShaderInfoLog(shader, static_cast<gl::GLsizei>(log.size()), nullptr, log.data());
        if (error) *error = std::string(type == gl::VERTEX_SHADER ? "Vertex" : "Fragment") + " shader: " + log.data();
        gl::DeleteShader(shader);
        return 0;
    }
    return shader;
}

}  // namespace

bool Shader::Create(const char* vertexSrc, const char* fragmentSrc, std::string* error) {
    Destroy();

    const gl::GLuint vs = Compile(gl::VERTEX_SHADER, vertexSrc, error);
    if (!vs) return false;
    const gl::GLuint fs = Compile(gl::FRAGMENT_SHADER, fragmentSrc, error);
    if (!fs) {
        gl::DeleteShader(vs);
        return false;
    }

    m_program = gl::CreateProgram();
    gl::AttachShader(m_program, vs);
    gl::AttachShader(m_program, fs);
    gl::LinkProgram(m_program);
    gl::DeleteShader(vs);  // 链接后着色器对象就可以删除了
    gl::DeleteShader(fs);

    gl::GLint ok = 0;
    gl::GetProgramiv(m_program, gl::LINK_STATUS, &ok);
    if (!ok) {
        gl::GLint len = 0;
        gl::GetProgramiv(m_program, gl::INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<size_t>(len > 1 ? len : 1));
        gl::GetProgramInfoLog(m_program, static_cast<gl::GLsizei>(log.size()), nullptr, log.data());
        if (error) *error = std::string("Link: ") + log.data();
        Destroy();
        return false;
    }
    return true;
}

void Shader::Destroy() {
    if (m_program) {
        gl::DeleteProgram(m_program);
        m_program = 0;
    }
}

}  // namespace app
