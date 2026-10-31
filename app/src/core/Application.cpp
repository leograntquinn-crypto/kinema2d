#include "core/Application.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#define GLFW_INCLUDE_NONE  // 我们使用自己的 OpenGL 加载器，不让 GLFW 包含系统 gl.h
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "editor/Editor.h"
#include "editor/Theme.h"
#include "render/GL.h"
#include "util/I18n.h"
#include "util/ImageWriter.h"
#include "util/Log.h"

namespace app {

namespace {

void GlfwErrorCallback(int code, const char* description) { Log::Error("GLFW error %d: %s", code, description); }

}  // namespace

Application::Application(AppConfig config) : m_config(std::move(config)) {}

Application::~Application() { Shutdown(); }

bool Application::Init() {
    glfwSetErrorCallback(GlfwErrorCallback);
    if (!glfwInit()) {
        Log::Error("Failed to initialize GLFW");
        return false;
    }

    // OpenGL 3.3 Core Profile（macOS 需要 forward-compatible）
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);  // Windows/Linux 高 DPI

    m_window = glfwCreateWindow(m_config.width, m_config.height, "Kinema2D", nullptr, nullptr);
    if (!m_window) {
        Log::Error("Failed to create window (OpenGL 3.3 required)");
        return false;
    }
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(m_config.vsync ? 1 : 0);

    const char* missing = nullptr;
    if (!gl::Load(reinterpret_cast<gl::LoadProc>(glfwGetProcAddress), &missing)) {
        Log::Error("Failed to load OpenGL function %s", missing ? missing : "?");
        return false;
    }
    Log::Info("OpenGL %s | %s", reinterpret_cast<const char*>(gl::GetString(gl::VERSION)),
              reinterpret_cast<const char*>(gl::GetString(gl::RENDERER)));

    // DPI 缩放
    float xscale = 1.0f, yscale = 1.0f;
    glfwGetWindowContentScale(m_window, &xscale, &yscale);
#ifdef __APPLE__
    const float dpiScale = 1.0f;  // macOS 由系统处理 Retina 缩放（帧缓冲本身就是 2x）
#else
    const float dpiScale = std::max(1.0f, xscale);
#endif

    // ---- ImGui ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = "kinema2d_layout.ini";  // 保存窗口布局
    io.ConfigWindowsMoveFromTitleBarOnly = true;

    if (m_config.language == "zh") SetLanguage(Language::Chinese);
    theme::LoadFonts(dpiScale);
    if (GetLanguage() == Language::Chinese && !theme::HasCjkFont()) SetLanguage(Language::English);
    theme::Apply(m_config.lightTheme ? ThemeMode::Light : ThemeMode::Dark, dpiScale);

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    m_imguiInitialized = true;

    // ---- 编辑器 ----
    m_editor = std::make_unique<Editor>();
    if (!m_editor->Init(dpiScale, m_config.scene)) return false;
    m_editor->Sim().paused = m_config.startPaused;
    return true;
}

void Application::Shutdown() {
    if (m_editor) {
        m_editor->Shutdown();
        m_editor.reset();
    }
    if (m_imguiInitialized) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        m_imguiInitialized = false;
    }
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
        glfwTerminate();
    }
}

bool Application::SaveWindowScreenshot(const std::string& path) const {
    int w = 0, h = 0;
    glfwGetFramebufferSize(m_window, &w, &h);
    if (w <= 0 || h <= 0) return false;

    std::vector<uint8_t> pixels(static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
    gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
    gl::ReadBuffer(gl::BACK);
    gl::PixelStorei(gl::PACK_ALIGNMENT, 1);
    gl::ReadPixels(0, 0, w, h, gl::RGBA, gl::UNSIGNED_BYTE, pixels.data());

    // 翻转（OpenGL 原点在左下角），并把 alpha 设为不透明
    std::vector<uint8_t> flipped(pixels.size());
    const size_t stride = static_cast<size_t>(w) * 4;
    for (int y = 0; y < h; ++y) {
        const uint8_t* src = pixels.data() + static_cast<size_t>(h - 1 - y) * stride;
        uint8_t* dst = flipped.data() + static_cast<size_t>(y) * stride;
        for (size_t i = 0; i < stride; i += 4) {
            dst[i] = src[i];
            dst[i + 1] = src[i + 1];
            dst[i + 2] = src[i + 2];
            dst[i + 3] = 255;
        }
    }
    return WritePng(path, w, h, flipped);
}

int Application::Run() {
    if (!Init()) return 1;

    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();
    int frame = 0;
    std::string lastTitle;

    while (!glfwWindowShouldClose(m_window) && !m_editor->WantsQuit()) {
        glfwPollEvents();

        // 窗口最小化时不渲染，降低 CPU/GPU 占用
        if (glfwGetWindowAttrib(m_window, GLFW_ICONIFIED)) {
            glfwWaitEventsTimeout(0.1);
            last = Clock::now();
            continue;
        }

        const auto now = Clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;
        // 自动截图模式下使用固定帧时间，保证结果可复现
        if (!m_config.screenshotPath.empty()) dt = 1.0f / 60.0f;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        m_editor->OnFrame(dt);

        ImGui::Render();
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(m_window, &fbw, &fbh);
        gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
        gl::Viewport(0, 0, fbw, fbh);
        gl::ClearColor(0.07f, 0.08f, 0.1f, 1.0f);
        gl::Clear(gl::COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // 截图必须在 SwapBuffers 之前读取后台缓冲区
        std::string shot = m_editor->ConsumeScreenshotRequest();
        ++frame;
        const bool autoShot = !m_config.screenshotPath.empty() && frame == m_config.screenshotFrames;
        if (autoShot) shot = m_config.screenshotPath;
        if (!shot.empty()) {
            if (SaveWindowScreenshot(shot)) {
                Log::Info("Screenshot saved: %s", shot.c_str());
                m_editor->Notify(std::string(Tr("Screenshot saved")) + ": " + shot);
            } else {
                Log::Error("Failed to save screenshot: %s", shot.c_str());
            }
        }

        glfwSwapBuffers(m_window);
        if (autoShot) break;

        const std::string title = m_editor->WindowTitle();
        if (title != lastTitle) {
            glfwSetWindowTitle(m_window, title.c_str());
            lastTitle = title;
        }
    }

    Shutdown();
    return 0;
}

}  // namespace app
