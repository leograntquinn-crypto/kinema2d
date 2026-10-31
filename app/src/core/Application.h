// =============================================================================
// Application：窗口 + OpenGL 上下文 + ImGui 生命周期 + 主循环
//
//   main()
//     └── Application::Run()
//           ├── Init()        创建窗口、加载 OpenGL、初始化 ImGui 和 Editor
//           ├── while (!关闭)
//           │     ├── glfwPollEvents()          处理系统事件
//           │     ├── ImGui::NewFrame()
//           │     ├── Editor::OnFrame(dt)       物理更新 + 构建界面 + 渲染场景
//           │     ├── ImGui::Render()           绘制界面
//           │     └── glfwSwapBuffers()         显示
//           └── Shutdown()
// =============================================================================
#pragma once

#include <memory>
#include <string>

struct GLFWwindow;

namespace app {

class Editor;

struct AppConfig {
    int width = 1600;
    int height = 920;
    std::string scene = "pyramid";
    std::string language;        // "en" / "zh"，空 = 默认
    std::string screenshotPath;  // 非空：运行若干帧后截图并退出（用于自动化/文档）
    int screenshotFrames = 120;
    bool startPaused = false;
    bool vsync = true;
    bool lightTheme = false;
};

class Application {
public:
    explicit Application(AppConfig config);
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    int Run();

private:
    bool Init();
    void Shutdown();
    bool SaveWindowScreenshot(const std::string& path) const;

    AppConfig m_config;
    GLFWwindow* m_window = nullptr;
    std::unique_ptr<Editor> m_editor;
    bool m_imguiInitialized = false;
};

}  // namespace app
