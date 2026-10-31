# =============================================================================
# Kinema2D —— 桌面客户端
# =============================================================================
add_executable(Kinema2D
    src/main.cpp
    src/core/Application.cpp
    src/render/GL.cpp
    src/render/Shader.cpp
    src/render/Framebuffer.cpp
    src/render/Renderer2D.cpp
    src/editor/Editor.cpp
    src/editor/Simulation.cpp
    src/editor/DebugDraw.cpp
    src/editor/Theme.cpp
    src/editor/panels/ViewportPanel.cpp
    src/editor/panels/ScenesPanel.cpp
    src/editor/panels/OutlinerPanel.cpp
    src/editor/panels/InspectorPanel.cpp
    src/editor/panels/SettingsPanel.cpp
    src/editor/panels/StatsPanel.cpp
    src/editor/panels/ConsolePanel.cpp
    src/util/I18n.cpp
    src/util/Log.cpp
    src/util/ImageWriter.cpp
)

target_include_directories(Kinema2D PRIVATE src)
target_link_libraries(Kinema2D PRIVATE kinema kinema_scene imgui glfw)
kinema_set_warnings(Kinema2D)

# 说明：Windows 上保留控制台窗口，方便学习时查看日志输出。
# 如需隐藏，可设置 WIN32_EXECUTABLE 属性并使用 /ENTRY:mainCRTStartup。

if(APPLE)
    target_compile_definitions(Kinema2D PRIVATE GL_SILENCE_DEPRECATION)
endif()
