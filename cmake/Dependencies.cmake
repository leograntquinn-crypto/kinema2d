# =============================================================================
# 第三方依赖：全部通过 FetchContent 在配置阶段自动下载，无需手动安装。
#
#   GLFW        3.4             窗口、OpenGL 上下文、键鼠输入
#   Dear ImGui  1.91.9b-docking 即时模式 GUI（docking 分支支持可停靠面板）
#   GoogleTest  1.15.2          单元测试框架
#
# 固定版本号（而不是跟随 master）能保证任何时候构建结果都一致。
# =============================================================================
include_guard(GLOBAL)
include(FetchContent)

set(FETCHCONTENT_QUIET OFF)

function(kinema_fetch_app_dependencies)
    if(TARGET glfw AND TARGET imgui)
        return()
    endif()

    # ---- GLFW ----
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
    # Linux 下仅构建 X11 后端以减少系统依赖（Wayland 桌面可通过 XWayland 运行）
    set(GLFW_BUILD_WAYLAND  OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4
        GIT_SHALLOW    TRUE)

    # ---- Dear ImGui（没有官方 CMakeLists，自己定义一个静态库目标） ----
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        v1.91.9b-docking
        GIT_SHALLOW    TRUE)

    FetchContent_MakeAvailable(glfw imgui)

    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
    target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
    target_link_libraries(imgui PUBLIC glfw)
    # 禁用已废弃的旧 API，强迫我们使用最新的写法
    target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
    if(MSVC)
        target_compile_options(imgui PRIVATE /utf-8)
    endif()
endfunction()

function(kinema_fetch_test_dependencies)
    if(TARGET GTest::gtest_main)
        return()
    endif()

    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # Windows: 与项目使用相同的 CRT
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.15.2
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(googletest)
endfunction()
