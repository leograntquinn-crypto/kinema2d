# =============================================================================
# Kinema2D —— C++ 2D 物理引擎 + 可视化桌面客户端
#
#   engine/   物理引擎核心库（kinema）       —— 纯 C++17，无第三方依赖
#   scene/    场景库（kinema_scene）         —— 示例场景 + 场景文件读写
#   app/      桌面客户端（Kinema2D）          —— GLFW + OpenGL 3.3 + Dear ImGui
#   tests/    单元测试（kinema_tests）        —— GoogleTest
#   tools/    命令行工具（kinema_bench）      —— 性能基准测试
# =============================================================================
cmake_minimum_required(VERSION 3.20)

project(Kinema2D
    VERSION 0.1.0
    DESCRIPTION "A 2D rigid body physics engine with a desktop sandbox client"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 所有可执行文件输出到 build/bin，方便查找
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(KINEMA_BUILD_APP        "Build the desktop sandbox client" ON)
option(KINEMA_BUILD_TESTS      "Build unit tests"                 ON)
option(KINEMA_BUILD_TOOLS      "Build command line tools"         ON)
option(KINEMA_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(CompilerWarnings)

add_subdirectory(engine)
add_subdirectory(scene)

if(KINEMA_BUILD_APP)
    include(Dependencies)
    kinema_fetch_app_dependencies()
    add_subdirectory(app)
endif()

if(KINEMA_BUILD_TOOLS)
    add_subdirectory(tools/benchmark)
endif()

if(KINEMA_BUILD_TESTS)
    enable_testing()
    include(Dependencies)
    kinema_fetch_test_dependencies()
    add_subdirectory(tests)
endif()
