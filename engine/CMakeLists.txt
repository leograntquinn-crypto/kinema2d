# =============================================================================
# kinema —— 物理引擎核心库
# 只依赖 C++ 标准库，不依赖任何图形/窗口库，可以单独被测试和复用。
# =============================================================================
add_library(kinema STATIC
    src/collision/Shape.cpp
    src/collision/Collision.cpp
    src/collision/BroadPhase.cpp
    src/dynamics/Body.cpp
    src/dynamics/Contact.cpp
    src/dynamics/World.cpp
    src/joints/Joint.cpp
)
add_library(kinema::kinema ALIAS kinema)

target_include_directories(kinema PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(kinema PUBLIC cxx_std_17)
kinema_set_warnings(kinema)
