# kinema_scene —— 示例场景 + 场景文件读写（依赖 kinema，不依赖图形库）
add_library(kinema_scene STATIC
    src/SceneLibrary.cpp
    src/SceneSerializer.cpp
)
add_library(kinema::scene ALIAS kinema_scene)

target_include_directories(kinema_scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kinema_scene PUBLIC kinema)
kinema_set_warnings(kinema_scene)
