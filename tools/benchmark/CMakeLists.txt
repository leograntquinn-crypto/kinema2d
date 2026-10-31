add_executable(kinema_bench main.cpp)
target_link_libraries(kinema_bench PRIVATE kinema kinema_scene)
kinema_set_warnings(kinema_bench)
