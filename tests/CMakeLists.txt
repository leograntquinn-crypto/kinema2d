add_executable(kinema_tests
    test_math.cpp
    test_shapes.cpp
    test_collision.cpp
    test_world.cpp
)
target_link_libraries(kinema_tests PRIVATE kinema kinema_scene GTest::gtest_main)
if(MSVC)
    target_compile_options(kinema_tests PRIVATE /utf-8)
endif()

include(GoogleTest)
gtest_discover_tests(kinema_tests DISCOVERY_TIMEOUT 60)
