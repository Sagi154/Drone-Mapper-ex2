add_executable(drone_mapper_simulation_test
    components/test_map3d_impl.cpp
    components/test_yaml_config_parsers.cpp
    components/test_simulation_run.cpp
    components/test_drone_control.cpp
    components/test_mapping_algorithm.cpp
    components/test_mock_lidar.cpp
    components/test_maps_comparison.cpp
)

target_link_libraries(drone_mapper_simulation_test
    PRIVATE
        drone_mapper
        GTest::gtest_main
        GTest::gmock
)

target_compile_definitions(drone_mapper_simulation_test
    PRIVATE
        DRONE_MAPPER_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_compile_features(drone_mapper_simulation_test PUBLIC cxx_std_20)
if(MSVC)
    target_compile_options(drone_mapper_simulation_test PRIVATE /W4 /WX /utf-8)
else()
    target_compile_options(drone_mapper_simulation_test PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
target_include_directories(drone_mapper_simulation_test
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

include(GoogleTest)
gtest_discover_tests(drone_mapper_simulation_test)
