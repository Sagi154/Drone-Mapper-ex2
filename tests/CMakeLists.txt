find_package(GTest REQUIRED)

add_executable(drone_mapper_simulation_test
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

target_compile_features(drone_mapper_simulation_test PUBLIC cxx_std_20)
target_compile_options(drone_mapper_simulation_test
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper_simulation_test
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

include(GoogleTest)
gtest_discover_tests(drone_mapper_simulation_test)
