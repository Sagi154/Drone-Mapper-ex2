include(GoogleTest)

function(add_drone_mapper_test_suite target_name label)
    add_executable(${target_name} ${ARGN})
    target_link_libraries(${target_name}
        PRIVATE
            drone_mapper
            GTest::gtest
            GTest::gmock
    )
    target_sources(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp)
    target_compile_definitions(${target_name}
        PRIVATE
            DRONE_MAPPER_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    target_compile_features(${target_name} PUBLIC cxx_std_20)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /WX /utf-8)
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Werror -pedantic)
    endif()
    target_include_directories(${target_name}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/include
    )
    gtest_discover_tests(${target_name} PROPERTIES LABELS "${label}")
endfunction()

set(DRONE_MAPPER_COMPONENT_TEST_SOURCES
    components/test_yaml_config_parsers.cpp
    components/test_run_error_log.cpp
    components/test_simulation_cli.cpp
    components/test_map3d_impl.cpp
    components/test_simulation_run.cpp
    components/test_simulation_run_factory.cpp
    components/test_simulation_manager.cpp
    components/test_mapping_algorithm.cpp
    components/test_mapping_algorithm_frontier.cpp
    components/test_drone_control.cpp
    components/test_mission_control.cpp
    components/test_maps_comparison.cpp
    components/test_mock_lidar.cpp
    integration/test_integration_full_flow.cpp
)

# Implemented component suites only — add stub files here once they contain real tests.
add_drone_mapper_test_suite(drone_mapper_config_tests config
    components/test_yaml_config_parsers.cpp
    components/test_run_error_log.cpp
    components/test_simulation_cli.cpp
)

add_drone_mapper_test_suite(drone_mapper_map_tests map
    components/test_map3d_impl.cpp
)

add_drone_mapper_test_suite(drone_mapper_simulation_tests simulation
    components/test_simulation_run.cpp
    components/test_simulation_run_factory.cpp
    components/test_simulation_manager.cpp
)

target_include_directories(drone_mapper_simulation_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/tests/support
)

add_drone_mapper_test_suite(drone_mapper_algorithm_tests algorithm
    components/test_mapping_algorithm.cpp
    components/test_mapping_algorithm_frontier.cpp
)

target_include_directories(drone_mapper_algorithm_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

add_drone_mapper_test_suite(drone_mapper_drone_tests drone
    components/test_drone_control.cpp
)

add_drone_mapper_test_suite(drone_mapper_mission_tests mission
    components/test_mission_control.cpp
)

add_drone_mapper_test_suite(drone_mapper_maps_comparison_tests maps-comparison
    components/test_maps_comparison.cpp
)

target_compile_definitions(drone_mapper_maps_comparison_tests
    PRIVATE
        MAPS_COMPARISON_EXE="$<TARGET_FILE:maps_comparison>"
)
add_dependencies(drone_mapper_maps_comparison_tests maps_comparison)

add_drone_mapper_test_suite(drone_mapper_lidar_tests lidar
    components/test_mock_lidar.cpp
)

add_executable(drone_mapper_simulation_test ${DRONE_MAPPER_COMPONENT_TEST_SOURCES})
target_link_libraries(drone_mapper_simulation_test
    PRIVATE
        drone_mapper
        GTest::gtest
        GTest::gmock
)
target_sources(drone_mapper_simulation_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp)
target_compile_definitions(drone_mapper_simulation_test
    PRIVATE
        DRONE_MAPPER_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        MAPS_COMPARISON_EXE="$<TARGET_FILE:maps_comparison>"
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
        ${CMAKE_SOURCE_DIR}/tests/support
        ${CMAKE_SOURCE_DIR}/src
)
add_dependencies(drone_mapper_simulation_test maps_comparison)
gtest_discover_tests(drone_mapper_simulation_test)

add_drone_mapper_test_suite(drone_mapper_integration_tests integration
    integration/test_integration_full_flow.cpp
)

target_include_directories(drone_mapper_integration_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/tests/support
)
