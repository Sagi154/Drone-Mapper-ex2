cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

option(DRONE_MAPPER_BUILD_YAML_EXAMPLE "Build the course yaml-cpp example executable" ON)

find_package(mp-units CONFIG REQUIRED)
find_package(TinyNPY CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!
find_package(GTest CONFIG REQUIRED)

add_library(drone_mapper
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MappingAlgorithmFrontier.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/io/RunErrorLog.cpp
    src/io/TimeFormat.cpp
    src/io/SimulationOutputYamlWriter.cpp
    src/io/PathResolver.cpp
    src/io/RunPathHelpers.cpp
    src/io/YamlParseUtil.cpp
    src/io/DroneConfigYamlParser.cpp
    src/io/MissionConfigYamlParser.cpp
    src/io/LidarConfigYamlParser.cpp
    src/io/SimulationConfigYamlParser.cpp
    src/io/CompositionYamlParser.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
if(MSVC)
    target_compile_options(drone_mapper PRIVATE /W4 /WX /utf-8)
else()
    target_compile_options(drone_mapper PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

if(DRONE_MAPPER_BUILD_YAML_EXAMPLE)
    add_executable(example_yml cpp_yaml_example/main.cpp)
    target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)
endif()

enable_testing()
add_subdirectory(tests)
