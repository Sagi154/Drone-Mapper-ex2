// test_mapping_algorithm.cpp — MappingAlgorithm.*
// Scenarios: first-step scan, post-scan planning, movement commands, finished states.

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] types::MapConfig makeUnitCubeConfig() {
    types::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 40.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 40.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 40.0 * z_extent[cm];
    return config;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeMutableMap(const NpyArray::shape_t& shape) {
    auto map = std::make_shared<NpyArray>(shape, sizeof(std::int8_t), NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(),
                map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] types::MissionConfigData defaultMission() {
    return types::MissionConfigData{100, 10.0 * cm, 1.0};
}

[[nodiscard]] types::DroneConfigData defaultDrone() {
    return types::DroneConfigData{
        5.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
}

[[nodiscard]] types::LidarConfigData defaultLidar() {
    return types::LidarConfigData{20.0 * cm, 120.0 * cm, 2.5 * cm, 3};
}

void stampEmptyCorridor(Map3DImpl& map) {
    for (int x = 0; x <= 2; ++x) {
        for (int y = 0; y <= 2; ++y) {
            for (int z = 0; z <= 2; ++z) {
                map.set(Position3D{
                              (10.0 * static_cast<double>(x)) * x_extent[cm],
                              (10.0 * static_cast<double>(y)) * y_extent[cm],
                              (10.0 * static_cast<double>(z)) * z_extent[cm]},
                        types::VoxelOccupancy::Empty);
            }
        }
    }
}

[[nodiscard]] types::DroneState defaultState() {
    return types::DroneState{
        Position3D{},
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        0,
    };
}

[[nodiscard]] types::LidarScanResult dummyScan() {
    return types::LidarScanResult{types::LidarHit{50.0 * cm, Orientation{}}};
}

} // namespace

TEST(MappingAlgorithmTest, FirstStepRequestsScanWhenNoPriorScanExists) {
    Map3DImpl output_map{makeMutableMap({3, 3, 3}), makeUnitCubeConfig()};
    MappingAlgorithmImpl algorithm{defaultMission(), defaultLidar(), defaultDrone(), output_map};

    const types::MappingStepCommand command = algorithm.nextStep(defaultState(), nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
}

TEST(MappingAlgorithmTest, PlansMovementAfterScanWhenFrontierExists) {
    Map3DImpl output_map{makeMutableMap({3, 3, 3}), makeUnitCubeConfig()};
    stampEmptyCorridor(output_map);
    MappingAlgorithmImpl algorithm{defaultMission(), defaultLidar(), defaultDrone(), output_map};

    const types::MappingStepCommand scan = algorithm.nextStep(defaultState(), nullptr);
    ASSERT_TRUE(scan.scan_orientation.has_value());

    const types::MappingStepCommand move = algorithm.nextStep(defaultState(), &dummyScan());

    EXPECT_EQ(move.status, types::AlgorithmStatus::Working);
    ASSERT_TRUE(move.movement.has_value());
    EXPECT_NE(move.movement->type, types::MovementCommandType::Hover);
}

TEST(MappingAlgorithmTest, FinishesWhenMapFullyKnownAndNoFrontier) {
    Map3DImpl output_map{makeMutableMap({1, 1, 1}), makeUnitCubeConfig()};
    output_map.set(Position3D{}, types::VoxelOccupancy::Empty);
    MappingAlgorithmImpl algorithm{defaultMission(), defaultLidar(), defaultDrone(), output_map};

    ASSERT_TRUE(algorithm.nextStep(defaultState(), nullptr).scan_orientation.has_value());
    const types::MappingStepCommand finished = algorithm.nextStep(defaultState(), &dummyScan());

    EXPECT_EQ(finished.status, types::AlgorithmStatus::Finished);
    EXPECT_FALSE(finished.movement.has_value());
}

TEST(MappingAlgorithmTest, ReportsUnmappableWhenUnknownCellsRemainWithoutFrontier) {
    Map3DImpl output_map{makeMutableMap({2, 2, 2}), makeUnitCubeConfig()};
    output_map.set(Position3D{}, types::VoxelOccupancy::Empty);
    MappingAlgorithmImpl algorithm{defaultMission(), defaultLidar(), defaultDrone(), output_map};

    ASSERT_TRUE(algorithm.nextStep(defaultState(), nullptr).scan_orientation.has_value());
    const types::MappingStepCommand finished = algorithm.nextStep(defaultState(), &dummyScan());

    EXPECT_EQ(finished.status, types::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithmTest, MovementCommandRotatesBeforeAdvancingAlongXAxis) {
    Map3DImpl output_map{makeMutableMap({3, 3, 3}), makeUnitCubeConfig()};
    stampEmptyCorridor(output_map);
    MappingAlgorithmImpl algorithm{defaultMission(), defaultLidar(), defaultDrone(), output_map};

    ASSERT_TRUE(algorithm.nextStep(defaultState(), nullptr).scan_orientation.has_value());
    const types::MappingStepCommand move = algorithm.nextStep(defaultState(), &dummyScan());

    ASSERT_TRUE(move.movement.has_value());
    EXPECT_TRUE(move.movement->type == types::MovementCommandType::Rotate ||
                move.movement->type == types::MovementCommandType::Advance ||
                move.movement->type == types::MovementCommandType::Elevate);
}

TEST(MappingAlgorithmTest, RemainsFinishedAfterCompletion) {
    Map3DImpl output_map{makeMutableMap({1, 1, 1}), makeUnitCubeConfig()};
    output_map.set(Position3D{}, types::VoxelOccupancy::Empty);
    MappingAlgorithmImpl algorithm{defaultMission(), defaultLidar(), defaultDrone(), output_map};

    ASSERT_TRUE(algorithm.nextStep(defaultState(), nullptr).scan_orientation.has_value());
    ASSERT_EQ(algorithm.nextStep(defaultState(), &dummyScan()).status, types::AlgorithmStatus::Finished);

    const types::MappingStepCommand again = algorithm.nextStep(defaultState(), &dummyScan());
    EXPECT_EQ(again.status, types::AlgorithmStatus::Finished);
}

} // namespace drone_mapper
