// test_drone_control.cpp — DroneControl.*
// Scenarios: scan pipeline, movement execution, algorithm completion, and error paths.

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>

#include <TinyNPY.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

namespace drone_mapper {
namespace {

class AlgorithmMock : public IMappingAlgorithm {
public:
    AlgorithmMock()
        : IMappingAlgorithm(defaultMission(), defaultLidar(), defaultDrone(), stand_in_map_) {
        ON_CALL(stand_in_map_, getMapConfig()).WillByDefault(testing::Return(makeMapConfig()));
        ON_CALL(stand_in_map_, isInBounds(testing::_)).WillByDefault(testing::Return(true));
        ON_CALL(stand_in_map_, atVoxel(testing::_))
            .WillByDefault(testing::Return(types::VoxelOccupancy::Unmapped));
    }

    MOCK_METHOD(types::MappingStepCommand,
                nextStep,
                (const types::DroneState& state, const types::LidarScanResult* latest_scan),
                (override));

private:
    class StandInMap final : public IMap3D {
    public:
        MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
        MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
        MOCK_METHOD(bool, isInBounds, (const Position3D&), (const, override));
    };

    static types::MapConfig makeMapConfig() {
        types::MapConfig config{};
        config.resolution = 10.0 * cm;
        return config;
    }

    static types::MissionConfigData defaultMission() {
        return types::MissionConfigData{100, 10.0 * cm, 1.0};
    }

    static types::DroneConfigData defaultDrone() {
        return types::DroneConfigData{
            5.0 * cm,
            90.0 * horizontal_angle[deg],
            20.0 * cm,
            20.0 * cm,
        };
    }

    static types::LidarConfigData defaultLidar() {
        return types::LidarConfigData{20.0 * cm, 120.0 * cm, 2.5 * cm, 3};
    }

    StandInMap stand_in_map_;
};

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

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyHiddenMap() {
    auto map = std::make_shared<NpyArray>(
        NpyArray::shape_t{4, 4, 4},
        sizeof(std::int8_t),
        NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(),
                map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Empty));
    return map;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyOutputMap() {
    auto map = std::make_shared<NpyArray>(
        NpyArray::shape_t{4, 4, 4},
        sizeof(std::int8_t),
        NpyArray::GetTypeChar(typeid(std::int8_t)));
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

struct DroneControlFixture {
    Map3DImpl hidden_map{makeEmptyHiddenMap(), makeUnitCubeConfig()};
    Map3DImpl output_map{makeEmptyOutputMap(), makeUnitCubeConfig()};
    MockGPS gps{Position3D{}, Orientation{}, 10.0 * cm};
    MockMovement movement{gps, hidden_map, defaultDrone()};
    MockLidar lidar{defaultLidar(), hidden_map, gps};
    testing::StrictMock<AlgorithmMock> algorithm;
};

} // namespace

// What: first algorithm call on a fresh mission.
// Expected: latest_scan is nullptr before any scan has been stored.
TEST(DroneControlTest, FirstStepPassesNullScanToAlgorithm) {
    DroneControlFixture fixture;
    DroneControlImpl control{defaultDrone(),
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::IsNull()))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .scan_orientation = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = types::AlgorithmStatus::Working,
        }));

    const types::DroneStepResult result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
}

// What: algorithm requests a scan orientation.
// Expected: output map receives mapped voxels from MockLidar via ScanResultToVoxels.
TEST(DroneControlTest, ExecutesScanAndUpdatesOutputMap) {
    DroneControlFixture fixture;
    DroneControlImpl control{defaultDrone(),
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::IsNull()))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .scan_orientation = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = types::AlgorithmStatus::Working,
        }));

    const types::DroneStepResult result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    EXPECT_NE(fixture.output_map.atVoxel(Position3D{}), types::VoxelOccupancy::Unmapped);
}

// What: algorithm reports Finished.
// Expected: DroneControl returns Completed without executing further commands.
TEST(DroneControlTest, ReturnsCompletedWhenAlgorithmFinishes) {
    DroneControlFixture fixture;
    DroneControlImpl control{defaultDrone(),
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .status = types::AlgorithmStatus::Finished,
        }));

    const types::DroneStepResult result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
}

// What: algorithm returns both movement and scan in one command.
// Expected: movement executes before scan side effects (heading changes first).
TEST(DroneControlTest, ExecutesMovementBeforeScanWhenBothRequested) {
    DroneControlFixture fixture;
    DroneControlImpl control{defaultDrone(),
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .movement = types::MovementCommand{
                .type = types::MovementCommandType::Rotate,
                .rotation = types::RotationDirection::Right,
                .angle = 45.0 * horizontal_angle[deg],
            },
            .scan_orientation = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = types::AlgorithmStatus::Working,
        }));

    const types::DroneStepResult result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    // MockMovement: Rotate Right subtracts from heading (see SimulationRunTest.Movement_Rotate_WithinLimit).
    EXPECT_DOUBLE_EQ(fixture.gps.heading().horizontal.numerical_value_in(deg), -45.0);
}

// What: movement driver rejects a command (e.g. exceeds per-command limit at driver).
// Expected: step returns Error with a message.
TEST(DroneControlTest, ReturnsErrorWhenMovementFails) {
    DroneControlFixture fixture;
    types::DroneConfigData tight_drone{
        5.0 * cm,
        1.0 * horizontal_angle[deg],
        1.0 * cm,
        1.0 * cm,
    };
    DroneControlImpl control{tight_drone,
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .movement = types::MovementCommand{
                .type = types::MovementCommandType::Rotate,
                .rotation = types::RotationDirection::Right,
                .angle = 90.0 * horizontal_angle[deg],
            },
            .status = types::AlgorithmStatus::Working,
        }));

    const types::DroneStepResult result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_FALSE(result.message.empty());
}

// What: movement command exceeds DroneConfigData limits before reaching the driver.
// Expected: step returns Error mentioning limits.
TEST(DroneControlTest, ReturnsErrorWhenMovementExceedsDroneLimits) {
    DroneControlFixture fixture;
    DroneControlImpl control{defaultDrone(),
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .movement = types::MovementCommand{
                .type = types::MovementCommandType::Advance,
                .distance = 500.0 * cm,
            },
            .status = types::AlgorithmStatus::Working,
        }));

    const types::DroneStepResult result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_NE(result.message.find("limits"), std::string::npos);
}

// What: state() is queried before and after a Continue step.
// Expected: GPS pose is reflected and step_index increments.
TEST(DroneControlTest, StateReflectsGpsPoseAndStepIndex) {
    DroneControlFixture fixture;
    fixture.gps.setPosition(Position3D{10.0 * x_extent[cm], 20.0 * y_extent[cm], 0.0 * z_extent[cm]});
    fixture.gps.setHeading(Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});

    DroneControlImpl control{defaultDrone(),
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_EQ(control.state().step_index, 0U);
    EXPECT_DOUBLE_EQ(control.state().position.x.numerical_value_in(cm), 10.0);

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .status = types::AlgorithmStatus::Working,
        }));

    ASSERT_EQ(control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(control.state().step_index, 1U);
}

// What: a scan step stores latest_scan for the next algorithm call.
// Expected: second nextStep receives a non-null latest_scan pointer.
TEST(DroneControlTest, PassesPreviousScanToAlgorithmOnLaterSteps) {
    DroneControlFixture fixture;
    DroneControlImpl control{defaultDrone(),
                             defaultMission(),
                             defaultLidar(),
                             fixture.lidar,
                             fixture.gps,
                             fixture.movement,
                             fixture.output_map,
                             fixture.algorithm};

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::IsNull()))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .scan_orientation = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = types::AlgorithmStatus::Working,
        }));
    ASSERT_EQ(control.step().status, types::DroneStepStatus::Continue);

    EXPECT_CALL(fixture.algorithm, nextStep(testing::_, testing::NotNull()))
        .WillOnce(testing::Return(types::MappingStepCommand{
            .status = types::AlgorithmStatus::Working,
        }));
    EXPECT_EQ(control.step().status, types::DroneStepStatus::Continue);
}

} // namespace drone_mapper
