// test_simulation_run.cpp — SimulationRun.* (MockGPS and MockMovement)

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

#include <TinyNPY.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

// GMock stand-in for IMap3D in movement tests.
class MapMock : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel,     (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (),                  (const, override));
    MOCK_METHOD(bool,                  isInBounds,   (const Position3D&), (const, override));
};

using NiceMapMock = testing::NiceMock<MapMock>;

class LidarMock : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation scan_orientation), (const, override));
};

class DroneControlMock : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

class MissionControlMock : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

class MappingAlgorithmStub : public IMappingAlgorithm {
public:
    MappingAlgorithmStub(const types::MissionConfigData& mission,
                         const types::LidarConfigData& lidar,
                         const types::DroneConfigData& drone,
                         const IMap3D& output_map)
        : IMappingAlgorithm(mission, lidar, drone, output_map) {}

    types::MappingStepCommand nextStep(const types::DroneState&,
                                       const types::LidarScanResult*) override {
        return {};
    }
};

using NiceLidarMock = testing::NiceMock<LidarMock>;
using NiceDroneControlMock = testing::NiceMock<DroneControlMock>;

[[nodiscard]] types::LidarConfigData makeDefaultLidarConfig() {
    return types::LidarConfigData{
        0.0 * cm,
        100.0 * cm,
        5.0 * cm,
        4,
    };
}

[[nodiscard]] types::SimulationConfigData makeDefaultSimulationConfig() {
    return types::SimulationConfigData{
        .map_filename = "data_maps/test.npy",
        .map_resolution = 10.0 * cm,
        .map_offset = Position3D{},
        .initial_drone_position = Position3D{},
        .initial_angle = 0.0 * horizontal_angle[deg],
    };
}

[[nodiscard]] types::MissionConfigData makeDefaultMissionConfig() {
    return types::MissionConfigData{
        .max_steps = 100,
        .gps_resolution = 10.0 * cm,
        .output_mapping_resolution_factor = 1.0,
    };
}

types::DroneConfigData makeDefaultDroneConfig() {
    return {
        .radius       = 5.0 * isq::length[cm],
        .max_rotate   = 360.0 * horizontal_angle[deg],
        .max_advance  = 500.0 * isq::length[cm],
        .max_elevate  = 500.0 * isq::length[cm],
    };
}

types::MapConfig makeMapConfig() {
    return types::MapConfig{
        .boundaries = types::MappingBounds{
            .min_x      = 0.0  * x_extent[cm],
            .max_x      = 200.0 * x_extent[cm],
            .min_y      = 0.0  * y_extent[cm],
            .max_y      = 200.0 * y_extent[cm],
            .min_height = 0.0  * z_extent[cm],
            .max_height = 200.0 * z_extent[cm],
        },
        .offset     = Position3D{},
        .resolution = 1.0 * isq::length[cm],
    };
}

void setClearMap(NiceMapMock& map) {
    const types::MapConfig cfg = makeMapConfig();
    ON_CALL(map, getMapConfig()).WillByDefault(testing::Return(cfg));
    ON_CALL(map, isInBounds(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(map, atVoxel(testing::_))
        .WillByDefault(testing::Return(types::VoxelOccupancy::Empty));
}

[[nodiscard]] std::vector<std::string> readAllLines(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    return lines;
}

[[nodiscard]] std::unique_ptr<SimulationRunImpl> makeSimulationRun(
    std::unique_ptr<IMissionControl> mission_control,
    const std::filesystem::path& output_map_file,
    types::SimulationConfigData simulation_config,
    types::MissionConfigData mission_config,
    std::vector<types::ErrorRef> startup_errors = {}) {
    const types::MapConfig map_cfg = makeMapConfig();
    auto hidden_array = std::make_shared<NpyArray>();
    auto output_array = std::make_shared<NpyArray>();
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, map_cfg);
    auto output_map = std::make_unique<Map3DImpl>(output_array, map_cfg);

    auto gps = std::make_unique<MockGPS>(
        simulation_config.initial_drone_position,
        Orientation{simulation_config.initial_angle, 0.0 * altitude_angle[deg]},
        mission_config.gps_resolution);
    auto movement =
        std::make_unique<MockMovement>(*gps, *hidden_map, makeDefaultDroneConfig());
    auto lidar = std::make_unique<NiceLidarMock>();
    auto mapping_algorithm = std::make_unique<MappingAlgorithmStub>(
        mission_config, makeDefaultLidarConfig(), makeDefaultDroneConfig(), *output_map);
    auto drone_control = std::make_unique<NiceDroneControlMock>();

    return std::make_unique<SimulationRunImpl>(
        std::unique_ptr<const IMap3D>(std::move(hidden_map)),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        std::move(simulation_config),
        std::move(mission_config),
        output_map_file,
        std::move(startup_errors));
}

TEST(SimulationRunTest, GPS_ReturnsInitialPosition) {
    const Position3D init{10.0 * x_extent[cm], 20.0 * y_extent[cm], 30.0 * z_extent[cm]};
    const Orientation heading{45.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    MockGPS gps{init, heading, 1.0 * isq::length[cm]};

    const Position3D got = gps.position();
    EXPECT_DOUBLE_EQ(got.x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(got.y.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(got.z.numerical_value_in(cm), 30.0);
}

TEST(SimulationRunTest, GPS_ReturnsInitialHeading) {
    const Position3D init{};
    const Orientation heading{90.0 * horizontal_angle[deg], 15.0 * altitude_angle[deg]};
    MockGPS gps{init, heading, 1.0 * isq::length[cm]};

    const Orientation got = gps.heading();
    EXPECT_DOUBLE_EQ(got.horizontal.numerical_value_in(deg), 90.0);
    EXPECT_DOUBLE_EQ(got.altitude.numerical_value_in(deg), 15.0);
}

TEST(SimulationRunTest, GPS_SetPosition_ExactMultiple_NoChange) {
    const Position3D init{};
    MockGPS gps{init, Orientation{}, 10.0 * isq::length[cm]};

    gps.setPosition(Position3D{
        20.0 * x_extent[cm],
        40.0 * y_extent[cm],
        60.0 * z_extent[cm],
    });
    const Position3D got = gps.position();
    EXPECT_DOUBLE_EQ(got.x.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(got.y.numerical_value_in(cm), 40.0);
    EXPECT_DOUBLE_EQ(got.z.numerical_value_in(cm), 60.0);
}

TEST(SimulationRunTest, GPS_SetPosition_SnapsToResolutionGrid) {
    const Position3D init{};
    MockGPS gps{init, Orientation{}, 10.0 * isq::length[cm]};

    gps.setPosition(Position3D{
        14.0 * x_extent[cm],   // nearest grid: 10
        16.0 * y_extent[cm],   // nearest grid: 20
        25.0 * z_extent[cm],   // nearest grid: 30
    });
    const Position3D got = gps.position();
    EXPECT_DOUBLE_EQ(got.x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(got.y.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(got.z.numerical_value_in(cm), 30.0);
}

TEST(SimulationRunTest, GPS_SetPosition_ZeroResolution_NoSnapping) {
    MockGPS gps{Position3D{}, Orientation{}, 0.0 * isq::length[cm]};
    gps.setPosition(Position3D{7.3 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 7.3);
}

TEST(SimulationRunTest, GPS_SetHeading_UpdatesHeading) {
    MockGPS gps{Position3D{}, Orientation{}, 1.0 * isq::length[cm]};
    const Orientation newH{270.0 * horizontal_angle[deg], -5.0 * altitude_angle[deg]};
    gps.setHeading(newH);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 270.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.numerical_value_in(deg), -5.0);
}

TEST(SimulationRunTest, GPS_SetPosition_NegativeCoordinates_SnapCorrectly) {
    MockGPS gps{Position3D{}, Orientation{}, 10.0 * isq::length[cm]};

    gps.setPosition(Position3D{
        -14.0 * x_extent[cm],  // nearest grid: -10
        -16.0 * y_extent[cm],  // nearest grid: -20
        -25.0 * z_extent[cm],  // nearest grid: -30
    });
    const Position3D got = gps.position();
    EXPECT_DOUBLE_EQ(got.x.numerical_value_in(cm), -10.0);
    EXPECT_DOUBLE_EQ(got.y.numerical_value_in(cm), -20.0);
    EXPECT_DOUBLE_EQ(got.z.numerical_value_in(cm), -30.0);
}

TEST(SimulationRunTest, GPS_SetPosition_HalfwayPoint_RoundsAwayFromZero) {
    MockGPS gps{Position3D{}, Orientation{}, 10.0 * isq::length[cm]};

    gps.setPosition(Position3D{
        15.0  * x_extent[cm],   // half-way → 20
        -15.0 * y_extent[cm],   // half-way → -20
        5.0   * z_extent[cm],   // already on grid → 10? No: 5/10=0.5 → round=1 → 10
    });
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm),  20.0);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), -20.0);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm),  10.0);
}

TEST(SimulationRunTest, GPS_SetPosition_FineResolution_SnapsCorrectly) {
    MockGPS gps{Position3D{}, Orientation{}, 0.5 * isq::length[cm]};

    gps.setPosition(Position3D{
        7.3 * x_extent[cm],   // 7.3 / 0.5 = 14.6 → round = 15 → 7.5
        0.2 * y_extent[cm],   // 0.2 / 0.5 = 0.4  → round = 0  → 0.0
        1.8 * z_extent[cm],   // 1.8 / 0.5 = 3.6  → round = 4  → 2.0
    });
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 7.5);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 2.0);
}

TEST(SimulationRunTest, GPS_SetPosition_SuccessiveCalls_EachSnapsIndependently) {
    MockGPS gps{Position3D{}, Orientation{}, 10.0 * isq::length[cm]};

    gps.setPosition(Position3D{13.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 10.0);

    // Second call with a fresh raw value — should snap to 20, not snap from 10.
    gps.setPosition(Position3D{17.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 20.0);
}

TEST(SimulationRunTest, GPS_Constructor_InitialPositionNotSnapped) {
    // 17.3 cm is off-grid for a 10 cm resolution, yet the constructor stores it raw.
    MockGPS gps{
        Position3D{17.3 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
        Orientation{},
        10.0 * isq::length[cm]
    };
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 17.3);
}

TEST(SimulationRunTest, Movement_Rotate_WithinLimit_UpdatesHeading) {
    MockGPS gps{Position3D{}, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.rotate(types::RotationDirection::Right, 45.0 * horizontal_angle[deg]);

    EXPECT_TRUE(result.success);
    // Rotate Right by 45° → heading becomes -45° (Right = -delta in the impl)
    // The heading should have changed.
    const double h = gps.heading().horizontal.numerical_value_in(deg);
    EXPECT_DOUBLE_EQ(std::abs(h), 45.0);
}

TEST(SimulationRunTest, Movement_Rotate_Left_DecreasesHeading) {
    MockGPS gps{Position3D{}, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.rotate(types::RotationDirection::Left, 30.0 * horizontal_angle[deg]);

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 30.0);
}

TEST(SimulationRunTest, Movement_Rotate_ExceedsLimit_Rejected) {
    MockGPS gps{Position3D{}, Orientation{10.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg      = makeDefaultDroneConfig();
    cfg.max_rotate = 45.0 * horizontal_angle[deg];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.rotate(types::RotationDirection::Right, 90.0 * horizontal_angle[deg]);

    EXPECT_FALSE(result.success);
    // Heading must remain at 10°
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 10.0);
}

TEST(SimulationRunTest, Movement_Advance_ClearSpace_MovesPosition) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.advance(10.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    // Heading 0° → advance along +X
    const double new_x = gps.position().x.numerical_value_in(cm);
    EXPECT_NEAR(new_x, 110.0, 0.1);
    // Y and Z should not change significantly
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 100.0, 0.1);
    EXPECT_NEAR(gps.position().z.numerical_value_in(cm), 50.0, 0.1);
}

TEST(SimulationRunTest, Movement_Advance_Heading90_MovesY) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.advance(20.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 100.0, 0.5);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 120.0, 0.5);
}

TEST(SimulationRunTest, Movement_Advance_ExceedsLimit_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg        = makeDefaultDroneConfig();
    cfg.max_advance  = 5.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.advance(50.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Movement_Advance_CollisionBlocked_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};

    NiceMapMock map;
    const types::MapConfig cfg = makeMapConfig();
    ON_CALL(map, getMapConfig()).WillByDefault(testing::Return(cfg));
    // Always in-bounds but always occupied → every advance is blocked.
    ON_CALL(map, isInBounds(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(map, atVoxel(testing::_))
        .WillByDefault(testing::Return(types::VoxelOccupancy::Occupied));

    MockMovement movement{gps, map, makeDefaultDroneConfig()};
    const auto result = movement.advance(10.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Movement_Advance_OutOfBoundsBlocked_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};

    NiceMapMock map;
    const types::MapConfig cfg = makeMapConfig();
    ON_CALL(map, getMapConfig()).WillByDefault(testing::Return(cfg));
    // Always out-of-bounds → every advance is blocked.
    ON_CALL(map, isInBounds(testing::_)).WillByDefault(testing::Return(false));

    MockMovement movement{gps, map, makeDefaultDroneConfig()};
    const auto result = movement.advance(10.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Movement_Elevate_ClearSpace_MovesZ) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{}, 1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.elevate(15.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().z.numerical_value_in(cm), 65.0, 0.1);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 100.0, 0.1);
}

TEST(SimulationRunTest, Movement_Elevate_Negative_MovesDown) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{}, 1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.elevate(-20.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().z.numerical_value_in(cm), 30.0, 0.1);
}

TEST(SimulationRunTest, Movement_Elevate_ExceedsLimit_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{}, 1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg        = makeDefaultDroneConfig();
    cfg.max_elevate  = 10.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.elevate(100.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 50.0);
}

TEST(SimulationRunTest, Movement_Advance_ZeroDistance_SucceedsPositionUnchanged) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.advance(0.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Movement_Advance_ExactlyAtMaxLimit_Accepted) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg       = makeDefaultDroneConfig();
    cfg.max_advance = 50.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.advance(50.0 * isq::length[cm]); // == limit

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 150.0, 0.5);
}

TEST(SimulationRunTest, Movement_Advance_JustOverMaxLimit_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg       = makeDefaultDroneConfig();
    cfg.max_advance = 50.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.advance(50.001 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Movement_Elevate_ZeroDistance_SucceedsZUnchanged) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{}, 1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.elevate(0.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 50.0);
}

TEST(SimulationRunTest, Movement_Elevate_ExactlyAtMaxLimit_Accepted) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{}, 1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg       = makeDefaultDroneConfig();
    cfg.max_elevate = 30.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.elevate(30.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().z.numerical_value_in(cm), 80.0, 0.5);
}

TEST(SimulationRunTest, Movement_Rotate_ZeroAngle_SucceedsHeadingUnchanged) {
    MockGPS gps{Position3D{}, Orientation{45.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.rotate(types::RotationDirection::Left, 0.0 * horizontal_angle[deg]);

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 45.0);
}

TEST(SimulationRunTest, Movement_Rotate_ExactlyAtMaxLimit_Accepted) {
    MockGPS gps{Position3D{}, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg       = makeDefaultDroneConfig();
    cfg.max_rotate  = 90.0 * horizontal_angle[deg];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.rotate(types::RotationDirection::Left, 90.0 * horizontal_angle[deg]);

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 90.0);
}

TEST(SimulationRunTest, Movement_Advance_NegativeDistance_MovesBackward) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.advance(-10.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 90.0, 0.1);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 100.0, 0.1);
    EXPECT_NEAR(gps.position().z.numerical_value_in(cm), 50.0, 0.1);
}

TEST(SimulationRunTest, Movement_Advance_DoesNotChangeHeading) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{135.0 * horizontal_angle[deg], 10.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    movement.advance(15.0 * isq::length[cm]);

    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 135.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.numerical_value_in(deg),   10.0);
}

TEST(SimulationRunTest, Movement_Elevate_DoesNotChangeHeading) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{270.0 * horizontal_angle[deg], -5.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    movement.elevate(10.0 * isq::length[cm]);

    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 270.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.numerical_value_in(deg),   -5.0);
}

TEST(SimulationRunTest, Movement_Rotate_AccumulatesAcrossMultipleCalls) {
    MockGPS gps{Position3D{}, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    movement.rotate(types::RotationDirection::Right, 30.0 * horizontal_angle[deg]);
    movement.rotate(types::RotationDirection::Right, 30.0 * horizontal_angle[deg]);

    // Each Right rotation subtracts from heading: 0 − 30 − 30 = −60
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), -60.0);
}

TEST(SimulationRunTest, Movement_Rotate_LeftThenRightCancels) {
    MockGPS gps{Position3D{}, Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    movement.rotate(types::RotationDirection::Left,  45.0 * horizontal_angle[deg]);
    movement.rotate(types::RotationDirection::Right, 45.0 * horizontal_angle[deg]);

    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 90.0);
}

TEST(SimulationRunTest, Movement_Rotate_HeadingExceeds360_NotWrapped) {
    MockGPS gps{Position3D{}, Orientation{360.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    movement.rotate(types::RotationDirection::Left, 10.0 * horizontal_angle[deg]);

    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 370.0);
}

TEST(SimulationRunTest, Movement_Elevate_CollisionBlocked_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{}, 1.0 * isq::length[cm]};
    NiceMapMock map;
    const types::MapConfig cfg = makeMapConfig();
    ON_CALL(map, getMapConfig()).WillByDefault(testing::Return(cfg));
    ON_CALL(map, isInBounds(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(map, atVoxel(testing::_))
        .WillByDefault(testing::Return(types::VoxelOccupancy::Occupied));
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    const auto result = movement.elevate(10.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 50.0);
}

TEST(SimulationRunTest, Movement_ZeroRadiusDrone_ClearCenter_Succeeds) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg    = makeDefaultDroneConfig();
    cfg.radius   = 0.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    const auto result = movement.advance(10.0 * isq::length[cm]);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 110.0, 0.1);
}

TEST(SimulationRunTest, Movement_ZeroRadiusDrone_OccupiedCenter_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    const types::MapConfig cfg = makeMapConfig();
    ON_CALL(map, getMapConfig()).WillByDefault(testing::Return(cfg));
    ON_CALL(map, isInBounds(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(map, atVoxel(testing::_))
        .WillByDefault(testing::Return(types::VoxelOccupancy::Occupied));
    auto dcfg  = makeDefaultDroneConfig();
    dcfg.radius = 0.0 * isq::length[cm];
    MockMovement movement{gps, map, dcfg};

    const auto result = movement.advance(10.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Movement_Advance_SphericalEdgeOutOfBounds_Rejected) {
    // Drone at (100, 100, 50), heading 0°, advance 10 → new centre (110, 100, 50).
    // Radius = 5 cm.  We make any position with x > 112 report OOB — that captures
    // the sphere cells at (+5 cm from 110 = 115 cm) which lie outside the boundary.
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};

    NiceMapMock map;
    const types::MapConfig cfg = makeMapConfig();
    ON_CALL(map, getMapConfig()).WillByDefault(testing::Return(cfg));
    // Positions with x > 112 are out-of-bounds (simulates wall 3 cm past new centre).
    ON_CALL(map, isInBounds(testing::_)).WillByDefault(
        testing::Invoke([](const Position3D& p) {
            return p.x.numerical_value_in(cm) <= 112.0;
        }));
    ON_CALL(map, atVoxel(testing::_))
        .WillByDefault(testing::Return(types::VoxelOccupancy::Empty));

    MockMovement movement{gps, map, makeDefaultDroneConfig()};
    const auto result = movement.advance(10.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Movement_Elevate_NegativeExceedsLimit_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{}, 1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg       = makeDefaultDroneConfig();
    cfg.max_elevate = 10.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    // −50 cm has magnitude 50, which exceeds the 10-cm limit.
    const auto result = movement.elevate(-50.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 50.0);
}

TEST(SimulationRunTest, Movement_Advance_NegativeExceedsLimit_Rejected) {
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps{start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    auto cfg       = makeDefaultDroneConfig();
    cfg.max_advance = 5.0 * isq::length[cm];
    MockMovement movement{gps, map, cfg};

    // −30 cm, magnitude 30 > limit 5.
    const auto result = movement.advance(-30.0 * isq::length[cm]);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 100.0);
}

TEST(SimulationRunTest, Run_MissionCompleted_ReturnsComparisonScore) {
    auto mission_control = std::make_unique<MissionControlMock>();
    EXPECT_CALL(*mission_control, runMission())
        .WillOnce(testing::Return(types::MissionRunResult{
            types::MissionRunStatus::Completed,
            5,
            {},
        }));

    const std::filesystem::path output_map_file = "output_results/run_0001/output_map.npy";
    auto run = makeSimulationRun(
        std::move(mission_control),
        output_map_file,
        makeDefaultSimulationConfig(),
        makeDefaultMissionConfig());

    const types::SimulationResult result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results.front().steps, 5U);
}

TEST(SimulationRunTest, Run_MissionError_ReturnsScoreMinusOne) {
    auto mission_control = std::make_unique<MissionControlMock>();
    EXPECT_CALL(*mission_control, runMission())
        .WillOnce(testing::Return(types::MissionRunResult{
            types::MissionRunStatus::Error,
            0,
            {types::ErrorRef{"DRONE_HITS_OBSTACLE", "collision"}},
        }));

    auto run = makeSimulationRun(
        std::move(mission_control),
        "output_results/run_0002/output_map.npy",
        makeDefaultSimulationConfig(),
        makeDefaultMissionConfig());

    const types::SimulationResult result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Error);
    ASSERT_EQ(result.mission_results.front().errors.size(), 1U);
    EXPECT_EQ(result.mission_results.front().errors.front().code, "DRONE_HITS_OBSTACLE");
}

TEST(SimulationRunTest, Run_MissionErrors_WrittenToErrorLog) {
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "simulation_run_mission_error_log";
    std::error_code ec;
    std::filesystem::remove_all(output_path, ec);
    std::filesystem::create_directories(output_path / "output_results" / "run_0001");

    const std::filesystem::path output_map_file =
        output_path / "output_results" / "run_0001" / "output_map.npy";
    const std::filesystem::path error_log_file =
        output_path / "output_results" / "run_0001" / "error.log";

    auto mission_control = std::make_unique<MissionControlMock>();
    EXPECT_CALL(*mission_control, runMission())
        .WillOnce(testing::Return(types::MissionRunResult{
            types::MissionRunStatus::Error,
            3,
            {types::ErrorRef{"DRONE_HITS_OBSTACLE", "collision at step 3"}},
        }));

    auto run = makeSimulationRun(
        std::move(mission_control),
        output_map_file,
        makeDefaultSimulationConfig(),
        makeDefaultMissionConfig());

    const types::SimulationResult result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);

    const std::vector<std::string> lines = readAllLines(error_log_file);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_NE(lines.front().find("DRONE_HITS_OBSTACLE"), std::string::npos);
    EXPECT_NE(lines.front().find("collision at step 3"), std::string::npos);

    std::filesystem::remove_all(output_path, ec);
}

TEST(SimulationRunTest, Run_PopulatesOutputMapFileAndConfigs) {
    auto mission_control = std::make_unique<MissionControlMock>();
    EXPECT_CALL(*mission_control, runMission())
        .WillOnce(testing::Return(types::MissionRunResult{
            types::MissionRunStatus::Completed,
            1,
            {},
        }));

    const types::SimulationConfigData simulation_config = makeDefaultSimulationConfig();
    types::MissionConfigData mission_config = makeDefaultMissionConfig();
    mission_config.output_mapping_resolution_factor = 2.0;
    const std::filesystem::path output_map_file = "output_results/run_0003/output_map.npy";

    auto run = makeSimulationRun(
        std::move(mission_control),
        output_map_file,
        simulation_config,
        mission_config);

    const types::SimulationResult result = run->run();
    EXPECT_EQ(result.output_map_file, output_map_file);
    EXPECT_EQ(result.simulation_config.map_filename, simulation_config.map_filename);
    EXPECT_DOUBLE_EQ(
        result.mission_config.output_mapping_resolution_factor,
        mission_config.output_mapping_resolution_factor);
    EXPECT_DOUBLE_EQ(result.output_map_config.resolution.numerical_value_in(cm), 1.0);
    EXPECT_EQ(result.resolution_request_status, types::ResolutionRequestStatus::Accepted);
}

TEST(SimulationRunTest, Run_StartupErrors_SkipsMissionAndReturnsMinusOne) {
    auto mission_control = std::make_unique<MissionControlMock>();
    EXPECT_CALL(*mission_control, runMission()).Times(0);

    const std::vector<types::ErrorRef> startup_errors{
        types::ErrorRef{"MAP_FILE_NOT_FOUND", "missing.npy"},
    };
    auto run = makeSimulationRun(
        std::move(mission_control),
        "output_results/run_0004/output_map.npy",
        makeDefaultSimulationConfig(),
        makeDefaultMissionConfig(),
        startup_errors);

    const types::SimulationResult result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Error);
    ASSERT_EQ(result.mission_results.front().errors.size(), 1U);
    EXPECT_EQ(result.mission_results.front().errors.front().code, "MAP_FILE_NOT_FOUND");
}

TEST(SimulationRunTest, Run_MaxSteps_StillScores) {
    auto mission_control = std::make_unique<MissionControlMock>();
    EXPECT_CALL(*mission_control, runMission())
        .WillOnce(testing::Return(types::MissionRunResult{
            types::MissionRunStatus::MaxSteps,
            100,
            {},
        }));

    auto run = makeSimulationRun(
        std::move(mission_control),
        "output_results/run_0005/output_map.npy",
        makeDefaultSimulationConfig(),
        makeDefaultMissionConfig());

    const types::SimulationResult result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::MaxSteps);
}

} // namespace
} // namespace drone_mapper
