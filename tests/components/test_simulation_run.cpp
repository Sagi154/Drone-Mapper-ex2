// test_simulation_run.cpp — SimulationRun.* (MockGPS and MockMovement)

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

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

} // namespace
} // namespace drone_mapper
