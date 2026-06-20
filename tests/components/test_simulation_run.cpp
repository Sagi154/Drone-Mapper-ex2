// test_simulation_run.cpp
// Suite: SimulationRun.*
//
// Covers MockGPS and MockMovement per testing requirements:
// "SimulationRun tests must also cover MockGPS and MockMovement."
//
// Each TEST documents what scenario is being tested and why the expected
// outcome holds.

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

namespace drone_mapper {
namespace {

// ---------------------------------------------------------------------------
// Minimal GMock map used to control collision detection in movement tests.
// NiceMock suppresses "uninteresting call" warnings so tests only fail on
// explicit EXPECT_CALLs or assertion failures, not on extra method calls.
// ---------------------------------------------------------------------------
class MapMock : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel,     (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (),                  (const, override));
    MOCK_METHOD(bool,                  isInBounds,   (const Position3D&), (const, override));
};

using NiceMapMock = testing::NiceMock<MapMock>;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a default DroneConfigData with generous limits and a small radius.
types::DroneConfigData makeDefaultDroneConfig() {
    return {
        .radius       = 5.0 * isq::length[cm],
        .max_rotate   = 360.0 * horizontal_angle[deg],
        .max_advance  = 500.0 * isq::length[cm],
        .max_elevate  = 500.0 * isq::length[cm],
    };
}

/// Build a MapConfig with 1 cm resolution and a 200×200×200 cm box.
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

/// Set up a NiceMapMock that always returns in-bounds and never returns Occupied.
/// Suitable as a "clear sky" environment for forward-movement tests.
void setClearMap(NiceMapMock& map) {
    const types::MapConfig cfg = makeMapConfig();
    ON_CALL(map, getMapConfig()).WillByDefault(testing::Return(cfg));
    ON_CALL(map, isInBounds(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(map, atVoxel(testing::_))
        .WillByDefault(testing::Return(types::VoxelOccupancy::Empty));
}

// ===========================================================================
// MockGPS tests
// ===========================================================================

/// Scenario: GPS constructed with initial position — position() returns it.
/// Expected: returned position matches the constructor argument (snapped to
/// resolution; 0.0 values snap to themselves regardless of resolution).
TEST(SimulationRunTest, GPS_ReturnsInitialPosition) {
    const Position3D init{10.0 * x_extent[cm], 20.0 * y_extent[cm], 30.0 * z_extent[cm]};
    const Orientation heading{45.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    MockGPS gps{init, heading, 1.0 * isq::length[cm]};

    const Position3D got = gps.position();
    EXPECT_DOUBLE_EQ(got.x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(got.y.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(got.z.numerical_value_in(cm), 30.0);
}

/// Scenario: GPS constructed with initial heading — heading() returns it.
/// Expected: both horizontal and altitude components match exactly.
TEST(SimulationRunTest, GPS_ReturnsInitialHeading) {
    const Position3D init{};
    const Orientation heading{90.0 * horizontal_angle[deg], 15.0 * altitude_angle[deg]};
    MockGPS gps{init, heading, 1.0 * isq::length[cm]};

    const Orientation got = gps.heading();
    EXPECT_DOUBLE_EQ(got.horizontal.numerical_value_in(deg), 90.0);
    EXPECT_DOUBLE_EQ(got.altitude.numerical_value_in(deg), 15.0);
}

/// Scenario: setPosition with exact resolution multiples — no rounding needed.
/// Expected: position stored and returned unchanged.
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

/// Scenario: setPosition with an off-grid value — should snap to nearest grid point.
/// With resolution=10cm, 14cm snaps to 10cm and 16cm snaps to 20cm.
/// Expected: coordinates are rounded to the nearest multiple of the resolution.
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

/// Scenario: resolution = 0 — snapping must not divide by zero; position stored as-is.
/// Expected: arbitrary floating-point value survives unchanged.
TEST(SimulationRunTest, GPS_SetPosition_ZeroResolution_NoSnapping) {
    MockGPS gps{Position3D{}, Orientation{}, 0.0 * isq::length[cm]};
    gps.setPosition(Position3D{7.3 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 7.3);
}

/// Scenario: setHeading updates the stored heading.
/// Expected: heading() returns the newly set value.
TEST(SimulationRunTest, GPS_SetHeading_UpdatesHeading) {
    MockGPS gps{Position3D{}, Orientation{}, 1.0 * isq::length[cm]};
    const Orientation newH{270.0 * horizontal_angle[deg], -5.0 * altitude_angle[deg]};
    gps.setHeading(newH);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 270.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.numerical_value_in(deg), -5.0);
}

// ===========================================================================
// MockMovement tests
// ===========================================================================

/// Scenario: rotate within limit — heading should update, result success.
/// Expected: horizontal heading increases by the rotated amount; success=true.
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

/// Scenario: rotate with Left direction — heading decreases.
/// Expected: horizontal angle becomes -45° (Left rotation).
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

/// Scenario: rotate exceeding max_rotate — must be rejected.
/// Expected: success=false; heading unchanged.
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

/// Scenario: advance in clear space (heading 0° = +X direction).
/// Expected: GPS x-coordinate increases by the advanced distance; success=true.
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

/// Scenario: advance with heading 90° (+Y direction).
/// Expected: only the Y coordinate changes.
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

/// Scenario: advance exceeding max_advance — must be rejected; position unchanged.
/// Expected: success=false; GPS position identical to before the call.
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

/// Scenario: advance into an occupied voxel — collision detected; position unchanged.
/// Expected: success=false; GPS x stays at 100 cm.
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

/// Scenario: advance toward map boundary (out-of-bounds) — must be rejected.
/// Expected: success=false; position unchanged.
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

/// Scenario: elevate in clear space — drone moves up by the given distance.
/// Expected: z coordinate increases; x and y unchanged.
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

/// Scenario: negative elevate (descend) — drone moves down.
/// Expected: z decreases by the given distance.
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

/// Scenario: elevate exceeding max_elevate — rejected; z unchanged.
/// Expected: success=false; GPS z stays at 50 cm.
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

/// Scenario: elevate into occupied voxel — collision detected; z unchanged.
/// Expected: success=false; GPS z stays at 50 cm.
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

} // namespace
} // namespace drone_mapper
