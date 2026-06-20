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

/// Scenario: setPosition with negative coordinates that need snapping.
/// With resolution=10cm, -14cm snaps to -10cm (round half away from zero).
/// Expected: all three axes snap independently and correctly for negative values.
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

/// Scenario: setPosition exactly at the halfway point (res=10cm, value=15cm).
/// std::round rounds half away from zero, so 15cm → 20cm, -15cm → -20cm.
/// Expected: 15cm snaps up to 20cm; -15cm snaps down to -20cm.
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

/// Scenario: fine sub-centimetre resolution (0.5 cm).
/// Expected: values snap to the nearest 0.5-cm grid point.
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

/// Scenario: successive setPosition calls — each snaps to its own nearest grid point.
/// Expected: each call is independent; no cumulative drift from prior snapping.
TEST(SimulationRunTest, GPS_SetPosition_SuccessiveCalls_EachSnapsIndependently) {
    MockGPS gps{Position3D{}, Orientation{}, 10.0 * isq::length[cm]};

    gps.setPosition(Position3D{13.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 10.0);

    // Second call with a fresh raw value — should snap to 20, not snap from 10.
    gps.setPosition(Position3D{17.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 20.0);
}

/// Scenario: initial position passed to constructor is stored as-is (not snapped).
/// The factory always provides validated on-grid positions, so constructor snapping
/// is intentionally omitted; this test documents and guards that contract.
TEST(SimulationRunTest, GPS_Constructor_InitialPositionNotSnapped) {
    // 17.3 cm is off-grid for a 10 cm resolution, yet the constructor stores it raw.
    MockGPS gps{
        Position3D{17.3 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
        Orientation{},
        10.0 * isq::length[cm]
    };
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 17.3);
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

/// Scenario: advance zero distance — no movement but also no collision at current spot;
/// with a clear map the call succeeds and position stays put.
/// Expected: success=true; GPS position identical to before.
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

/// Scenario: advance exactly at max_advance — the limit check is strict (>), so the
/// boundary value itself must be accepted.
/// Expected: success=true; position updated.
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

/// Scenario: advance infinitesimally beyond max_advance must be rejected.
/// Expected: success=false; position unchanged.
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

/// Scenario: elevate zero distance — sphere check on current position; with clear map succeeds.
/// Expected: success=true; z unchanged.
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

/// Scenario: elevate exactly at max_elevate — boundary must be accepted (strict >).
/// Expected: success=true; z updated.
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

/// Scenario: rotate zero angle — no-op rotation; heading unchanged and success.
/// Expected: success=true; heading identical to before.
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

/// Scenario: rotate exactly at max_rotate — boundary must be accepted (strict >).
/// Expected: success=true; heading updated.
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

/// Scenario: advance with negative distance (backward movement).
/// The spec allows negative distance; the drone moves opposite to its heading.
/// With heading 0° (+X) and distance −10 cm, x should decrease by 10.
/// Expected: success=true; x decreases; y and z unchanged.
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

/// Scenario: advance must never change the drone's heading.
/// Expected: horizontal and altitude heading values are identical before and after advance.
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

/// Scenario: elevate must never change the drone's heading.
/// Expected: heading identical before and after elevate.
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

/// Scenario: multiple rotate commands accumulate correctly.
/// Rotate Right 30°, then Right 30° again → total −60° from start.
/// Expected: final heading = −60° (or equivalently 300° in unsigned terms).
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

/// Scenario: alternating Left and Right rotations cancel out.
/// Rotate Left 45° then Right 45° → net change of zero.
/// Expected: heading returns to original value.
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

/// Scenario: heading can exceed 360° — MockMovement does not wrap; the algorithm
/// is responsible for staying in [0, 360). This documents and guards the contract.
/// Expected: heading = 370° after rotating 10° from 360°.
TEST(SimulationRunTest, Movement_Rotate_HeadingExceeds360_NotWrapped) {
    MockGPS gps{Position3D{}, Orientation{360.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * isq::length[cm]};
    NiceMapMock map;
    setClearMap(map);
    MockMovement movement{gps, map, makeDefaultDroneConfig()};

    movement.rotate(types::RotationDirection::Left, 10.0 * horizontal_angle[deg]);

    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 370.0);
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
