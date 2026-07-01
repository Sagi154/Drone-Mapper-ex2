// test_mapping_algorithm.cpp — MappingAlgorithm.*
// Tests MappingAlgorithmImpl nextStep state machine and command emission.

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <tuple>

namespace drone_mapper {
namespace {

[[nodiscard]] types::MapConfig makeCorridorConfig() {
    types::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 100.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 100.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

[[nodiscard]] types::DroneConfigData makeDroneConfig() {
    types::DroneConfigData cfg{};
    cfg.radius = 5.0 * cm;
    cfg.max_rotate = 45.0 * deg;
    cfg.max_advance = 10.0 * cm;
    cfg.max_elevate = 10.0 * cm;
    return cfg;
}

[[nodiscard]] types::LidarConfigData makeLidarConfig() {
    types::LidarConfigData cfg{};
    cfg.z_min = 1.0 * cm;
    cfg.z_max = 200.0 * cm;
    cfg.d = 1.0 * cm;
    cfg.fov_circles = 1;
    return cfg;
}

[[nodiscard]] types::MissionConfigData makeMissionConfig() {
    types::MissionConfigData cfg{};
    cfg.max_steps = 10000;
    cfg.gps_resolution = 1.0 * cm;
    return cfg;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyMap(const NpyArray::shape_t& shape) {
    auto map = std::make_shared<NpyArray>(shape, sizeof(std::int8_t),
                                          NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(), map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] Position3D gridPoint(int x, int y, int z, const types::MapConfig& config) {
    const double step = config.resolution.force_numerical_value_in(cm);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    return Position3D{
        (ox + static_cast<double>(x) * step) * x_extent[cm],
        (oy + static_cast<double>(y) * step) * y_extent[cm],
        (oz + static_cast<double>(z) * step) * z_extent[cm],
    };
}

void fillEmptyBox(Map3DImpl& map, int x0, int x1, int y0, int y1, int z0, int z1,
                  const types::MapConfig& config) {
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                map.set(gridPoint(x, y, z, config), types::VoxelOccupancy::Empty);
            }
        }
    }
}

[[nodiscard]] std::optional<types::MappingStepCommand>
firstCommandMatching(MappingAlgorithmImpl& algorithm, const types::DroneState& state,
                     int max_steps,
                     const std::function<bool(const types::MappingStepCommand&)>& predicate) {
    for (int step = 0; step < max_steps; ++step) {
        const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (predicate(cmd)) {
            return cmd;
        }
        if (cmd.status != types::AlgorithmStatus::Working) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::set<std::tuple<double, double>>
collectScanOrientationsUntilNonScan(MappingAlgorithmImpl& algorithm,
                                    const types::DroneState& state, int max_steps) {
    std::set<std::tuple<double, double>> orientations;
    for (int step = 0; step < max_steps; ++step) {
        const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (!cmd.scan_orientation.has_value()) {
            break;
        }
        const double az = cmd.scan_orientation->horizontal.force_numerical_value_in(deg);
        const double el = cmd.scan_orientation->altitude.force_numerical_value_in(deg);
        orientations.insert({az, el});
        if (cmd.status != types::AlgorithmStatus::Working) {
            break;
        }
    }
    return orientations;
}

void fillStartBubble(Map3DImpl& map, int cx, int cy, int cz, const types::MapConfig& config) {
    map.set(gridPoint(cx, cy, cz, config), types::VoxelOccupancy::Empty);
    fillEmptyBox(map, cx - 1, cx + 1, cy, cy, cz, cz, config);
    fillEmptyBox(map, cx, cx, cy - 1, cy + 1, cz, cz, config);
    fillEmptyBox(map, cx, cx, cy, cy, cz - 1, cz + 1, config);
}

[[nodiscard]] types::AlgorithmStatus runUntilTerminal(MappingAlgorithmImpl& algorithm,
                                                    const types::DroneState& state,
                                                    int max_steps) {
    types::AlgorithmStatus status = types::AlgorithmStatus::Working;
    for (int step = 0; step < max_steps; ++step) {
        status = algorithm.nextStep(state, nullptr).status;
        if (status != types::AlgorithmStatus::Working) {
            return status;
        }
    }
    return status;
}

} // namespace

// What: first nextStep on a fresh mission with no prior scan.
// Expected: algorithm requests a scan orientation (latest_scan is nullptr).
TEST(MappingAlgorithmTest, FirstStepRequestsScanWithNullLatestScan) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 11, 11}), config};
    fillEmptyBox(output_map, 4, 6, 4, 6, 4, 6, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
    EXPECT_EQ(cmd.status, types::AlgorithmStatus::Working);
    ASSERT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_FALSE(cmd.movement.has_value());
}

// What: fully mapped empty volume with no adjacent unknown cells.
// Expected: after exhausting scan sweep, algorithm reports Finished.
TEST(MappingAlgorithmTest, FinishesWhenNoFrontierRemains) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{3, 3, 3}), config};
    fillEmptyBox(output_map, 0, 2, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const types::AlgorithmStatus last_status = runUntilTerminal(algorithm, state, 5000);
    EXPECT_EQ(last_status, types::AlgorithmStatus::Finished);
}

// What: empty corridor with unknown space beyond +X; start sphere is passable.
// Expected: planning finds a frontier and nextStep emits movement toward it.
TEST(MappingAlgorithmTest, EmitsMovementTowardFrontier) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const types::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type != types::MovementCommandType::Hover;
        });

    ASSERT_TRUE(cmd.has_value());
    EXPECT_EQ(cmd->status, types::AlgorithmStatus::Working);
}

// What: algorithm begins in scanning phase on a fresh mission.
// Expected: multiple consecutive steps request scan orientations before movement.
TEST(MappingAlgorithmTest, ScanSweepEmitsMultipleOrientationsBeforeMovement) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    std::size_t scan_steps = 0;
    bool saw_movement = false;
    for (int step = 0; step < 500; ++step) {
        const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (cmd.scan_orientation.has_value()) {
            ++scan_steps;
        }
        if (cmd.movement.has_value() &&
            cmd.movement->type != types::MovementCommandType::Hover) {
            saw_movement = true;
            break;
        }
        if (cmd.status != types::AlgorithmStatus::Working) {
            break;
        }
    }

    EXPECT_GT(scan_steps, 1U);
    EXPECT_TRUE(saw_movement);
}

// What: exploration completes on a fully known map.
// Expected: subsequent nextStep calls keep returning Finished.
TEST(MappingAlgorithmTest, ReturnsFinishedOnSubsequentCallsAfterCompletion) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{3, 3, 3}), config};
    fillEmptyBox(output_map, 0, 2, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    ASSERT_EQ(runUntilTerminal(algorithm, state, 5000), types::AlgorithmStatus::Finished);

    for (int i = 0; i < 3; ++i) {
        const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        EXPECT_EQ(cmd.status, types::AlgorithmStatus::Finished);
        EXPECT_FALSE(cmd.scan_orientation.has_value());
        EXPECT_FALSE(cmd.movement.has_value());
    }
}

// What: drone heading is 90° while the frontier path lies along +X.
// Expected: first movement command is a Rotate toward the path.
TEST(MappingAlgorithmTest, EmitsRotateWhenHeadingMisalignedWithPath) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{gridPoint(2, 1, 1, config),
                                  Orientation{90.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const types::MappingStepCommand& step) {
            return step.movement.has_value();
        });

    ASSERT_TRUE(cmd.has_value());
    ASSERT_TRUE(cmd->movement.has_value());
    EXPECT_EQ(cmd->movement->type, types::MovementCommandType::Rotate);
}

// What: drone heading already faces +X along the corridor path.
// Expected: first movement command is Advance (or Hover if already at waypoint).
TEST(MappingAlgorithmTest, EmitsAdvanceWhenHeadingAlignedWithPath) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const types::MappingStepCommand& step) {
            return step.movement.has_value();
        });

    ASSERT_TRUE(cmd.has_value());
    ASSERT_TRUE(cmd->movement.has_value());
    EXPECT_EQ(cmd->movement->type, types::MovementCommandType::Advance);
}

// What: waypoint is at a higher z than the drone on the same column.
// Expected: first movement command elevates toward the waypoint.
TEST(MappingAlgorithmTest, EmitsElevateWhenWaypointHeightDiffers) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{5, 3, 5}), config};
    fillEmptyBox(output_map, 2, 2, 1, 1, 0, 3, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(2, 1, 0, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 800, [](const types::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type == types::MovementCommandType::Elevate;
        });

    ASSERT_TRUE(cmd.has_value());
    EXPECT_GT(cmd->movement->distance.force_numerical_value_in(cm), 0.0);
}

// What: each emitted command during scanning/planning/moving.
// Expected: algorithm never bundles scan and movement in the same step.
TEST(MappingAlgorithmTest, DoesNotCombineScanAndMovementInOneStep) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    for (int step = 0; step < 200; ++step) {
        const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        EXPECT_FALSE(cmd.scan_orientation.has_value() && cmd.movement.has_value());
        if (cmd.status != types::AlgorithmStatus::Working) {
            break;
        }
    }
}

// What: isolated empty cell surrounded by Unmapped with no frontier progress possible.
// Expected: algorithm eventually stops with FinishedWithUnmappableVoxels while unknown remains.
TEST(MappingAlgorithmTest, FinishesWithUnmappableWhenStartNotSpherePassable) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 11, 11}), config};
    output_map.set(gridPoint(5, 5, 5, config), types::VoxelOccupancy::Empty);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const types::AlgorithmStatus last_status = runUntilTerminal(algorithm, state, 8000);
    EXPECT_EQ(last_status, types::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

// What: scan pass zero on a fresh mission.
// Expected: first scan sweep includes all six axis-aligned orientations.
TEST(MappingAlgorithmTest, ScanPassZeroIncludesAxisAlignedOrientations) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 11, 11}), config};
    output_map.set(gridPoint(5, 5, 5, config), types::VoxelOccupancy::Empty);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const std::set<std::tuple<double, double>> orientations =
        collectScanOrientationsUntilNonScan(algorithm, state, 200);

    const std::set<std::tuple<double, double>> expected = {
        {0.0, 0.0}, {180.0, 0.0}, {90.0, 0.0}, {270.0, 0.0}, {0.0, 90.0}, {0.0, -90.0},
    };
    for (const auto& axis : expected) {
        EXPECT_TRUE(orientations.contains(axis)) << "missing axis-aligned orientation";
    }
}

// What: start position with a passable local bubble (center + six face neighbors Empty).
// Expected: planning finds a frontier and emits movement.
TEST(MappingAlgorithmTest, DroneNavigatesFromStartWhenAdjacentCellsAreEmpty) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 3, 3}), config};
    fillStartBubble(output_map, 2, 1, 1, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const types::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type != types::MovementCommandType::Hover;
        });

    ASSERT_TRUE(cmd.has_value());
    EXPECT_EQ(cmd->status, types::AlgorithmStatus::Working);
}

// What: latest_scan pointer is accepted but scanning phase is driven internally.
// Expected: providing a non-null latest_scan on step zero still begins with a scan request.
TEST(MappingAlgorithmTest, AcceptsNonNullLatestScanWithoutChangingFirstScanRequest) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 11, 11}), config};
    fillEmptyBox(output_map, 4, 6, 4, 6, 4, 6, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    types::LidarScanResult fake_scan{{types::LidarHit{100.0 * cm, Orientation{}}}};
    const types::MappingStepCommand cmd = algorithm.nextStep(state, &fake_scan);

    EXPECT_EQ(cmd.status, types::AlgorithmStatus::Working);
    ASSERT_TRUE(cmd.scan_orientation.has_value());
}

// What: scan commands from the redesigned algorithm.
// Expected: fusion_max is never set — DroneControl uses full lidar range.
TEST(MappingAlgorithmTest, NoFusionMaxEmittedInScanCommand) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 11, 11}), config};
    fillEmptyBox(output_map, 4, 6, 4, 6, 4, 6, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    for (int step = 0; step < 30; ++step) {
        const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (cmd.scan_orientation.has_value()) {
            EXPECT_FALSE(cmd.fusion_max.has_value());
        }
        if (cmd.status != types::AlgorithmStatus::Working) {
            break;
        }
    }
}

// What: corridor with unknown space beyond the fused empty region.
// Expected: algorithm keeps working through scan + explore/rescan instead of quitting early.
TEST(MappingAlgorithmTest, DoesNotTerminatePrematurely) {
    const types::MapConfig config = makeCorridorConfig();
    Map3DImpl output_map{makeEmptyMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    MappingAlgorithmImpl algorithm{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map};

    const types::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    int working_steps = 0;
    for (int step = 0; step < 100; ++step) {
        const types::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (cmd.status == types::AlgorithmStatus::Working) {
            ++working_steps;
            continue;
        }
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            break;
        }
    }

    EXPECT_GT(working_steps, 26);
}

} // namespace drone_mapper
