// test_mapping_algorithm_frontier.cpp — MappingAlgorithm.*
// Direct tests for internal MappingAlgorithmFrontier BFS (ported from ex1).

#include <drone_mapper/Map3DImpl.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include "MappingAlgorithmFrontier.h"

#include <algorithm>
#include <cstdint>
#include <memory>

namespace drone_mapper {
namespace {

[[nodiscard]] types::MapConfig makeWideConfig() {
    types::MapConfig config{};
    config.resolution = 1.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = -50.0 * x_extent[cm];
    config.boundaries.max_x = 50.0 * x_extent[cm];
    config.boundaries.min_y = -50.0 * y_extent[cm];
    config.boundaries.max_y = 50.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

[[nodiscard]] types::MapConfig makeNarrowCorridorConfig() {
    types::MapConfig config{};
    config.resolution = 1.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 10.0 * x_extent[cm];
    config.boundaries.min_y = -1.0 * y_extent[cm];
    config.boundaries.max_y = 1.0 * y_extent[cm];
    config.boundaries.min_height = 49.0 * z_extent[cm];
    config.boundaries.max_height = 51.0 * z_extent[cm];
    return config;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeUnmappedMap(const NpyArray::shape_t& shape) {
    auto map = std::make_shared<NpyArray>(shape, sizeof(std::int8_t),
                                          NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(), map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] Position3D pointCm(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

void fillEmptyBox(Map3DImpl& map, int x0, int x1, int y0, int y1, int z0, int z1,
                  const types::MapConfig& config) {
    const double step = config.resolution.force_numerical_value_in(cm);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                map.set(Position3D{
                            (ox + x * step) * x_extent[cm],
                            (oy + y * step) * y_extent[cm],
                            (oz + z * step) * z_extent[cm]},
                        types::VoxelOccupancy::Empty);
            }
        }
    }
}

void fillEmptyCube(Map3DImpl& map, int cx, int cy, int cz, int half,
                   const types::MapConfig& config) {
    fillEmptyBox(map, cx - half, cx + half, cy - half, cy + half, cz - half, cz + half, config);
}

} // namespace

// What: start cell is Empty but neighbours within drone_radius are Unmapped.
// Expected: findPath returns found=false because the start is not sphere-passable.
TEST(MappingAlgorithmTest, FrontierStartNotPassableWhenSphereHasUnmapped) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{101, 101, 101}), config};
    map.set(pointCm(0, 0, 50), types::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result =
        frontier.findPath(map, pointCm(0, 0, 50), 5.0 * cm);

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
}

// What: centre is Empty and surrounding sphere cells are Occupied walls.
// Expected: start is passable — Occupied inside the body sphere is allowed.
TEST(MappingAlgorithmTest, FrontierStartPassableWhenSphereHasOccupiedWalls) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{101, 101, 101}), config};
    map.set(pointCm(0, 0, 50), types::VoxelOccupancy::Empty);
    for (int dx = -5; dx <= 5; ++dx) {
        for (int dy = -5; dy <= 5; ++dy) {
            for (int dz = -5; dz <= 5; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                if (dx * dx + dy * dy + dz * dz > 25) {
                    continue;
                }
                map.set(pointCm(dx, dy, 50 + dz), types::VoxelOccupancy::Occupied);
            }
        }
    }

    const detail::MappingAlgorithmFrontier frontier;
    const detail::PlanningDiagnostics diag =
        frontier.diagnose(map, pointCm(0, 0, 50), 5.0 * cm);
    EXPECT_TRUE(diag.start_passable);
}

// What: centre cell is Occupied.
// Expected: start is not passable.
TEST(MappingAlgorithmTest, FrontierStartNotPassableWhenCentreOccupied) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{101, 101, 101}), config};
    map.set(pointCm(0, 0, 50), types::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::PlanningDiagnostics diag =
        frontier.diagnose(map, pointCm(0, 0, 50), 5.0 * cm);
    EXPECT_FALSE(diag.start_passable);
}

// What: straight empty corridor ending before unknown space along +X.
// Expected: path reaches the nearest frontier two steps ahead on a zero-radius drone.
TEST(MappingAlgorithmTest, FrontierFindsPathAlongEmptyCorridor) {
    const types::MapConfig config = makeNarrowCorridorConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(map, 0, 2, -1, 1, 49, 51, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result =
        frontier.findPath(map, pointCm(0, 0, 50), 0.0 * cm);

    ASSERT_TRUE(result.found);
    ASSERT_EQ(result.path.size(), 2U);
    EXPECT_EQ(result.path[1].x.force_numerical_value_in(cm), 2.0);
    EXPECT_EQ(result.path[1].y.force_numerical_value_in(cm), 0.0);
}

// What: large empty cube with a 5 cm drone radius.
// Expected: BFS reaches a frontier near the cube face at x=3 cm.
TEST(MappingAlgorithmTest, FrontierFindsFrontierInsideEmptyCube) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{101, 101, 101}), config};
    fillEmptyCube(map, 0, 0, 50, 8, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result =
        frontier.findPath(map, pointCm(0, 0, 50), 5.0 * cm);

    ASSERT_TRUE(result.found);
    ASSERT_FALSE(result.path.empty());
    EXPECT_EQ(result.path.back().x.force_numerical_value_in(cm), 3.0);
    EXPECT_EQ(result.path.back().y.force_numerical_value_in(cm), 0.0);
    EXPECT_EQ(result.path.back().z.force_numerical_value_in(cm), 50.0);
}

// What: unstored cells exist inside the probe sphere.
// Expected: hasNotMappedInSphere returns true.
TEST(MappingAlgorithmTest, FrontierHasUnmappedInSphereWhenUnknownExists) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{101, 101, 101}), config};
    map.set(pointCm(0, 0, 50), types::VoxelOccupancy::Empty);

    EXPECT_TRUE(detail::hasNotMappedInSphere(map, pointCm(0, 0, 50), 2.0 * cm));
}

// What: every cell in the probe sphere is explicitly Empty.
// Expected: hasNotMappedInSphere returns false.
TEST(MappingAlgorithmTest, FrontierHasNoUnmappedInSphereWhenFullyKnown) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{101, 101, 101}), config};
    fillEmptyCube(map, 0, 0, 50, 2, config);

    EXPECT_FALSE(detail::hasNotMappedInSphere(map, pointCm(0, 0, 50), 2.0 * cm));
}

// What: long corridor with unknown beyond fused range.
// Expected: findExplorePath returns a non-empty path toward unknown space.
TEST(MappingAlgorithmTest, FrontierFindExplorePathMovesTowardUnknown) {
    const types::MapConfig config = makeNarrowCorridorConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(map, 0, 8, -1, 1, 49, 51, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult explore =
        frontier.findExplorePath(map, pointCm(0, 0, 50), 0.0 * cm);

    ASSERT_TRUE(explore.found);
    ASSERT_FALSE(explore.path.empty());
    EXPECT_GT(explore.path.back().x.force_numerical_value_in(cm), 0.0);
}

// What: corridor with no reachable frontier at zero radius but distant unknown.
// Expected: diagnose reports connectivity and explore_path_available.
TEST(MappingAlgorithmTest, FrontierDiagnoseReportsConnectivityMetrics) {
    const types::MapConfig config = makeNarrowCorridorConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{11, 3, 3}), config};
    fillEmptyBox(map, 0, 8, -1, 1, 49, 51, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::PlanningDiagnostics diag =
        frontier.diagnose(map, pointCm(0, 0, 50), 0.0 * cm);

    EXPECT_TRUE(diag.start_passable);
    EXPECT_GT(diag.passable_reached, 0U);
    EXPECT_GE(diag.nearest_unknown_steps, 0);
    EXPECT_TRUE(diag.explore_path_available);
}

// What: map still contains Unmapped voxels inside mission bounds.
// Expected: hasAnyNotMappedInBounds returns true.
TEST(MappingAlgorithmTest, FrontierDetectsUnmappedCellsInBounds) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{101, 101, 101}), config};
    map.set(pointCm(0, 0, 50), types::VoxelOccupancy::Empty);

    EXPECT_TRUE(detail::hasAnyNotMappedInBounds(map));
}

// What: every in-bounds cell is marked Empty.
// Expected: hasAnyNotMappedInBounds returns false.
TEST(MappingAlgorithmTest, FrontierNoUnmappedWhenFullyMappedEmpty) {
    const types::MapConfig config = makeWideConfig();
    Map3DImpl map{makeUnmappedMap(NpyArray::shape_t{5, 5, 5}), config};
    fillEmptyBox(map, 0, 4, 0, 4, 0, 4, config);

    EXPECT_FALSE(detail::hasAnyNotMappedInBounds(map));
}

} // namespace drone_mapper
