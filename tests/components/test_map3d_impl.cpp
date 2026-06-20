#include <drone_mapper/Map3DImpl.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

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

[[nodiscard]] std::shared_ptr<NpyArray> loadMapFile(const std::filesystem::path& path) {
    auto map = std::make_shared<NpyArray>();
    const LPCSTR error = map->LoadNPY(path.string());
    if (error != nullptr) {
        throw std::runtime_error(error);
    }
    return map;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyMutableMap(const NpyArray::shape_t& shape) {
    auto map = std::make_shared<NpyArray>(shape, sizeof(std::int8_t), NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(), map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] std::filesystem::path goldenMapPath() {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "data_maps" / "single_voxel_x2_y4_z2.npy";
#else
    return std::filesystem::path{"data_maps/single_voxel_x2_y4_z2.npy"};
#endif
}

} // namespace

TEST(Map3DImpl, RejectsNullMapPointer) {
    EXPECT_THROW(Map3DImpl(std::shared_ptr<NpyArray>{}), std::invalid_argument);
}

TEST(Map3DImpl, ReportsInBoundsForKnownVoxelCenter) {
    const auto map = loadMapFile(goldenMapPath());
    const Map3DImpl map_impl{map, makeUnitCubeConfig()};

    const Position3D occupied{20.0 * x_extent[cm], 40.0 * y_extent[cm], 20.0 * z_extent[cm]};
    EXPECT_TRUE(map_impl.isInBounds(occupied));
    EXPECT_EQ(map_impl.atVoxel(occupied), types::VoxelOccupancy::Occupied);
}

TEST(Map3DImpl, ReturnsOutOfBoundsOutsideVolume) {
    const auto map = loadMapFile(goldenMapPath());
    const Map3DImpl map_impl{map, makeUnitCubeConfig()};

    const Position3D outside{50.0 * x_extent[cm], 40.0 * y_extent[cm], 20.0 * z_extent[cm]};
    EXPECT_FALSE(map_impl.isInBounds(outside));
    EXPECT_EQ(map_impl.atVoxel(outside), types::VoxelOccupancy::OutOfBounds);
}

TEST(Map3DImpl, SetUpdatesVoxelAndSaveLoadRoundtripPreservesValue) {
    const auto map = makeEmptyMutableMap(NpyArray::shape_t{3, 3, 3});
    Map3DImpl map_impl{map, makeUnitCubeConfig()};

    const Position3D target{10.0 * x_extent[cm], 10.0 * y_extent[cm], 10.0 * z_extent[cm]};
    map_impl.set(target, types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map_impl.atVoxel(target), types::VoxelOccupancy::Occupied);

    const std::filesystem::path temp_path =
        std::filesystem::temp_directory_path() / "map3d_impl_roundtrip_test.npy";
    map_impl.save(temp_path);

    const auto reloaded = loadMapFile(temp_path);
    const Map3DImpl reloaded_impl{reloaded, makeUnitCubeConfig()};
    EXPECT_EQ(reloaded_impl.atVoxel(target), types::VoxelOccupancy::Occupied);

    std::error_code ec;
    std::filesystem::remove(temp_path, ec);
}

TEST(Map3DImpl, SetOutOfBoundsIsNoOp) {
    const auto map = makeEmptyMutableMap(NpyArray::shape_t{3, 3, 3});
    Map3DImpl map_impl{map, makeUnitCubeConfig()};

    const Position3D outside{100.0 * x_extent[cm], 10.0 * y_extent[cm], 10.0 * z_extent[cm]};
    map_impl.set(outside, types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map_impl.atVoxel(outside), types::VoxelOccupancy::OutOfBounds);
}

TEST(Map3DImpl, DerivesBoundsFromShapeWhenUnset) {
    const auto map = loadMapFile(goldenMapPath());
    types::MapConfig config{};
    config.resolution = 10.0 * cm;

    const Map3DImpl map_impl{map, config};
    const types::MapConfig derived = map_impl.getMapConfig();

    EXPECT_EQ(derived.boundaries.max_x, 40.0 * x_extent[cm]);
    EXPECT_EQ(derived.boundaries.max_y, 40.0 * y_extent[cm]);
    EXPECT_EQ(derived.boundaries.max_height, 40.0 * z_extent[cm]);
}

} // namespace drone_mapper
