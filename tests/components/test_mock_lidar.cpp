// test_mock_lidar.cpp — MockLidar.*

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>

namespace drone_mapper {
namespace {

[[nodiscard]] types::MapConfig makeLongRangeMapConfig() {
    types::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 200.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 40.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 40.0 * z_extent[cm];
    return config;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyMap(const NpyArray::shape_t& shape) {
    auto map = std::make_shared<NpyArray>(
        shape,
        sizeof(std::int8_t),
        NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(),
                map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Empty));
    return map;
}

[[nodiscard]] types::LidarConfigData makeCenterBeamLidar() {
    return types::LidarConfigData{
        20.0 * cm,
        100.0 * cm,
        2.5 * cm,
        1,
    };
}

[[nodiscard]] Orientation zeroOrientation() {
    return Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
}

[[nodiscard]] double maxMissDistanceCm() {
    return std::numeric_limits<double>::max();
}

struct MockLidarFixture {
    std::shared_ptr<NpyArray> array = makeEmptyMap(NpyArray::shape_t{20, 4, 4});
    Map3DImpl map{array, makeLongRangeMapConfig()};
    MockGPS gps{Position3D{}, zeroOrientation(), 10.0 * cm};
    types::LidarConfigData lidar_config = makeCenterBeamLidar();

    void setOccupiedAlongBeam(PhysicalLength distance) {
        const Position3D pos{
            distance.force_numerical_value_in(cm) * x_extent[cm],
            0.0 * y_extent[cm],
            0.0 * z_extent[cm],
        };
        map.set(pos, types::VoxelOccupancy::Occupied);
    }

    [[nodiscard]] types::LidarScanResult scanCenterBeam() const {
        MockLidar lidar{lidar_config, map, gps};
        return lidar.scan(zeroOrientation());
    }
};

} // namespace

// What: unobstructed center beam along +X.
// Expected: miss distance is max double centimeters (skeleton miss sentinel).
TEST(MockLidar, OpenBeamReturnsMaxRange) {
    MockLidarFixture fixture;
    const types::LidarScanResult scan = fixture.scanCenterBeam();

    ASSERT_EQ(scan.size(), 1U);
    EXPECT_DOUBLE_EQ(scan.front().distance.force_numerical_value_in(cm), maxMissDistanceCm());
}

// What: occupied voxel exactly at z_max along the beam.
// Expected: hit distance equals z_max (catches rays truncated before max range).
TEST(MockLidar, DetectsObstacleAtFarEnd) {
    MockLidarFixture fixture;
    fixture.setOccupiedAlongBeam(100.0 * cm);

    const types::LidarScanResult scan = fixture.scanCenterBeam();

    ASSERT_EQ(scan.size(), 1U);
    EXPECT_DOUBLE_EQ(scan.front().distance.force_numerical_value_in(cm), 100.0);
}

// What: occupied voxel one resolution step before z_max (grid-aligned).
// Expected: hit distance equals that near-max range (catches far-end detection bugs).
TEST(MockLidar, DetectsObstacleOneStepBeforeMax) {
    MockLidarFixture fixture;
    fixture.setOccupiedAlongBeam(90.0 * cm);

    const types::LidarScanResult scan = fixture.scanCenterBeam();

    ASSERT_EQ(scan.size(), 1U);
    EXPECT_DOUBLE_EQ(scan.front().distance.force_numerical_value_in(cm), 90.0);
}

// What: obstacle beyond z_max along the beam.
// Expected: beam still reports a miss (max double), not a false hit.
TEST(MockLidar, ObstacleBeyondMaxRangeNotDetected) {
    MockLidarFixture fixture;
    fixture.setOccupiedAlongBeam(110.0 * cm);

    const types::LidarScanResult scan = fixture.scanCenterBeam();

    ASSERT_EQ(scan.size(), 1U);
    EXPECT_DOUBLE_EQ(scan.front().distance.force_numerical_value_in(cm), maxMissDistanceCm());
}

// What: obstacle closer than z_min.
// Expected: MockLidar reports zero distance for near-field hits.
TEST(MockLidar, ObstacleBeforeZMinReturnsZero) {
    MockLidarFixture fixture;
    fixture.setOccupiedAlongBeam(10.0 * cm);

    const types::LidarScanResult scan = fixture.scanCenterBeam();

    ASSERT_EQ(scan.size(), 1U);
    EXPECT_DOUBLE_EQ(scan.front().distance.force_numerical_value_in(cm), 0.0);
}

// What: lidar configured with zero FOV circles.
// Expected: scan returns no hits.
TEST(MockLidar, ZeroFovCirclesReturnsEmptyResult) {
    MockLidarFixture fixture;
    fixture.lidar_config.fov_circles = 0;

    const types::LidarScanResult scan = fixture.scanCenterBeam();

    EXPECT_TRUE(scan.empty());
}

} // namespace drone_mapper
