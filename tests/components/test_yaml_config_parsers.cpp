#include <drone_mapper/io/YamlConfigParsers.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

class CapturingErrorLog final : public io::IRunErrorLog {
public:
    void log(const types::ErrorRef& error) override {
        entries_.push_back(error);
    }

    [[nodiscard]] const std::vector<types::ErrorRef>& entries() const {
        return entries_;
    }

    [[nodiscard]] bool containsCode(const std::string& code) const {
        for (const types::ErrorRef& entry : entries_) {
            if (entry.code == code) {
                return true;
            }
        }
        return false;
    }

private:
    std::vector<types::ErrorRef> entries_;
};

[[nodiscard]] std::filesystem::path configFixturePath(const std::string& filename) {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "tests" / "data" / "configs" / filename;
#else
    return std::filesystem::path{"tests/data/configs"} / filename;
#endif
}

} // namespace

TEST(YamlConfigParser, ParsesDroneConfigWithRadiusConversion) {
    CapturingErrorLog log;
    const types::DroneConfigData config = io::parseDroneConfig(configFixturePath("drone_small.yaml"), log);

    EXPECT_EQ(config.radius, 15.0 * cm);
    EXPECT_EQ(config.max_rotate, 45.0 * horizontal_angle[deg]);
    EXPECT_EQ(config.max_advance, 50.0 * cm);
    EXPECT_EQ(config.max_elevate, 40.0 * cm);
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, ParsesMissionConfig) {
    CapturingErrorLog log;
    const types::MissionConfigData config = io::parseMissionConfig(configFixturePath("mission_basic.yaml"), log);

    EXPECT_EQ(config.max_steps, 2400U);
    EXPECT_EQ(config.gps_resolution, 10.0 * cm);
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 2.0);
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, ParsesLidarConfig) {
    CapturingErrorLog log;
    const types::LidarConfigData config = io::parseLidarConfig(configFixturePath("lidar_a.yaml"), log);

    EXPECT_EQ(config.z_min, 20.0 * cm);
    EXPECT_EQ(config.z_max, 120.0 * cm);
    EXPECT_EQ(config.d, 2.5 * cm);
    EXPECT_EQ(config.fov_circles, 5U);
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, ParsesSimulationConfig) {
    CapturingErrorLog log;
    const types::SimulationConfigData config =
        io::parseSimulationConfig(configFixturePath("sim_tiny.yaml"), log);

    EXPECT_EQ(config.map_filename, "../../../data_maps/single_voxel_x2_y4_z2.npy");
    EXPECT_EQ(config.map_resolution, 10.0 * cm);
    EXPECT_EQ(config.initial_drone_position.x, 20.0 * x_extent[cm]);
    EXPECT_EQ(config.initial_drone_position.y, 40.0 * y_extent[cm]);
    EXPECT_EQ(config.initial_drone_position.z, 20.0 * z_extent[cm]);
    EXPECT_EQ(config.initial_angle, 0.0 * horizontal_angle[deg]);
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, MissingFileReturnsDefaultsAndLogsError) {
    CapturingErrorLog log;
    const types::DroneConfigData config =
        io::parseDroneConfig(configFixturePath("missing_drone.yaml"), log);

    EXPECT_EQ(config.radius, PhysicalLength{});
    EXPECT_TRUE(log.containsCode("CONFIG_FILE_NOT_FOUND"));
}

TEST(YamlConfigParser, MissionBoundariesAreIgnoredWithLog) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(configFixturePath("mission_with_boundaries.yaml"), log);

    EXPECT_EQ(config.max_steps, 100U);
    EXPECT_TRUE(log.containsCode("MISSION_BOUNDARIES_IGNORED"));
}

TEST(YamlConfigParser, ParsesCompositionWithAlignedSimMissionPairs) {
    CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("composition_minimal.yaml"), log);

    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.value.simulations.size(), 1U);
    EXPECT_EQ(result.value.missions.size(), 1U);
    EXPECT_EQ(result.value.drones.size(), 1U);
    EXPECT_EQ(result.value.lidars.size(), 1U);
    EXPECT_EQ(result.value.simulations.size(), result.value.missions.size());
    EXPECT_EQ(result.value.simulations.front().map_resolution, 10.0 * cm);
    EXPECT_EQ(result.value.missions.front().max_steps, 2400U);
    EXPECT_EQ(result.value.drones.front().radius, 15.0 * cm);
    EXPECT_EQ(result.value.lidars.front().fov_circles, 5U);
}

TEST(YamlConfigParser, MissingCompositionFileFailsParse) {
    CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("missing_composition.yaml"), log);

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());
    EXPECT_TRUE(log.containsCode("CONFIG_FILE_NOT_FOUND"));
}

} // namespace drone_mapper
