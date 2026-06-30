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

[[nodiscard]] std::filesystem::path skeletonHouseSimulationPath() {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR}.parent_path() / "ex_2_skeleton" / "inputs" /
           "simulation" / "house_simulation.yaml";
#else
    return std::filesystem::path{"ex_2_skeleton/inputs/simulation/house_simulation.yaml"};
#endif
}

[[nodiscard]] std::filesystem::path instructorPath(const std::string& rel) {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "tests" / "data" / "instructor" / rel;
#else
    return std::filesystem::path{"tests/data/instructor"} / rel;
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

    ASSERT_TRUE(std::filesystem::exists(config.map_filename));
    EXPECT_EQ(config.map_filename.filename(), "single_voxel_x2_y4_z2.npy");
    EXPECT_EQ(config.map_resolution, 10.0 * cm);
    EXPECT_EQ(config.initial_drone_position.x, 20.0 * x_extent[cm]);
    EXPECT_EQ(config.initial_drone_position.y, 40.0 * y_extent[cm]);
    EXPECT_EQ(config.initial_drone_position.z, 20.0 * z_extent[cm]);
    EXPECT_EQ(config.initial_angle, 0.0 * horizontal_angle[deg]);
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, ParsesMapAxesOffsetWhenMapOffsetAbsent) {
    CapturingErrorLog log;
    const types::SimulationConfigData config =
        io::parseSimulationConfig(configFixturePath("sim_axes_offset.yaml"), log);

    EXPECT_EQ(config.map_offset.x, 1.0 * x_extent[cm]);
    EXPECT_EQ(config.map_offset.y, 2.0 * y_extent[cm]);
    EXPECT_EQ(config.map_offset.z, 150.0 * z_extent[cm]);
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, ParsesInstructorHouseSimulationMapAxesOffsetAndMapPath) {
    const std::filesystem::path path = skeletonHouseSimulationPath();
    if (!std::filesystem::exists(path)) {
        GTEST_SKIP() << "Skeleton inputs not available: " << path;
    }

    CapturingErrorLog log;
    const types::SimulationConfigData config = io::parseSimulationConfig(path, log);

    EXPECT_EQ(config.map_offset.x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.map_offset.y, 0.0 * y_extent[cm]);
    EXPECT_EQ(config.map_offset.z, 150.0 * z_extent[cm]);
    ASSERT_TRUE(std::filesystem::exists(config.map_filename));
    EXPECT_EQ(config.map_filename.filename(), "scenario_house.npy");
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, MissingFileReturnsDefaultsAndLogsError) {
    CapturingErrorLog log;
    const types::DroneConfigData config =
        io::parseDroneConfig(configFixturePath("missing_drone.yaml"), log);

    EXPECT_EQ(config.radius, PhysicalLength{});
    EXPECT_TRUE(log.containsCode("CONFIG_FILE_NOT_FOUND"));
    ASSERT_TRUE(config.config_load_error.has_value());
    EXPECT_EQ(config.config_load_error->code, "CONFIG_FILE_NOT_FOUND");
}

TEST(YamlConfigParser, MissionBoundariesAreParsed) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(configFixturePath("mission_with_boundaries.yaml"), log);

    EXPECT_EQ(config.max_steps, 100U);
    EXPECT_EQ(config.boundaries.min_x, -500.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_x, -30.0 * x_extent[cm]);
    EXPECT_TRUE(log.entries().empty());
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

TEST(YamlConfigParser, CompositionWithMissingDroneRefLoadsErrorOnDroneConfig) {
    CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("composition_invalid_drone.yaml"), log);

    ASSERT_TRUE(result.ok);
    ASSERT_EQ(result.value.drones.size(), 1U);
    ASSERT_TRUE(result.value.drones.front().config_load_error.has_value());
    EXPECT_EQ(result.value.drones.front().config_load_error->code, "CONFIG_FILE_NOT_FOUND");
    EXPECT_TRUE(log.containsCode("CONFIG_FILE_NOT_FOUND"));
}

// ---------------------------------------------------------------------------
// Instructor input tests — drone configs
// ---------------------------------------------------------------------------

TEST(YamlConfigParser, InstructorDroneSmall_ParsesCorrectly) {
    CapturingErrorLog log;
    const types::DroneConfigData config =
        io::parseDroneConfig(instructorPath("drone/drone_small.yaml"), log);

    EXPECT_EQ(config.radius, 4.0 * cm);
    EXPECT_EQ(config.max_rotate, 90.0 * horizontal_angle[deg]);
    EXPECT_EQ(config.max_advance, 30.0 * cm);
    EXPECT_EQ(config.max_elevate, 20.0 * cm);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorDroneLarge_ParsesCorrectly) {
    CapturingErrorLog log;
    const types::DroneConfigData config =
        io::parseDroneConfig(instructorPath("drone/drone_large.yaml"), log);

    EXPECT_EQ(config.radius, 7.5 * cm);
    EXPECT_EQ(config.max_rotate, 45.0 * horizontal_angle[deg]);
    EXPECT_EQ(config.max_advance, 50.0 * cm);
    EXPECT_EQ(config.max_elevate, 40.0 * cm);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

// ---------------------------------------------------------------------------
// Instructor input tests — lidar configs
// ---------------------------------------------------------------------------

TEST(YamlConfigParser, InstructorLidarShort_ParsesCorrectly) {
    CapturingErrorLog log;
    const types::LidarConfigData config =
        io::parseLidarConfig(instructorPath("lidar/lidar_short.yaml"), log);

    EXPECT_EQ(config.z_min, 20.0 * cm);
    EXPECT_EQ(config.z_max, 80.0 * cm);
    EXPECT_EQ(config.d, 2.5 * cm);
    EXPECT_EQ(config.fov_circles, 4U);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorLidarLong_ParsesCorrectly) {
    CapturingErrorLog log;
    const types::LidarConfigData config =
        io::parseLidarConfig(instructorPath("lidar/lidar_long.yaml"), log);

    EXPECT_EQ(config.z_min, 20.0 * cm);
    EXPECT_EQ(config.z_max, 150.0 * cm);
    EXPECT_EQ(config.d, 2.5 * cm);
    EXPECT_EQ(config.fov_circles, 3U);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

// ---------------------------------------------------------------------------
// Instructor input tests — simulation configs
// ---------------------------------------------------------------------------

TEST(YamlConfigParser, InstructorHouseSimulation_ParsesMapAndOffset) {
    CapturingErrorLog log;
    const types::SimulationConfigData config =
        io::parseSimulationConfig(instructorPath("simulation/house_simulation.yaml"), log);

    EXPECT_EQ(config.map_resolution, 10.0 * cm);
    EXPECT_EQ(config.map_offset.x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.map_offset.y, 0.0 * y_extent[cm]);
    EXPECT_EQ(config.map_offset.z, 150.0 * z_extent[cm]);
    ASSERT_TRUE(std::filesystem::exists(config.map_filename));
    EXPECT_EQ(config.map_filename.filename(), "scenario_house.npy");
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorSmallSimulationRoom_ParsesMap) {
    CapturingErrorLog log;
    const types::SimulationConfigData config =
        io::parseSimulationConfig(instructorPath("simulation/small_simulation_room.yaml"), log);

    EXPECT_EQ(config.map_resolution, 10.0 * cm);
    ASSERT_TRUE(std::filesystem::exists(config.map_filename));
    EXPECT_EQ(config.map_filename.filename(), "scenario_small.npy");
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorSmallSimulationOut_ParsesMap) {
    CapturingErrorLog log;
    const types::SimulationConfigData config =
        io::parseSimulationConfig(instructorPath("simulation/small_simulation_out.yaml"), log);

    EXPECT_EQ(config.map_resolution, 10.0 * cm);
    ASSERT_TRUE(std::filesystem::exists(config.map_filename));
    EXPECT_EQ(config.map_filename.filename(), "scenario_small.npy");
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorLargeSimulationRoom_ParsesMap) {
    CapturingErrorLog log;
    const types::SimulationConfigData config =
        io::parseSimulationConfig(instructorPath("simulation/large_simulation_room.yaml"), log);

    EXPECT_EQ(config.map_resolution, 10.0 * cm);
    ASSERT_TRUE(std::filesystem::exists(config.map_filename));
    EXPECT_EQ(config.map_filename.filename(), "scenario_big.npy");
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorLargeSimulationOut_ParsesMap) {
    CapturingErrorLog log;
    const types::SimulationConfigData config =
        io::parseSimulationConfig(instructorPath("simulation/large_simulation_out.yaml"), log);

    EXPECT_EQ(config.map_resolution, 10.0 * cm);
    ASSERT_TRUE(std::filesystem::exists(config.map_filename));
    EXPECT_EQ(config.map_filename.filename(), "scenario_big.npy");
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

// ---------------------------------------------------------------------------
// Instructor input tests — mission configs
// ---------------------------------------------------------------------------

TEST(YamlConfigParser, InstructorHouseMissionLower_ParsesStepsAndBoundaries) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(instructorPath("mission/house_mission_lower.yaml"), log);

    EXPECT_EQ(config.max_steps, 2000U);
    EXPECT_EQ(config.gps_resolution, 5.0 * cm);
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 1.0);
    EXPECT_EQ(config.boundaries.min_x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_x, 290.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.min_height, 0.0 * z_extent[cm]);
    EXPECT_EQ(config.boundaries.max_height, 60.0 * z_extent[cm]);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorHouseMissionFull_ParsesStepsAndBoundaries) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(instructorPath("mission/house_mission_full.yaml"), log);

    EXPECT_EQ(config.max_steps, 10000U);
    EXPECT_EQ(config.gps_resolution, 10.0 * cm);
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 1.0);
    EXPECT_EQ(config.boundaries.max_height, 150.0 * z_extent[cm]);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorSmallMissionRoom_ParsesStepsAndBoundaries) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(instructorPath("mission/small_mission_room.yaml"), log);

    EXPECT_EQ(config.max_steps, 1000U);
    EXPECT_EQ(config.gps_resolution, 5.0 * cm);
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 1.0);
    EXPECT_EQ(config.boundaries.min_y, 90.0 * y_extent[cm]);
    EXPECT_EQ(config.boundaries.max_y, 200.0 * y_extent[cm]);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorSmallMissionOut_ParsesStepsAndBoundaries) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(instructorPath("mission/small_mission_out.yaml"), log);

    EXPECT_EQ(config.max_steps, 2000U);
    EXPECT_EQ(config.gps_resolution, 5.0 * cm);
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 1.0);
    EXPECT_EQ(config.boundaries.max_x, 200.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_height, 200.0 * z_extent[cm]);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorLargeMissionRoom_ParsesStepsAndBoundaries) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(instructorPath("mission/large_mission_room.yaml"), log);

    EXPECT_EQ(config.max_steps, 500U);
    EXPECT_EQ(config.gps_resolution, 5.0 * cm);
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 1.0);
    EXPECT_EQ(config.boundaries.min_x, 90.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_x, 210.0 * x_extent[cm]);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

TEST(YamlConfigParser, InstructorLargeMissionOut_ParsesStepsAndBoundaries) {
    CapturingErrorLog log;
    const types::MissionConfigData config =
        io::parseMissionConfig(instructorPath("mission/large_mission_out.yaml"), log);

    EXPECT_EQ(config.max_steps, 10000U);
    EXPECT_EQ(config.gps_resolution, 5.0 * cm);
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 1.0);
    EXPECT_EQ(config.boundaries.max_x, 300.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_height, 300.0 * z_extent[cm]);
    EXPECT_FALSE(config.config_load_error.has_value());
    EXPECT_TRUE(log.entries().empty());
}

// ---------------------------------------------------------------------------
// Instructor input tests — full sim_compose.yaml
// ---------------------------------------------------------------------------

TEST(YamlConfigParser, InstructorSimCompose_ExpandsToSixPairsAndTwoDronesAndTwoLidars) {
    CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(instructorPath("sim_compose.yaml"), log);

    ASSERT_TRUE(result.ok);
    // house×2 + large_out×1 + large_room×1 + small_out×1 + small_room×1 = 6 aligned pairs
    EXPECT_EQ(result.value.simulations.size(), 6U);
    EXPECT_EQ(result.value.missions.size(), 6U);
    EXPECT_EQ(result.value.simulations.size(), result.value.missions.size());
    EXPECT_EQ(result.value.drones.size(), 2U);
    EXPECT_EQ(result.value.lidars.size(), 2U);
    EXPECT_TRUE(log.entries().empty());

    for (const types::SimulationConfigData& sim : result.value.simulations) {
        EXPECT_FALSE(sim.config_load_error.has_value());
        EXPECT_EQ(sim.map_resolution, 10.0 * cm);
        EXPECT_TRUE(std::filesystem::exists(sim.map_filename));
    }
    for (const types::DroneConfigData& drone : result.value.drones) {
        EXPECT_FALSE(drone.config_load_error.has_value());
    }
    for (const types::LidarConfigData& lidar : result.value.lidars) {
        EXPECT_FALSE(lidar.config_load_error.has_value());
    }
}

} // namespace drone_mapper
