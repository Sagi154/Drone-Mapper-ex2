#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/io/RunPathHelpers.h>

#include <ConfigFixtures.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <filesystem>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] types::SimulationConfigData minimalSimulation() {
    return test_support::e2eSimulation();
}

[[nodiscard]] types::MissionConfigData minimalMission() {
    // Factory layout tests only create runs; integration test uses one mission step max.
    return types::MissionConfigData{1, 10.0 * cm, 1.0};
}

[[nodiscard]] types::DroneConfigData minimalDrone() {
    return types::DroneConfigData{
        15.0 * cm,
        45.0 * horizontal_angle[deg],
        50.0 * cm,
        40.0 * cm,
    };
}

[[nodiscard]] types::LidarConfigData minimalLidar() {
    return types::LidarConfigData{
        20.0 * cm,
        120.0 * cm,
        2.5 * cm,
        5,
    };
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

} // namespace

TEST(SimulationRunFactory, CreateReturnsNonNullRun) {
    SimulationRunFactoryImpl factory;
    const std::filesystem::path output_path = std::filesystem::temp_directory_path() / "factory_smoke";

    std::unique_ptr<ISimulationRun> run = factory.create(
        minimalSimulation(), minimalMission(), minimalDrone(), minimalLidar(), output_path);

    ASSERT_NE(run, nullptr);

    std::error_code ec;
    std::filesystem::remove_all(output_path, ec);
}

TEST(SimulationRunFactory, UsesAgreedOutputLayoutForFirstRun) {
    SimulationRunFactoryImpl factory;
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "factory_layout_smoke";

    std::unique_ptr<ISimulationRun> run = factory.create(
        minimalSimulation(), minimalMission(), minimalDrone(), minimalLidar(), output_path);
    ASSERT_NE(run, nullptr);

    const std::filesystem::path expected_run_dir = io::runOutputDir(output_path, 1);
    const std::filesystem::path expected_error_log = io::runErrorLog(output_path, 1);
    const std::filesystem::path expected_output_map = io::runOutputMap(output_path, 1);

    EXPECT_TRUE(std::filesystem::exists(expected_run_dir));
    EXPECT_TRUE(std::filesystem::exists(expected_error_log));
    EXPECT_EQ(expected_output_map.filename(), "output_map.npy");
    EXPECT_EQ(expected_output_map.parent_path(), expected_run_dir);

    std::error_code ec;
    std::filesystem::remove_all(output_path, ec);
}

TEST(SimulationRun, FactoryLoadsHiddenMapFromDisk) {
    SimulationRunFactoryImpl factory;
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "factory_hidden_map_load";

    std::unique_ptr<ISimulationRun> run = factory.create(
        minimalSimulation(), minimalMission(), minimalDrone(), minimalLidar(), output_path);
    ASSERT_NE(run, nullptr);

    const types::SimulationResult result = run->run();
    EXPECT_EQ(result.simulation_config.map_filename, test_support::goldenMapPath());
    EXPECT_FALSE(result.mission_results.empty());

    const std::vector<std::string> lines = readAllLines(io::runErrorLog(output_path, 1));
    EXPECT_TRUE(lines.empty());

    std::error_code ec;
    std::filesystem::remove_all(output_path, ec);
}

TEST(SimulationRun, Factory_EndToEnd_WritesOutputMapAndScores) {
    SimulationRunFactoryImpl factory;
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "factory_e2e_smoke";

    std::unique_ptr<ISimulationRun> run = factory.create(
        minimalSimulation(), minimalMission(), minimalDrone(), minimalLidar(), output_path);
    ASSERT_NE(run, nullptr);

    const types::SimulationResult result = run->run();

    const std::filesystem::path output_map = io::runOutputMap(output_path, 1);
    EXPECT_TRUE(std::filesystem::exists(output_map));
    EXPECT_GE(result.mission_score, 0.0);
    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_TRUE(result.mission_results.front().status == types::MissionRunStatus::Completed ||
                result.mission_results.front().status == types::MissionRunStatus::MaxSteps);

    const std::vector<std::string> lines = readAllLines(io::runErrorLog(output_path, 1));
    EXPECT_TRUE(lines.empty());

    std::error_code ec;
    std::filesystem::remove_all(output_path, ec);
}

TEST(SimulationRun, FactoryLogsErrorWhenMapFileMissing) {
    SimulationRunFactoryImpl factory;
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "factory_missing_map";

    types::SimulationConfigData simulation = minimalSimulation();
    simulation.map_filename = "data_maps/does_not_exist.npy";

    std::unique_ptr<ISimulationRun> run =
        factory.create(simulation, minimalMission(), minimalDrone(), minimalLidar(), output_path);
    ASSERT_NE(run, nullptr);

    const types::SimulationResult result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Error);
    ASSERT_EQ(result.mission_results.front().errors.size(), 1U);
    EXPECT_EQ(result.mission_results.front().errors.front().code, "MAP_FILE_NOT_FOUND");

    const std::vector<std::string> lines = readAllLines(io::runErrorLog(output_path, 1));
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_NE(lines.front().find("MAP_FILE_NOT_FOUND"), std::string::npos);
    EXPECT_NE(lines.front().find("does_not_exist.npy"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove_all(output_path, ec);
}

} // namespace drone_mapper
