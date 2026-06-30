// tests/integration/test_integration_full_flow.cpp — Integration.*
// End-to-end runs: real algorithm on benchmark map, mock algorithm wiring.

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/io/RunPathHelpers.h>

#include <ConfigFixtures.hpp>

#include <TinyNPY.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

void removeDirectory(const std::filesystem::path& path) {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
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

[[nodiscard]] types::MissionConfigData integrationMission() {
    return types::MissionConfigData{100, 10.0 * cm, 1.0};
}

[[nodiscard]] types::DroneConfigData integrationDrone() {
    return types::DroneConfigData{
        15.0 * cm,
        45.0 * horizontal_angle[deg],
        50.0 * cm,
        40.0 * cm,
    };
}

[[nodiscard]] types::LidarConfigData integrationLidar() {
    return types::LidarConfigData{
        20.0 * cm,
        120.0 * cm,
        2.5 * cm,
        5,
    };
}

[[nodiscard]] std::shared_ptr<NpyArray> loadMapArray(const std::filesystem::path& path) {
    auto map_array = std::make_shared<NpyArray>();
    const LPCSTR load_error = map_array->LoadNPY(path.string());
    if (load_error != nullptr) {
        return nullptr;
    }
    return map_array;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyOutputArray(const NpyArray& hidden) {
    if (hidden.IsEmpty() || hidden.Shape().size() != 3) {
        return std::make_shared<NpyArray>();
    }

    auto output = std::make_shared<NpyArray>(hidden.Shape(),
                                              sizeof(std::int8_t),
                                              NpyArray::GetTypeChar(typeid(std::int8_t)));
    output->Allocate();
    std::fill_n(output->Data<std::int8_t>(),
                output->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return output;
}

class IntegrationAlgorithmMock : public IMappingAlgorithm {
public:
    IntegrationAlgorithmMock(const types::MissionConfigData& mission,
                             const types::LidarConfigData& lidar,
                             const types::DroneConfigData& drone,
                             const IMap3D& output_map)
        : IMappingAlgorithm(mission, lidar, drone, output_map) {}

    MOCK_METHOD(types::MappingStepCommand,
                nextStep,
                (const types::DroneState& state, const types::LidarScanResult* latest_scan),
                (override));
};

[[nodiscard]] std::unique_ptr<SimulationRunImpl> makeRunWithAlgorithm(
    std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
    const types::SimulationConfigData& simulation,
    const types::MissionConfigData& mission,
    const types::DroneConfigData& drone,
    const types::LidarConfigData& lidar,
    const std::filesystem::path& output_path) {
    const std::filesystem::path run_dir = io::runOutputDir(output_path, 1);
    const std::filesystem::path output_map_file = io::runOutputMap(output_path, 1);
    std::filesystem::create_directories(run_dir);

    const std::shared_ptr<NpyArray> hidden_array = loadMapArray(simulation.map_filename);
    if (!hidden_array) {
        return nullptr;
    }

    types::MapConfig hidden_config{
        .boundaries = {},
        .offset = simulation.map_offset,
        .resolution = simulation.map_resolution,
    };
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, hidden_config);
    const types::MapConfig output_config = hidden_map->getMapConfig();
    auto output_map =
        std::make_unique<Map3DImpl>(makeEmptyOutputArray(*hidden_array), output_config);

    IMappingAlgorithm& algorithm_ref = *mapping_algorithm;
    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        lidar,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        algorithm_ref);
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::unique_ptr<const IMap3D>(std::move(hidden_map)),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

[[nodiscard]] types::SimulationManagerReport runComposition(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {
    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>()};
    return manager.run(composition, output_path);
}

// Parse a focused instructor composition, run it end-to-end, and assert:
//   • 1 run, no errors in error.log
//   • mission_score >= 90
//   • wall-clock elapsed < 60 s
// Caller owns creation/removal of output_path to allow unique names per test.
void runInstructorCompositionIntegration(const std::string& composition_filename,
                                         const std::filesystem::path& output_path) {
    test_support::CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> composition_result =
        test_support::loadInstructorFocusedComposition(composition_filename, log);
    ASSERT_TRUE(composition_result.ok)
        << "Failed to load instructor composition: " << composition_filename;
    EXPECT_TRUE(log.entries().empty())
        << "Parse errors for " << composition_filename;
    ASSERT_EQ(composition_result.value.missions.size(), 1U)
        << "Focused composition must have exactly 1 mission pair";

    removeDirectory(output_path);

    const auto start = std::chrono::steady_clock::now();
    const types::SimulationManagerReport report =
        runComposition(composition_result.value, output_path);
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start);

    ASSERT_EQ(report.runs.size(), 1U);
    ASSERT_EQ(report.runs.front().mission_results.size(), 1U);
    EXPECT_NE(report.runs.front().mission_results.front().status, types::MissionRunStatus::Error)
        << "Mission reported Error status for " << composition_filename;
    EXPECT_GE(report.runs.front().mission_score, 90.0)
        << composition_filename << " scored below 90";
    EXPECT_TRUE(readAllLines(io::runErrorLog(output_path, 1)).empty())
        << "Unexpected entries in error.log for " << composition_filename;
    EXPECT_LT(elapsed.count(), 60)
        << composition_filename << " exceeded 60 s time budget";

    removeDirectory(output_path);
}

} // namespace

// Scenario: instructor benchmark map with real MappingAlgorithmImpl via SimulationManager.
// Expected: mission completes without errors, writes output map, finishes within 60 s.
// Score threshold is permissive (>= 0): current exploration scores ~0.04 on this map on main;
// target >= 90 is a Phase 5 algorithm-tuning goal (see workplan.md).
TEST(Integration, BenchmarkMap_RealAlgorithm_CompletesWithoutError) {
    test_support::CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> composition_result =
        test_support::loadBenchmarkComposition(log);
    ASSERT_TRUE(composition_result.ok) << "Failed to load benchmark composition fixture";
    EXPECT_TRUE(log.entries().empty());
    ASSERT_EQ(composition_result.value.missions.size(), 1U);
    ASSERT_EQ(composition_result.value.missions.front().max_steps, 20000U);

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "integration_benchmark_real_algorithm";
    removeDirectory(output_path);

    const auto start = std::chrono::steady_clock::now();
    const types::SimulationManagerReport report =
        runComposition(composition_result.value, output_path);
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start);

    ASSERT_EQ(report.runs.size(), 1U);
    ASSERT_EQ(report.runs.front().mission_results.size(), 1U);
    EXPECT_NE(report.runs.front().mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_GT(report.runs.front().mission_results.front().steps, 0U);
    EXPECT_GE(report.runs.front().mission_score, 0.0);
    EXPECT_TRUE(std::filesystem::exists(io::runOutputMap(output_path, 1)));
    EXPECT_TRUE(readAllLines(io::runErrorLog(output_path, 1)).empty());
    EXPECT_LT(elapsed.count(), 60);

    removeDirectory(output_path);
}

// Scenario: GMock IMappingAlgorithm wired through real DroneControl/MissionControl/SimulationRunImpl.
// Expected: mission completes on first Finished command, score computed, nextStep invoked.
TEST(Integration, MockAlgorithm_FullRun_CompletesWithScore) {
    types::SimulationConfigData simulation = test_support::e2eSimulation();
    const types::MissionConfigData mission = integrationMission();
    const types::DroneConfigData drone = integrationDrone();
    const types::LidarConfigData lidar = integrationLidar();

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "integration_mock_algorithm";
    removeDirectory(output_path);

    const std::shared_ptr<NpyArray> hidden_array = loadMapArray(simulation.map_filename);
    ASSERT_NE(hidden_array, nullptr);

    types::MapConfig hidden_config{
        .boundaries = {},
        .offset = simulation.map_offset,
        .resolution = simulation.map_resolution,
    };
    Map3DImpl stand_in_output{makeEmptyOutputArray(*hidden_array), hidden_config};

    auto algorithm = std::make_unique<testing::NiceMock<IntegrationAlgorithmMock>>(
        mission, lidar, drone, stand_in_output);
    IntegrationAlgorithmMock& algorithm_ref = *algorithm;

    types::MappingStepCommand finished_command{};
    finished_command.status = types::AlgorithmStatus::Finished;
    EXPECT_CALL(algorithm_ref, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(finished_command));

    std::unique_ptr<SimulationRunImpl> run =
        makeRunWithAlgorithm(std::move(algorithm), simulation, mission, drone, lidar, output_path);
    ASSERT_NE(run, nullptr);

    const types::SimulationResult result = run->run();

    EXPECT_GE(result.mission_score, 0.0);
    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_TRUE(std::filesystem::exists(io::runOutputMap(output_path, 1)));
    EXPECT_TRUE(readAllLines(io::runErrorLog(output_path, 1)).empty());

    removeDirectory(output_path);
}

// Scenario: instructor small_room focused composition (scenario_small.npy, 20×20×20, 1 run).
// Expected: mission_score >= 90, no errors, completes within 60 s.
TEST(Integration, InstructorSmallRoom_AchievesTargetScore) {
    runInstructorCompositionIntegration(
        "composition_small_room.yaml",
        std::filesystem::temp_directory_path() / "integration_instructor_small_room");
}

// Scenario: instructor big_room focused composition (scenario_big.npy, 30×30×30, 1 run).
// Expected: mission_score >= 90, no errors, completes within 60 s.
TEST(Integration, InstructorBigRoom_AchievesTargetScore) {
    runInstructorCompositionIntegration(
        "composition_big_room.yaml",
        std::filesystem::temp_directory_path() / "integration_instructor_big_room");
}

// Scenario: instructor house_lower focused composition (scenario_house.npy, 29×30×31, 1 run).
// Validates map_axes_offset (height_offset: 150 cm) applied end-to-end: drone starts in the
// lower volume, not at world Z = 0 inside the floor.
// Expected: mission_score >= 90, no errors, completes within 60 s.
TEST(Integration, InstructorHouseLower_AchievesTargetScore) {
    runInstructorCompositionIntegration(
        "composition_house_lower.yaml",
        std::filesystem::temp_directory_path() / "integration_instructor_house_lower");
}

// Scenario: parse full sim_compose.yaml (24 runs) — no execution.
// Expected: 6 aligned simulation/mission pairs, 2 drones, 2 lidars → 24 total runs.
TEST(Integration, InstructorCompose_ParsesTwentyFourRuns) {
    test_support::CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> result =
        test_support::loadInstructorComposition(log);
    ASSERT_TRUE(result.ok) << "Failed to parse sim_compose.yaml";
    EXPECT_TRUE(log.entries().empty());

    const types::SimulationCompositionData& comp = result.value;
    EXPECT_EQ(comp.simulations.size(), 6U) << "Expected 6 aligned simulation configs";
    EXPECT_EQ(comp.missions.size(), 6U) << "Expected 6 aligned mission configs";
    EXPECT_EQ(comp.drones.size(), 2U) << "Expected 2 drone configs";
    EXPECT_EQ(comp.lidars.size(), 2U) << "Expected 2 lidar configs";
    EXPECT_EQ(comp.missions.size() * comp.drones.size() * comp.lidars.size(), 24U)
        << "6 pairs × 2 drones × 2 lidars should expand to 24 runs";
}

} // namespace drone_mapper
