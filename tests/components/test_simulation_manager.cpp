// test_simulation_manager.cpp — SimulationManager.*

#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <memory>
#include <string>

namespace drone_mapper {
namespace {

class SimulationRunMock : public ISimulationRun {
public:
    MOCK_METHOD(types::SimulationResult, run, (), (override));
};

class SimulationRunFactoryMock : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>,
                create,
                (const types::SimulationConfigData&,
                 const types::MissionConfigData&,
                 const types::DroneConfigData&,
                 const types::LidarConfigData&,
                 const std::filesystem::path&),
                (override));
};

[[nodiscard]] types::SimulationConfigData makeSimulation(std::size_t index) {
    types::SimulationConfigData simulation{};
    simulation.map_filename = "data_maps/test_" + std::to_string(index) + ".npy";
    simulation.map_resolution = 10.0 * cm;
    simulation.map_offset = Position3D{};
    simulation.initial_drone_position = Position3D{};
    simulation.initial_angle = 0.0 * horizontal_angle[deg];
    return simulation;
}

[[nodiscard]] types::MissionConfigData makeMission() {
    return types::MissionConfigData{
        .max_steps = 10,
        .gps_resolution = 10.0 * cm,
        .output_mapping_resolution_factor = 1.0,
    };
}

[[nodiscard]] types::DroneConfigData makeDrone() {
    return types::DroneConfigData{
        .radius = 5.0 * cm,
        .max_rotate = 45.0 * horizontal_angle[deg],
        .max_advance = 50.0 * cm,
        .max_elevate = 40.0 * cm,
    };
}

[[nodiscard]] types::LidarConfigData makeLidar() {
    return types::LidarConfigData{
        .z_min = 0.0 * cm,
        .z_max = 100.0 * cm,
        .d = 5.0 * cm,
        .fov_circles = 4,
    };
}

[[nodiscard]] types::SimulationCompositionData makeComposition(std::size_t simulation_count) {
    types::SimulationCompositionData composition{};
    composition.composition_file = "composition.yaml";
    for (std::size_t index = 0; index < simulation_count; ++index) {
        composition.simulations.push_back(makeSimulation(index));
    }
    composition.missions.push_back(makeMission());
    composition.drones.push_back(makeDrone());
    composition.lidars.push_back(makeLidar());
    return composition;
}

[[nodiscard]] types::SimulationResult makeResult(double mission_score) {
    types::SimulationResult result{};
    result.simulation_config = makeSimulation(0);
    result.mission_config = makeMission();
    result.mission_score = mission_score;
    result.mission_results.push_back(types::MissionRunResult{
        mission_score < 0.0 ? types::MissionRunStatus::Error : types::MissionRunStatus::Completed,
        1,
        {},
    });
    return result;
}

[[nodiscard]] std::unique_ptr<SimulationRunMock> makeRunMock(types::SimulationResult result) {
    auto run = std::make_unique<SimulationRunMock>();
    EXPECT_CALL(*run, run()).WillOnce(testing::Return(std::move(result)));
    return run;
}

void removeDirectory(const std::filesystem::path& path) {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
}

} // namespace

TEST(SimulationManagerTest, Run_PopulatesReportMetadata) {
    auto factory = std::make_unique<SimulationRunFactoryMock>();
    EXPECT_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillOnce(testing::Return(testing::ByMove(makeRunMock(makeResult(80.0)))))
        .WillOnce(testing::Return(testing::ByMove(makeRunMock(makeResult(90.0)))));

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "simulation_manager_report_metadata";
    removeDirectory(output_path);

    SimulationManager manager{std::move(factory)};
    const types::SimulationManagerReport report =
        manager.run(makeComposition(2), output_path);

    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_EQ(report.metric, "maps_comparison_score");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);
    ASSERT_EQ(report.runs.size(), 2U);

    removeDirectory(output_path);
}

TEST(SimulationManagerTest, Run_WritesSimulationOutputYaml) {
    auto factory = std::make_unique<SimulationRunFactoryMock>();
    EXPECT_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillOnce(testing::Return(testing::ByMove(makeRunMock(makeResult(75.0)))));

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "simulation_manager_yaml_write";
    removeDirectory(output_path);

    SimulationManager manager{std::move(factory)};
    static_cast<void>(manager.run(makeComposition(1), output_path));

    const std::filesystem::path yaml_path = output_path / "simulation_output.yaml";
    ASSERT_TRUE(std::filesystem::exists(yaml_path));

    const YAML::Node root = YAML::LoadFile(yaml_path.string());
    ASSERT_TRUE(root["score_report"]);
    EXPECT_EQ(root["score_report"]["metric"].as<std::string>(), "maps_comparison_score");
    ASSERT_TRUE(root["score_report"]["runs"]);
    EXPECT_EQ(root["score_report"]["runs"].size(), 1U);

    removeDirectory(output_path);
}

TEST(SimulationManagerTest, Run_ContinuesAfterFailedScenario) {
    auto factory = std::make_unique<SimulationRunFactoryMock>();
    EXPECT_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillOnce(testing::Return(testing::ByMove(makeRunMock(makeResult(-1.0)))))
        .WillOnce(testing::Return(testing::ByMove(makeRunMock(makeResult(88.0)))));

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "simulation_manager_continue_after_fail";
    removeDirectory(output_path);

    SimulationManager manager{std::move(factory)};
    const types::SimulationManagerReport report =
        manager.run(makeComposition(2), output_path);

    ASSERT_EQ(report.runs.size(), 2U);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 88.0);

    const YAML::Node root = YAML::LoadFile((output_path / "simulation_output.yaml").string());
    ASSERT_EQ(root["score_report"]["runs"].size(), 2U);

    removeDirectory(output_path);
}

TEST(SimulationManagerTest, Run_EmitsConfigIndicesAndRunId) {
    auto factory = std::make_unique<SimulationRunFactoryMock>();
    EXPECT_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillOnce(testing::Return(testing::ByMove(makeRunMock(makeResult(70.0)))))
        .WillOnce(testing::Return(testing::ByMove(makeRunMock(makeResult(71.0)))));

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "simulation_manager_config_indices";
    removeDirectory(output_path);

    SimulationManager manager{std::move(factory)};
    static_cast<void>(manager.run(makeComposition(2), output_path));

    const YAML::Node root = YAML::LoadFile((output_path / "simulation_output.yaml").string());
    const YAML::Node first_run = root["score_report"]["runs"][0];
    EXPECT_EQ(first_run["run_id"].as<int>(), 1);
    EXPECT_EQ(first_run["config_indices"]["simulation"].as<std::size_t>(), 0U);
    EXPECT_EQ(first_run["config_indices"]["mission"].as<std::size_t>(), 0U);
    EXPECT_EQ(first_run["config_indices"]["drone"].as<std::size_t>(), 0U);
    EXPECT_EQ(first_run["config_indices"]["lidar"].as<std::size_t>(), 0U);

    const YAML::Node second_run = root["score_report"]["runs"][1];
    EXPECT_EQ(second_run["run_id"].as<int>(), 2);
    EXPECT_EQ(second_run["config_indices"]["simulation"].as<std::size_t>(), 1U);

    removeDirectory(output_path);
}

} // namespace drone_mapper
