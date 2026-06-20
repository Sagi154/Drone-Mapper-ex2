#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/io/RunPathHelpers.h>

#include <gtest/gtest.h>

#include <filesystem>

namespace drone_mapper {
namespace {

[[nodiscard]] types::SimulationConfigData minimalSimulation() {
    return types::SimulationConfigData{
        "data_maps/single_voxel_x2_y4_z2.npy",
        10.0 * cm,
        Position3D{},
        Position3D{},
        0.0 * horizontal_angle[deg],
    };
}

[[nodiscard]] types::MissionConfigData minimalMission() {
    return types::MissionConfigData{100, 10.0 * cm, 1.0};
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

} // namespace

TEST(SimulationRunFactory, CreateReturnsNonNullRun) {
    SimulationRunFactoryImpl factory;
    const std::filesystem::path output_path = std::filesystem::temp_directory_path() / "factory_smoke";

    std::unique_ptr<ISimulationRun> run = factory.create(
        minimalSimulation(), minimalMission(), minimalDrone(), minimalLidar(), output_path);

    ASSERT_NE(run, nullptr);
    EXPECT_NO_THROW(static_cast<void>(run->run()));

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

} // namespace drone_mapper
