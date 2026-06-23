// test_mission_control.cpp — MissionControl.*
// Scenarios: completed mission, max-steps exhaustion, drone step error, output map save.

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>

#include <TinyNPY.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>

namespace drone_mapper {
namespace {

class DroneControlMock : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

class HiddenMapMock : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D&), (const, override));
};

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

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyMutableMap() {
    auto map = std::make_shared<NpyArray>(
        NpyArray::shape_t{3, 3, 3},
        sizeof(std::int8_t),
        NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(),
                map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] MissionControlImpl makeMissionControl(const IMap3D& hidden_map,
                                                    IDroneControl& drone_control,
                                                    IMutableMap3D& output_map,
                                                    const std::filesystem::path& output_file,
                                                    std::size_t max_steps) {
    const types::MissionConfigData mission{max_steps, 10.0 * cm, 1.0};
    const types::DroneConfigData drone{
        15.0 * cm,
        45.0 * horizontal_angle[deg],
        50.0 * cm,
        40.0 * cm,
    };
    return MissionControlImpl{mission, drone, hidden_map, output_map, drone_control, output_file};
}

} // namespace

TEST(MissionControlTest, CompletesWhenDroneSignalsCompleted) {
    testing::StrictMock<DroneControlMock> drone_control;
    testing::NiceMock<HiddenMapMock> hidden_map;
    Map3DImpl output_map{makeEmptyMutableMap(), makeUnitCubeConfig()};
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_completed.npy";

    EXPECT_CALL(drone_control, step())
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Completed, ""}));

    MissionControlImpl mission_control =
        makeMissionControl(hidden_map, drone_control, output_map, output_file, /*max_steps=*/10);
    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1U);
    EXPECT_TRUE(result.errors.empty());
    EXPECT_TRUE(std::filesystem::exists(output_file));

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

TEST(MissionControlTest, StopsAtMaxStepsWhenDroneKeepsContinuing) {
    testing::StrictMock<DroneControlMock> drone_control;
    testing::NiceMock<HiddenMapMock> hidden_map;
    Map3DImpl output_map{makeEmptyMutableMap(), makeUnitCubeConfig()};
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_max_steps.npy";
    constexpr std::size_t kMaxSteps = 3;

    EXPECT_CALL(drone_control, step())
        .Times(static_cast<int>(kMaxSteps))
        .WillRepeatedly(testing::Return(types::DroneStepResult{types::DroneStepStatus::Continue, ""}));

    MissionControlImpl mission_control =
        makeMissionControl(hidden_map, drone_control, output_map, output_file, kMaxSteps);
    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, kMaxSteps);
    EXPECT_TRUE(result.errors.empty());
    EXPECT_TRUE(std::filesystem::exists(output_file));

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

TEST(MissionControlTest, ReturnsErrorAndSavesMapWhenDroneStepFails) {
    testing::StrictMock<DroneControlMock> drone_control;
    testing::NiceMock<HiddenMapMock> hidden_map;
    Map3DImpl output_map{makeEmptyMutableMap(), makeUnitCubeConfig()};
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_error.npy";

    EXPECT_CALL(drone_control, step())
        .WillOnce(testing::Return(
            types::DroneStepResult{types::DroneStepStatus::Error, "collision detected"}));

    MissionControlImpl mission_control =
        makeMissionControl(hidden_map, drone_control, output_map, output_file, /*max_steps=*/10);
    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1U);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "DRONE_STEP_ERROR");
    EXPECT_EQ(result.errors.front().message, "collision detected");
    EXPECT_TRUE(std::filesystem::exists(output_file));

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

TEST(MissionControlTest, SavesOutputMapEvenWhenMaxStepsIsZero) {
    testing::StrictMock<DroneControlMock> drone_control;
    testing::NiceMock<HiddenMapMock> hidden_map;
    Map3DImpl output_map{makeEmptyMutableMap(), makeUnitCubeConfig()};
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_zero_steps.npy";

    EXPECT_CALL(drone_control, step()).Times(0);

    MissionControlImpl mission_control =
        makeMissionControl(hidden_map, drone_control, output_map, output_file, /*max_steps=*/0);
    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 0U);
    EXPECT_TRUE(std::filesystem::exists(output_file));

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

} // namespace drone_mapper
