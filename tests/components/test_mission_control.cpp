// test_mission_control.cpp — MissionControl.*
// Scenarios: step loop termination, max_steps, error propagation, and output map save.

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>

#include <TinyNPY.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>

namespace drone_mapper {
namespace {

class DroneControlMock : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

class HiddenMapMock final : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D&), (const, override));
};

class OutputMapMock final : public IMutableMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D&), (const, override));
    MOCK_METHOD(void, set, (const Position3D&, types::VoxelOccupancy), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path&), (const, override));
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

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyOutputMap() {
    auto map = std::make_shared<NpyArray>(
        NpyArray::shape_t{4, 4, 4},
        sizeof(std::int8_t),
        NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(),
                map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] types::MissionConfigData makeMission(std::size_t max_steps) {
    return types::MissionConfigData{max_steps, 10.0 * cm, 1.0};
}

[[nodiscard]] types::DroneConfigData defaultDrone() {
    return types::DroneConfigData{
        5.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
}

struct MissionControlFixture {
    HiddenMapMock hidden_map;
    Map3DImpl output_map{makeEmptyOutputMap(), makeUnitCubeConfig()};
    testing::StrictMock<DroneControlMock> drone_control;
};

[[nodiscard]] MissionControlImpl makeControl(MissionControlFixture& fixture,
                                             std::size_t max_steps,
                                             const std::filesystem::path& output_file) {
    return MissionControlImpl{
        makeMission(max_steps),
        defaultDrone(),
        fixture.hidden_map,
        fixture.output_map,
        fixture.drone_control,
        output_file,
    };
}

} // namespace

// What: drone completes after two Continue steps and one Completed step.
// Expected: MissionRunStatus::Completed with steps == 3.
TEST(MissionControlTest, ReturnsCompletedWhenDroneFinishes) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_completed.npy";

    MissionControlImpl control = makeControl(fixture, 10, output_file);

    testing::InSequence sequence;
    EXPECT_CALL(fixture.drone_control, step())
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}))
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}))
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Completed, {}}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3U);
    EXPECT_TRUE(result.errors.empty());
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: drone never completes within max_steps.
// Expected: MissionRunStatus::MaxSteps with steps == max_steps.
TEST(MissionControlTest, ReturnsMaxStepsWhenLimitReached) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_max_steps.npy";

    MissionControlImpl control = makeControl(fixture, 5, output_file);

    EXPECT_CALL(fixture.drone_control, step())
        .Times(5)
        .WillRepeatedly(testing::Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 5U);
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: first drone step returns Error.
// Expected: MissionRunStatus::Error, one ErrorRef, steps == 1.
TEST(MissionControlTest, ReturnsErrorWhenDroneStepFails) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_step_error.npy";

    MissionControlImpl control = makeControl(fixture, 10, output_file);

    EXPECT_CALL(fixture.drone_control, step())
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Error, "boom"}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1U);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "DRONE_STEP_FAILED");
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: drone step fails with a specific message.
// Expected: errors[0].message matches the step message.
TEST(MissionControlTest, PropagatesDroneStepErrorMessage) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_error_message.npy";

    MissionControlImpl control = makeControl(fixture, 10, output_file);

    EXPECT_CALL(fixture.drone_control, step())
        .WillOnce(testing::Return(
            types::DroneStepResult{types::DroneStepStatus::Error, "Movement command exceeds drone limits."}));

    const types::MissionRunResult result = control.runMission();
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().message, "Movement command exceeds drone limits.");
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: drone step fails on the first iteration.
// Expected: step() is invoked exactly once before returning Error.
TEST(MissionControlTest, StopsLoopImmediatelyOnError) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_stop_on_error.npy";

    MissionControlImpl control = makeControl(fixture, 100, output_file);

    EXPECT_CALL(fixture.drone_control, step())
        .Times(1)
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Error, "fail"}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: mission completes successfully.
// Expected: output map file is written to the configured path.
TEST(MissionControlTest, SavesOutputMapOnCompletedMission) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_save_completed.npy";

    MissionControlImpl control = makeControl(fixture, 5, output_file);

    EXPECT_CALL(fixture.drone_control, step())
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Completed, {}}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_TRUE(std::filesystem::exists(output_file));
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: mission hits max_steps without completion.
// Expected: output map is still saved.
TEST(MissionControlTest, SavesOutputMapOnMaxSteps) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_save_max_steps.npy";

    MissionControlImpl control = makeControl(fixture, 2, output_file);

    EXPECT_CALL(fixture.drone_control, step())
        .Times(2)
        .WillRepeatedly(testing::Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_TRUE(std::filesystem::exists(output_file));
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: drone step fails mid-mission.
// Expected: partial output map is still saved before returning Error.
TEST(MissionControlTest, SavesOutputMapOnDroneError) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_save_on_error.npy";

    MissionControlImpl control = makeControl(fixture, 10, output_file);

    testing::InSequence sequence;
    EXPECT_CALL(fixture.drone_control, step())
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}))
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Error, "collision"}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_TRUE(std::filesystem::exists(output_file));
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

// What: output map save throws.
// Expected: MissionRunStatus::Error with MAP_SAVE_FAILED ErrorRef.
TEST(MissionControlTest, ReturnsErrorWhenMapSaveFails) {
    HiddenMapMock hidden_map;
    OutputMapMock output_map;
    testing::StrictMock<DroneControlMock> drone_control;

    ON_CALL(output_map, getMapConfig()).WillByDefault(testing::Return(makeUnitCubeConfig()));
    ON_CALL(output_map, isInBounds(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(output_map, atVoxel(testing::_))
        .WillByDefault(testing::Return(types::VoxelOccupancy::Unmapped));

    MissionControlImpl control{
        makeMission(5),
        defaultDrone(),
        hidden_map,
        output_map,
        drone_control,
        std::filesystem::temp_directory_path() / "mission_control_save_fail.npy",
    };

    EXPECT_CALL(drone_control, step())
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Completed, {}}));
    EXPECT_CALL(output_map, save(testing::_))
        .WillOnce(testing::Throw(std::runtime_error("disk full")));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "MAP_SAVE_FAILED");
    EXPECT_NE(result.errors.front().message.find("disk full"), std::string::npos);
}

// What: max_steps is 1 and the single step returns Continue.
// Expected: MaxSteps with steps == 1.
TEST(MissionControlTest, RespectsMaxStepsOne) {
    MissionControlFixture fixture;
    const std::filesystem::path output_file =
        std::filesystem::temp_directory_path() / "mission_control_max_one.npy";

    MissionControlImpl control = makeControl(fixture, 1, output_file);

    EXPECT_CALL(fixture.drone_control, step())
        .Times(1)
        .WillOnce(testing::Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}));

    const types::MissionRunResult result = control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 1U);
    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

} // namespace drone_mapper
