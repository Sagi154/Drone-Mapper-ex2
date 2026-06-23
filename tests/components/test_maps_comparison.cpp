// test_maps_comparison.cpp — MapsComparison.*
// Scenarios: identical/near/distinct scoring, union-of-cells semantics, multi-target
// compare, and maps_comparison CLI stdout/stderr contract.

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <TinyNPY.h>

#include <fstream>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] types::MapConfig makeThreeByThreeConfig() {
    types::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 20.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 20.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 20.0 * z_extent[cm];
    return config;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyGrid(const NpyArray::shape_t& shape) {
    auto map = std::make_shared<NpyArray>(shape, sizeof(std::int8_t),
                                            NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(), map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] std::unique_ptr<Map3DImpl> makeMutableMap(const NpyArray::shape_t& shape,
                                                        const types::MapConfig& config) {
    return std::make_unique<Map3DImpl>(makeEmptyGrid(shape), config);
}

[[nodiscard]] Position3D cellCenter(int x, int y, int z) {
    return Position3D{
        static_cast<double>(x * 10) * x_extent[cm],
        static_cast<double>(y * 10) * y_extent[cm],
        static_cast<double>(z * 10) * z_extent[cm],
    };
}

void fillKnownGrid(Map3DImpl& map, types::VoxelOccupancy value) {
    for (int x = 0; x < 3; ++x) {
        for (int y = 0; y < 3; ++y) {
            for (int z = 0; z < 3; ++z) {
                map.set(cellCenter(x, y, z), value);
            }
        }
    }
}

[[nodiscard]] types::MapConfig makeGoldenMapConfig() {
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

[[nodiscard]] std::filesystem::path tempMapPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / name;
}

#ifdef MAPS_COMPARISON_EXE
struct ProcessOutput {
    int exit_code = -1;
    std::string stdout_text;
    std::string stderr_text;
};

[[nodiscard]] ProcessOutput runMapsComparison(const std::vector<std::string>& args) {
    std::ostringstream command;
    command << '"' << MAPS_COMPARISON_EXE << '"';
    for (const std::string& arg : args) {
        command << " \"" << arg << '"';
    }
    command << " 2>\"" << tempMapPath("maps_comparison_stderr.txt").string() << '"';

    ProcessOutput output{};
    std::array<char, 256> buffer{};
    FILE* pipe = _popen(command.str().c_str(), "r");
    if (pipe == nullptr) {
        return output;
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        output.stdout_text += buffer.data();
    }
    output.exit_code = _pclose(pipe);

    const std::filesystem::path stderr_path = tempMapPath("maps_comparison_stderr.txt");
    if (std::filesystem::exists(stderr_path)) {
        std::ifstream stderr_file(stderr_path);
        output.stderr_text.assign(std::istreambuf_iterator<char>(stderr_file),
                                  std::istreambuf_iterator<char>());
        std::error_code ec;
        std::filesystem::remove(stderr_path, ec);
    }

    return output;
}
#endif

} // namespace

TEST(MapsComparison, IdenticalMapsScore100) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    fillKnownGrid(*reference, types::VoxelOccupancy::Empty);
    fillKnownGrid(*target, types::VoxelOccupancy::Empty);
    reference->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);
    target->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

TEST(MapsComparison, EmptyUnionScore100) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

TEST(MapsComparison, SingleMismatchReducesScore) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    fillKnownGrid(*reference, types::VoxelOccupancy::Occupied);
    fillKnownGrid(*target, types::VoxelOccupancy::Occupied);
    target->set(cellCenter(0, 0, 0), types::VoxelOccupancy::Empty);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_LT(scores.front(), 100.0);
    EXPECT_GT(scores.front(), 95.0);
}

TEST(MapsComparison, NearIdenticalNotExactly100) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    fillKnownGrid(*reference, types::VoxelOccupancy::Empty);
    fillKnownGrid(*target, types::VoxelOccupancy::Empty);
    target->set(cellCenter(2, 2, 2), types::VoxelOccupancy::Occupied);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_LT(scores.front(), 100.0);
    EXPECT_GT(scores.front(), 90.0);
}

TEST(MapsComparison, DistinctMapsScoreNearZero) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    fillKnownGrid(*reference, types::VoxelOccupancy::Occupied);
    fillKnownGrid(*target, types::VoxelOccupancy::Empty);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_NEAR(scores.front(), 0.0, 1e-6);
}

TEST(MapsComparison, IntermediateMismatchProducesGradient) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto mostly_wrong = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto slightly_wrong = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);

    fillKnownGrid(*reference, types::VoxelOccupancy::Occupied);
    fillKnownGrid(*mostly_wrong, types::VoxelOccupancy::Empty);
    fillKnownGrid(*slightly_wrong, types::VoxelOccupancy::Occupied);
    slightly_wrong->set(cellCenter(0, 0, 0), types::VoxelOccupancy::Empty);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {mostly_wrong.get(), slightly_wrong.get()});
    ASSERT_EQ(scores.size(), 2U);
    EXPECT_LT(scores[0], scores[1]);
    EXPECT_GT(scores[1], 95.0);
    EXPECT_LT(scores[0], 5.0);
}

TEST(MapsComparison, TargetOnlyEmptyInUnmappedRegionCountsCorrect) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    reference->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);
    target->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);
    target->set(cellCenter(0, 0, 0), types::VoxelOccupancy::Empty);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

TEST(MapsComparison, TargetOnlyOccupiedInUnmappedRegionCountsWrong) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    reference->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);
    target->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);
    target->set(cellCenter(0, 0, 0), types::VoxelOccupancy::Occupied);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_LT(scores.front(), 100.0);
}

TEST(MapsComparison, ReferenceUnmappedCellsAreSkipped) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    reference->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);
    target->set(cellCenter(1, 1, 1), types::VoxelOccupancy::Occupied);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

TEST(MapsComparison, OutOfBoundsCellsAreSkipped) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    fillKnownGrid(*reference, types::VoxelOccupancy::Empty);
    fillKnownGrid(*target, types::VoxelOccupancy::Empty);

    const Position3D outside{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    EXPECT_EQ(reference->atVoxel(outside), types::VoxelOccupancy::OutOfBounds);
    EXPECT_EQ(target->atVoxel(outside), types::VoxelOccupancy::OutOfBounds);

    const std::vector<double> scores =
        MapsComparison::compare(*reference, {target.get()});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

TEST(MapsComparison, MultipleTargetsReturnParallelScores) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto identical = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto distinct = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    fillKnownGrid(*reference, types::VoxelOccupancy::Occupied);
    fillKnownGrid(*identical, types::VoxelOccupancy::Occupied);
    fillKnownGrid(*distinct, types::VoxelOccupancy::Empty);

    const std::vector<double> scores = MapsComparison::compare(
        *reference, {identical.get(), distinct.get()});
    ASSERT_EQ(scores.size(), 2U);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_NEAR(scores[1], 0.0, 1e-6);
}

TEST(MapsComparison, EmptyTargetsVectorReturnsEmpty) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    const std::vector<double> scores = MapsComparison::compare(*reference, {});
    EXPECT_TRUE(scores.empty());
}

TEST(MapsComparison, GoldenMapRoundTripScores100) {
#ifdef DRONE_MAPPER_SOURCE_DIR
    const std::filesystem::path golden =
        std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "data_maps" / "single_voxel_x2_y4_z2.npy";
#else
    const std::filesystem::path golden =
        std::filesystem::path{"data_maps/single_voxel_x2_y4_z2.npy"};
#endif

    const types::MapConfig config = makeGoldenMapConfig();
    auto target_array = std::make_shared<NpyArray>();
    ASSERT_EQ(origin_array->LoadNPY(golden.string()), nullptr);
    ASSERT_EQ(target_array->LoadNPY(golden.string()), nullptr);

    Map3DImpl reference{origin_array, config};
    Map3DImpl target{target_array, config};

    const std::vector<double> scores =
        MapsComparison::compare(reference, {&target});
    ASSERT_EQ(scores.size(), 1U);
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

#ifdef MAPS_COMPARISON_EXE
TEST(MapsComparison, CliIdenticalMapsPrintsScoreOnly) {
    const types::MapConfig config = makeThreeByThreeConfig();
    auto reference = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    auto target = makeMutableMap(NpyArray::shape_t{3, 3, 3}, config);
    fillKnownGrid(*reference, types::VoxelOccupancy::Empty);
    fillKnownGrid(*target, types::VoxelOccupancy::Empty);

    const std::filesystem::path origin_path = tempMapPath("maps_comparison_origin.npy");
    const std::filesystem::path target_path = tempMapPath("maps_comparison_target.npy");
    reference->save(origin_path);
    target->save(target_path);

    const ProcessOutput output = runMapsComparison(
        {origin_path.string(), target_path.string()});
    EXPECT_EQ(output.exit_code, 0);
    EXPECT_EQ(output.stdout_text, "100\n");
    EXPECT_TRUE(output.stderr_text.empty());

    std::error_code ec;
    std::filesystem::remove(origin_path, ec);
    std::filesystem::remove(target_path, ec);
}

TEST(MapsComparison, CliMissingOriginPrintsMinusOne) {
    const ProcessOutput output = runMapsComparison(
        {"data_maps/does_not_exist.npy", "data_maps/does_not_exist.npy"});
    EXPECT_NE(output.exit_code, 0);
    EXPECT_EQ(output.stdout_text, "-1\n");
    EXPECT_FALSE(output.stderr_text.empty());
}

TEST(MapsComparison, CliWrongArgCountPrintsMinusOne) {
    const ProcessOutput output = runMapsComparison({});
    EXPECT_NE(output.exit_code, 0);
    EXPECT_EQ(output.stdout_text, "-1\n");
    EXPECT_NE(output.stderr_text.find("Usage"), std::string::npos);
}
#endif

} // namespace
} // namespace drone_mapper
