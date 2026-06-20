#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/io/RunErrorLog.h>
#include <drone_mapper/io/YamlConfigParsers.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] std::filesystem::path goldenMapPath() {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "data_maps" / "single_voxel_x2_y4_z2.npy";
#else
    return std::filesystem::path{"data_maps/single_voxel_x2_y4_z2.npy"};
#endif
}

[[nodiscard]] std::filesystem::path configFixturePath(const std::string& filename) {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "tests" / "data" / "configs" / filename;
#else
    return std::filesystem::path{"tests/data/configs"} / filename;
#endif
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

[[nodiscard]] bool matchesErrorLogFormat(const std::string& line) {
    static const std::regex pattern{
        R"(^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z [A-Z0-9_]+ .+$)"};
    return std::regex_match(line, pattern);
}

} // namespace

TEST(RunErrorLog, WritesStructuredLineAndFlushesImmediately) {
    const std::filesystem::path log_path =
        std::filesystem::temp_directory_path() / "run_error_log_flush_test.log";

    {
        io::RunErrorLog logger{log_path};
        logger.log({"TEST_ERROR_CODE", "user-facing message"});
    }

    const std::vector<std::string> lines = readAllLines(log_path);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_TRUE(matchesErrorLogFormat(lines.front()));
    EXPECT_NE(lines.front().find("TEST_ERROR_CODE"), std::string::npos);
    EXPECT_NE(lines.front().find("user-facing message"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(log_path, ec);
}

TEST(RunErrorLog, CreatesParentDirectoryIfMissing) {
    const std::filesystem::path log_path =
        std::filesystem::temp_directory_path() / "run_error_log_nested" / "deep" / "error.log";

    io::RunErrorLog logger{log_path};
    logger.log({"DIR_TEST", "created nested log path"});

    EXPECT_TRUE(std::filesystem::exists(log_path));

    std::error_code ec;
    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "run_error_log_nested", ec);
}

TEST(RunErrorLog, AppendsMultipleEntries) {
    const std::filesystem::path log_path =
        std::filesystem::temp_directory_path() / "run_error_log_append_test.log";

    io::RunErrorLog logger{log_path};
    logger.log({"FIRST", "first message"});
    logger.log({"SECOND", "second message"});

    const std::vector<std::string> lines = readAllLines(log_path);
    ASSERT_EQ(lines.size(), 2U);
    EXPECT_NE(lines[0].find("FIRST"), std::string::npos);
    EXPECT_NE(lines[1].find("SECOND"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(log_path, ec);
}

TEST(RunErrorLog, GateAPartialLoadMapParseYamlAndLogToFile) {
    const auto map_array = std::make_shared<NpyArray>();
    const LPCSTR load_error = map_array->LoadNPY(goldenMapPath().string());
    ASSERT_EQ(load_error, nullptr);

    types::MapConfig config{};
    config.resolution = 10.0 * cm;
    const Map3DImpl map_impl{map_array, config};
    EXPECT_TRUE(map_impl.isInBounds(
        Position3D{20.0 * x_extent[cm], 40.0 * y_extent[cm], 20.0 * z_extent[cm]}));

    const std::filesystem::path log_path =
        std::filesystem::temp_directory_path() / "gate_a_partial_demo.log";
    io::RunErrorLog logger{log_path};

    const types::MissionConfigData mission =
        io::parseMissionConfig(configFixturePath("mission_basic.yaml"), logger);
    EXPECT_EQ(mission.max_steps, 2400U);

    logger.log({.code = "GATE_A_DEMO", .message = "Phase 1 Gate A partial verification log entry"});

    const std::vector<std::string> lines = readAllLines(log_path);
    ASSERT_GE(lines.size(), 1U);
    EXPECT_TRUE(matchesErrorLogFormat(lines.back()));
    EXPECT_NE(lines.back().find("GATE_A_DEMO"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(log_path, ec);
}

} // namespace drone_mapper
