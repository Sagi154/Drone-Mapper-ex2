#include <drone_mapper/io/SimulationCli.h>
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

TEST(SimulationCliTest, DefaultCompositionPathWhenNoArgs) {
    char arg0[] = "drone_mapper_simulation";
    char* argv[] = {arg0};

    const io::SimulationCliArgs args = io::parseSimulationCliArgs(1, argv);

    EXPECT_EQ(args.composition_file, std::filesystem::path{"simulation.yaml"});
    EXPECT_EQ(args.output_path, std::filesystem::current_path());
}

TEST(SimulationCliTest, UsesExplicitCompositionAndOutputPaths) {
    char arg0[] = "drone_mapper_simulation";
    char arg1[] = "configs/composition_minimal.yaml";
    char arg2[] = "output";
    char* argv[] = {arg0, arg1, arg2};

    const io::SimulationCliArgs args = io::parseSimulationCliArgs(3, argv);

    EXPECT_EQ(args.composition_file, std::filesystem::path{"configs/composition_minimal.yaml"});
    EXPECT_EQ(args.output_path, std::filesystem::path{"output"});
}

TEST(SimulationCliTest, MissingCompositionFileFailsParse) {
    CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("missing_composition.yaml"), log);

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());
    EXPECT_TRUE(log.containsCode("CONFIG_FILE_NOT_FOUND"));
}

TEST(SimulationCliTest, LoadsMinimalCompositionFixture) {
    CapturingErrorLog log;
    const io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("composition_minimal.yaml"), log);

    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.value.simulations.size(), 1U);
    EXPECT_EQ(result.value.missions.size(), 1U);
    EXPECT_EQ(result.value.drones.size(), 1U);
    EXPECT_EQ(result.value.lidars.size(), 1U);
    EXPECT_EQ(result.value.simulations.size(), result.value.missions.size());
    EXPECT_TRUE(log.entries().empty());
}

} // namespace drone_mapper
