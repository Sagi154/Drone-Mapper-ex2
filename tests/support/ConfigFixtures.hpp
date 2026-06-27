#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/io/ConfigParseResult.h>
#include <drone_mapper/io/IRunErrorLog.h>
#include <drone_mapper/io/YamlConfigParsers.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>
#include <string>
#include <vector>

namespace drone_mapper::test_support {

[[nodiscard]] inline std::filesystem::path goldenMapPath() {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "data_maps" / "single_voxel_x2_y4_z2.npy";
#else
    return std::filesystem::path{"data_maps/single_voxel_x2_y4_z2.npy"};
#endif
}

[[nodiscard]] inline std::filesystem::path benchmarkMapPath() {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "data_maps" / "benchmark_map.npy";
#else
    return std::filesystem::path{"data_maps/benchmark_map.npy"};
#endif
}

[[nodiscard]] inline std::filesystem::path configFixturePath(const std::string& filename) {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "tests" / "data" / "configs" / filename;
#else
    return std::filesystem::path{"tests/data/configs"} / filename;
#endif
}

class CapturingErrorLog final : public io::IRunErrorLog {
public:
    void log(const types::ErrorRef& error) override {
        entries_.push_back(error);
    }

    [[nodiscard]] const std::vector<types::ErrorRef>& entries() const {
        return entries_;
    }

private:
    std::vector<types::ErrorRef> entries_;
};

// Matches sim_tiny.yaml with an absolute map path so factory resolveMapPath works from any CWD.
[[nodiscard]] inline types::SimulationConfigData e2eSimulation() {
    return types::SimulationConfigData{
        goldenMapPath(),
        10.0 * cm,
        Position3D{},
        Position3D{20.0 * x_extent[cm], 40.0 * y_extent[cm], 20.0 * z_extent[cm]},
        0.0 * horizontal_angle[deg],
    };
}

[[nodiscard]] inline io::ConfigParseResult<types::SimulationCompositionData> loadE2eComposition(
    io::IRunErrorLog& log) {
    io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("composition_e2e.yaml"), log);
    if (result.ok) {
        for (types::SimulationConfigData& simulation : result.value.simulations) {
            simulation.map_filename = goldenMapPath();
        }
    }
    return result;
}

[[nodiscard]] inline io::ConfigParseResult<types::SimulationCompositionData> loadContinueAfterFailureComposition(
    io::IRunErrorLog& log) {
    return io::parseCompositionFile(configFixturePath("composition_continue_after_failure.yaml"), log);
}

[[nodiscard]] inline std::filesystem::path corruptMapPath() {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "tests" / "data" / "maps" / "corrupt_map.npy";
#else
    return std::filesystem::path{"tests/data/maps/corrupt_map.npy"};
#endif
}

[[nodiscard]] inline io::ConfigParseResult<types::SimulationCompositionData> loadInvalidDroneComposition(
    io::IRunErrorLog& log) {
    io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("composition_invalid_drone.yaml"), log);
    if (result.ok) {
        for (types::SimulationConfigData& simulation : result.value.simulations) {
            simulation.map_filename = goldenMapPath();
        }
    }
    return result;
}

[[nodiscard]] inline std::filesystem::path scenarioMapPath(int scenario) {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "data_maps" /
           ("scenario_" + std::to_string(scenario) + "_map.npy");
#else
    return std::filesystem::path{"data_maps/scenario_" + std::to_string(scenario) + "_map.npy"};
#endif
}

[[nodiscard]] inline std::filesystem::path scenarioCompositionPath(int scenario) {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "scenarios" /
           ("composition_scenario" + std::to_string(scenario) + ".yaml");
#else
    return std::filesystem::path{"scenarios/composition_scenario" + std::to_string(scenario) +
                                 ".yaml"};
#endif
}

[[nodiscard]] inline io::ConfigParseResult<types::SimulationCompositionData> loadBenchmarkComposition(
    io::IRunErrorLog& log) {
    io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(configFixturePath("composition_benchmark.yaml"), log);
    if (result.ok) {
        for (types::SimulationConfigData& simulation : result.value.simulations) {
            simulation.map_filename = benchmarkMapPath();
        }
    }
    return result;
}

[[nodiscard]] inline io::ConfigParseResult<types::SimulationCompositionData> loadScenarioComposition(
    int scenario,
    io::IRunErrorLog& log) {
    io::ConfigParseResult<types::SimulationCompositionData> result =
        io::parseCompositionFile(scenarioCompositionPath(scenario), log);
    if (result.ok) {
        for (types::SimulationConfigData& simulation : result.value.simulations) {
            simulation.map_filename = scenarioMapPath(scenario);
        }
    }
    return result;
}

} // namespace drone_mapper::test_support
