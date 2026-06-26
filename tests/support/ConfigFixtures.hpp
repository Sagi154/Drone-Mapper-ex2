#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/io/ConfigParseResult.h>
#include <drone_mapper/io/IRunErrorLog.h>
#include <drone_mapper/io/YamlConfigParsers.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>
#include <vector>

namespace drone_mapper::test_support {

[[nodiscard]] inline std::filesystem::path goldenMapPath() {
#ifdef DRONE_MAPPER_SOURCE_DIR
    return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR} / "data_maps" / "single_voxel_x2_y4_z2.npy";
#else
    return std::filesystem::path{"data_maps/single_voxel_x2_y4_z2.npy"};
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

} // namespace drone_mapper::test_support
