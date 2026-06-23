#pragma once

#include <drone_mapper/types/SimulationTypes.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace drone_mapper::io {

struct SimulationRunYamlEntry {
    int run_id = 0;
    std::size_t simulation_index = 0;
    std::size_t mission_index = 0;
    std::size_t drone_index = 0;
    std::size_t lidar_index = 0;
    types::SimulationResult result{};
};

void writeSimulationOutputYaml(const std::filesystem::path& output_path,
                               const types::SimulationManagerReport& report,
                               const std::vector<SimulationRunYamlEntry>& entries);

} // namespace drone_mapper::io
