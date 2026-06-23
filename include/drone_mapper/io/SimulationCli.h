#pragma once

#include <filesystem>

namespace drone_mapper::io {

struct SimulationCliArgs {
    std::filesystem::path composition_file;
    std::filesystem::path output_path;
};

[[nodiscard]] SimulationCliArgs parseSimulationCliArgs(int argc, char** argv);

} // namespace drone_mapper::io
