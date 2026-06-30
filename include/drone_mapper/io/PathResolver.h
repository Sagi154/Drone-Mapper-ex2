#pragma once

#include <filesystem>

namespace drone_mapper::io {

[[nodiscard]] std::filesystem::path resolveConfigPath(const std::filesystem::path& base_dir,
                                                      const std::filesystem::path& config_path);

// Resolves hidden-map paths from simulation_config YAML.
// Search order: as-is, simulation YAML parent dir, inputs-style grandparent dir, CWD.
// When simulation_config_path is empty, only as-is / absolute / CWD are tried.
[[nodiscard]] std::filesystem::path resolveMapFilename(
    const std::filesystem::path& map_filename,
    const std::filesystem::path& simulation_config_path = {});

} // namespace drone_mapper::io
