#pragma once

#include <filesystem>

namespace drone_mapper::io {

[[nodiscard]] std::filesystem::path resolveConfigPath(const std::filesystem::path& base_dir,
                                                      const std::filesystem::path& config_path);

} // namespace drone_mapper::io
