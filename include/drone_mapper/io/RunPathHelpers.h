#pragma once

#include <filesystem>

namespace drone_mapper::io {

[[nodiscard]] std::filesystem::path runOutputDir(const std::filesystem::path& output_path, int run_id);

[[nodiscard]] std::filesystem::path runOutputMap(const std::filesystem::path& output_path, int run_id);

[[nodiscard]] std::filesystem::path runErrorLog(const std::filesystem::path& output_path, int run_id);

} // namespace drone_mapper::io
