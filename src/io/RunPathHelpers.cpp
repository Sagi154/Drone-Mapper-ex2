#include <drone_mapper/io/RunPathHelpers.h>

#include <iomanip>
#include <sstream>

namespace drone_mapper::io {

namespace {

[[nodiscard]] std::filesystem::path formatRunDir(const std::filesystem::path& output_path, int run_id) {
    std::ostringstream folder;
    folder << "run_" << std::setw(4) << std::setfill('0') << run_id;
    return output_path / "output_results" / folder.str();
}

} // namespace

std::filesystem::path runOutputDir(const std::filesystem::path& output_path, int run_id) {
    return formatRunDir(output_path, run_id);
}

std::filesystem::path runOutputMap(const std::filesystem::path& output_path, int run_id) {
    return runOutputDir(output_path, run_id) / "output_map.npy";
}

std::filesystem::path runErrorLog(const std::filesystem::path& output_path, int run_id) {
    return runOutputDir(output_path, run_id) / "error.log";
}

} // namespace drone_mapper::io
