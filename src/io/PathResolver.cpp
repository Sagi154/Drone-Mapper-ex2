#include <drone_mapper/io/PathResolver.h>

namespace drone_mapper::io {

std::filesystem::path resolveConfigPath(const std::filesystem::path& base_dir,
                                        const std::filesystem::path& config_path) {
    if (config_path.is_absolute()) {
        return config_path;
    }
    return base_dir / config_path;
}

} // namespace drone_mapper::io
