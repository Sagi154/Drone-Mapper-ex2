#include <drone_mapper/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

#include <sstream>

namespace drone_mapper::io {

types::DroneConfigData parseDroneConfig(const std::filesystem::path& path, IRunErrorLog& log) {
    types::DroneConfigData config{};

    const auto root = detail::loadYamlFile(path, log, "[drone_config]");
    if (!root.has_value()) {
        config.config_load_error =
            types::ErrorRef{"CONFIG_FILE_NOT_FOUND", path.string()};
        return config;
    }

    const YAML::Node node = detail::configRoot(*root, "drone_config");

    if (const auto dimensions = detail::readLengthCm(node, "dimensions_cm")) {
        config.radius = (*dimensions) / 2.0;
    }
    if (const auto max_rotate = detail::readHorizontalAngleDeg(node, "max_rotate_deg")) {
        config.max_rotate = *max_rotate;
    }
    if (const auto max_advance = detail::readLengthCm(node, "max_advance_cm")) {
        config.max_advance = *max_advance;
    }
    if (const auto max_elevate = detail::readLengthCm(node, "max_elevate_cm")) {
        config.max_elevate = *max_elevate;
    }

    return config;
}

} // namespace drone_mapper::io
