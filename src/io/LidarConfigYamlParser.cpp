#include <drone_mapper/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

namespace drone_mapper::io {

types::LidarConfigData parseLidarConfig(const std::filesystem::path& path, IRunErrorLog& log) {
    types::LidarConfigData config{};

    const auto root = detail::loadYamlFile(path, log, "[lidar_config]");
    if (!root.has_value()) {
        return config;
    }

    const YAML::Node node = detail::configRoot(*root, "lidar_config");

    if (const auto z_min = detail::readLengthCm(node, "z_min_cm")) {
        config.z_min = *z_min;
    }
    if (const auto z_max = detail::readLengthCm(node, "z_max_cm")) {
        config.z_max = *z_max;
    }
    if (const auto d = detail::readLengthCm(node, "d_cm")) {
        config.d = *d;
    }
    if (const YAML::Node fov = node["fov_circles"]; fov && fov.IsScalar()) {
        try {
            config.fov_circles = fov.as<std::size_t>();
        } catch (const YAML::Exception&) {
            detail::logRecoverable(log, "CONFIG_BAD_VALUE",
                                   "[lidar_config] bad value for fov_circles — field keeps default");
        }
    }

    return config;
}

} // namespace drone_mapper::io
