#include <drone_mapper/io/YamlConfigParsers.h>
#include <drone_mapper/io/PathResolver.h>

#include "YamlParseUtil.hpp"

namespace drone_mapper::io {

types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path, IRunErrorLog& log) {
    types::SimulationConfigData config{};

    const auto root = detail::loadYamlFile(path, log, "[simulation_config]");
    if (!root.has_value()) {
        config.config_load_error =
            types::ErrorRef{"CONFIG_FILE_NOT_FOUND", path.string()};
        return config;
    }

    const YAML::Node node = detail::configRoot(*root, "simulation_config");

    if (const YAML::Node map_filename = node["map_filename"]; map_filename && map_filename.IsScalar()) {
        config.map_filename =
            resolveMapFilename(map_filename.as<std::string>(), path);
    }

    if (const auto map_resolution = detail::readLengthCm(node, "map_resolution_cm")) {
        config.map_resolution = *map_resolution;
    }

    if (const auto position = detail::readPosition3D(node["initial_drone_position"])) {
        config.initial_drone_position = *position;
    }

    if (const auto angle = detail::readHorizontalAngleDeg(node, "initial_angle_deg")) {
        config.initial_angle = *angle;
    }

    if (const auto map_offset = detail::readMapOffset(node["map_offset"])) {
        config.map_offset = *map_offset;
    } else if (const auto axes_offset = detail::readMapOffset(node["map_axes_offset"])) {
        config.map_offset = *axes_offset;
    }

    return config;
}

} // namespace drone_mapper::io
