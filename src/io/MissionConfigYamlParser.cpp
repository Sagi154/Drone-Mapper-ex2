#include <drone_mapper/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

#include <sstream>

namespace drone_mapper::io {

types::MissionConfigData parseMissionConfig(const std::filesystem::path& path, IRunErrorLog& log) {
    types::MissionConfigData config{};
    config.output_mapping_resolution_factor = 1.0;

    const auto root = detail::loadYamlFile(path, log, "[mission_config]");
    if (!root.has_value()) {
        return config;
    }

    const YAML::Node node = detail::configRoot(*root, "mission_config");

    if (node["boundaries"]) {
        detail::logRecoverable(
            log, "MISSION_BOUNDARIES_IGNORED",
            "[mission_config] boundaries are ignored — map bounds come from MapConfig (9.6 refactor)");
    }

    if (const YAML::Node max_steps = node["max_steps"]; max_steps && max_steps.IsScalar()) {
        try {
            config.max_steps = max_steps.as<std::size_t>();
        } catch (const YAML::Exception&) {
            detail::logRecoverable(log, "CONFIG_BAD_VALUE",
                                   "[mission_config] bad value for max_steps — field keeps default");
        }
    }

    if (const auto gps_resolution = detail::readLengthCm(node, "gps_resolution_cm")) {
        config.gps_resolution = *gps_resolution;
    }

    if (const auto factor = detail::readScalarDouble(node, "output_mapping_resolution_factor")) {
        if (*factor < 1.0) {
            detail::logRecoverable(
                log, "CONFIG_BAD_VALUE",
                "[mission_config] output_mapping_resolution_factor < 1 — ignored, using default 1");
            config.output_mapping_resolution_factor = 1.0;
        } else {
            config.output_mapping_resolution_factor = *factor;
        }
    }

    return config;
}

} // namespace drone_mapper::io
