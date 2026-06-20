#include <drone_mapper/io/YamlConfigParsers.h>
#include <drone_mapper/io/PathResolver.h>

#include "YamlParseUtil.hpp"

#include <sstream>

namespace drone_mapper::io {

namespace {

[[nodiscard]] std::vector<std::filesystem::path> readPathList(const YAML::Node& node, const char* key) {
    std::vector<std::filesystem::path> paths;
    const YAML::Node list = node[key];
    if (!list || !list.IsSequence()) {
        return paths;
    }

    for (const YAML::Node& entry : list) {
        if (entry.IsScalar()) {
            paths.emplace_back(entry.as<std::string>());
        }
    }
    return paths;
}

} // namespace

ConfigParseResult<types::SimulationCompositionData> parseCompositionFile(const std::filesystem::path& path,
                                                                         IRunErrorLog& log) {
    ConfigParseResult<types::SimulationCompositionData> result{};

    const auto root = detail::loadYamlFile(path, log, "[simulation_compositions]");
    if (!root.has_value()) {
        result.errors.push_back({"COMPOSITION_FILE_UNREADABLE", "Could not read composition file: " + path.string()});
        return result;
    }

    const YAML::Node compositions = detail::configRoot(*root, "simulation_compositions");
    const std::filesystem::path base_dir = path.has_parent_path() ? path.parent_path() : std::filesystem::path{"."};

    const YAML::Node simulations = compositions["simulations"];
    if (!simulations || !simulations.IsSequence() || simulations.size() == 0) {
        result.errors.push_back(
            {"COMPOSITION_INVALID", "simulation_compositions.simulations must be a non-empty sequence"});
        return result;
    }

    types::SimulationCompositionData composition{};
    composition.composition_file = path;

    // Expand nested simulation+mission pairs into aligned vectors for SimulationManager zip-loop (Phase 3).
    for (const YAML::Node& simulation_entry : simulations) {
        if (!simulation_entry["simulation_config"]) {
            detail::logRecoverable(log, "COMPOSITION_INVALID",
                                   "[simulation_compositions] simulation entry missing simulation_config — skipped");
            continue;
        }

        const auto simulation_path =
            resolveConfigPath(base_dir, simulation_entry["simulation_config"].as<std::string>());
        types::SimulationConfigData simulation = parseSimulationConfig(simulation_path, log);

        const YAML::Node mission_list = simulation_entry["mission_configs"];
        if (!mission_list || !mission_list.IsSequence() || mission_list.size() == 0) {
            detail::logRecoverable(log, "COMPOSITION_INVALID",
                                   "[simulation_compositions] simulation entry missing mission_configs — skipped");
            continue;
        }

        for (const YAML::Node& mission_entry : mission_list) {
            if (!mission_entry.IsScalar()) {
                continue;
            }
            const auto mission_path = resolveConfigPath(base_dir, mission_entry.as<std::string>());
            composition.simulations.push_back(simulation);
            composition.missions.push_back(parseMissionConfig(mission_path, log));
        }
    }

    if (composition.simulations.empty()) {
        result.errors.push_back({"COMPOSITION_INVALID", "No valid simulation/mission pairs found in composition"});
        return result;
    }

    const auto drone_paths = readPathList(compositions, "drone_configs");
    const auto lidar_paths = readPathList(compositions, "lidar_configs");

    if (drone_paths.empty() || lidar_paths.empty()) {
        result.errors.push_back(
            {"COMPOSITION_INVALID", "simulation_compositions requires non-empty drone_configs and lidar_configs"});
        return result;
    }

    for (const auto& drone_path : drone_paths) {
        composition.drones.push_back(parseDroneConfig(resolveConfigPath(base_dir, drone_path), log));
    }
    for (const auto& lidar_path : lidar_paths) {
        composition.lidars.push_back(parseLidarConfig(resolveConfigPath(base_dir, lidar_path), log));
    }

    if (composition.simulations.size() != composition.missions.size()) {
        result.errors.push_back({"COMPOSITION_INTERNAL_ERROR", "Aligned simulation/mission vectors size mismatch"});
        return result;
    }

    result.ok = true;
    result.value = std::move(composition);
    return result;
}

} // namespace drone_mapper::io
