#pragma once

#include <drone_mapper/io/ConfigParseResult.h>
#include <drone_mapper/io/IRunErrorLog.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>

namespace drone_mapper::io {

[[nodiscard]] types::DroneConfigData parseDroneConfig(const std::filesystem::path& path, IRunErrorLog& log);

[[nodiscard]] types::MissionConfigData parseMissionConfig(const std::filesystem::path& path, IRunErrorLog& log);

[[nodiscard]] types::LidarConfigData parseLidarConfig(const std::filesystem::path& path, IRunErrorLog& log);

[[nodiscard]] types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path,
                                                                IRunErrorLog& log);

[[nodiscard]] ConfigParseResult<types::SimulationCompositionData> parseCompositionFile(
    const std::filesystem::path& path, IRunErrorLog& log);

} // namespace drone_mapper::io
