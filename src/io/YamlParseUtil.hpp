#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/io/IRunErrorLog.h>
#include <drone_mapper/types/MapTypes.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <optional>
#include <string>

namespace drone_mapper::io::detail {

[[nodiscard]] std::optional<YAML::Node> loadYamlFile(const std::filesystem::path& path, IRunErrorLog& log,
                                                     const std::string& context);

[[nodiscard]] YAML::Node configRoot(const YAML::Node& root, const char* wrapper_key);

void logRecoverable(IRunErrorLog& log, const std::string& code, const std::string& message);

[[nodiscard]] std::optional<double> readScalarDouble(const YAML::Node& node, const char* key);

[[nodiscard]] std::optional<PhysicalLength> readLengthCm(const YAML::Node& node, const char* key);

[[nodiscard]] std::optional<HorizontalAngle> readHorizontalAngleDeg(const YAML::Node& node, const char* key);

[[nodiscard]] std::optional<Position3D> readPosition3D(const YAML::Node& node);

[[nodiscard]] std::optional<Position3D> readMapOffset(const YAML::Node& node);

[[nodiscard]] std::optional<types::MappingBounds> readMissionBoundaries(const YAML::Node& node);

} // namespace drone_mapper::io::detail
