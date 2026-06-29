#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace drone_mapper::types {

struct ErrorRef {
    std::string code{};
    std::string message{};
};

struct MissionConfigData {
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    double output_mapping_resolution_factor = 0;
    // Optional mission mapping bounds from mission_config YAML (20.6). When unset (all zero),
    // output map boundaries fall back to the hidden map's MapConfig.
    MappingBounds boundaries{};
    std::optional<ErrorRef> config_load_error{};
};

enum class MissionRunStatus {
    Completed,
    MaxSteps,
    Error,
};

struct MissionRunResult {
    MissionRunStatus status = MissionRunStatus::Completed;
    std::size_t steps = 0;
    // Changed: a run can report multiple errors instead of a single ErrorRef.
    std::vector<ErrorRef> errors{}; // we may have multiple errors

    //Removed in 9.6
    // double score = 0.0; // moved to simulationResult
    // std::filesystem::path output_map_file{}; // moved to simulation Result
};

} // namespace drone_mapper::types
