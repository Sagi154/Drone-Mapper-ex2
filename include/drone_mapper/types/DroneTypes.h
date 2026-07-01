#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/types/MissionTypes.h>

#include <cstddef>
#include <optional>
#include <string>

namespace drone_mapper::types {

struct DroneConfigData {
    //Change: dimensions changes to radius
    PhysicalLength radius{}; // we assume it to be a perfect sphere
    HorizontalAngle max_rotate{};
    PhysicalLength max_advance{};
    PhysicalLength max_elevate{};
    std::optional<ErrorRef> config_load_error{};
};

enum class RotationDirection {
    Left,
    Right,
};

enum class MovementCommandType {
    Hover,
    Rotate,
    Advance,
    Elevate,
};


struct MovementCommand {
    MovementCommandType type = MovementCommandType::Hover;
    RotationDirection rotation = RotationDirection::Left;
    HorizontalAngle angle{};
    PhysicalLength distance{};
};

enum class AlgorithmStatus{
    Working,
    Finished,
    FinishedWithUnmappableVoxels,
};

struct MappingStepCommand {

    // Valid to provide both - if both are provided, movement must be performed before scan.
    std::optional<MovementCommand> movement{};
    std::optional<Orientation> scan_orientation{};
    // Local lidar fusion cap for this shot (ex1 proximity_max). When set, only
    // observations within this range are written to the output map.
    std::optional<PhysicalLength> fusion_max{};
    AlgorithmStatus status = AlgorithmStatus::Working;
};

struct MovementResult {
    bool success = true;
    std::string message{};

    [[nodiscard]] explicit operator bool() const noexcept {
        return success;
    }
};

struct DroneState {
    Position3D position{};
    Orientation heading{};
    std::size_t step_index = 0;
};

enum class DroneStepStatus {
    Continue,
    Completed,
    Error,
};

struct DroneStepResult {
    DroneStepStatus status = DroneStepStatus::Continue;
    std::string message{};
};

} // namespace drone_mapper::types
