// DroneControlImpl — per-step movement, scan, and voxel fusion orchestration.
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ScanResultToVoxels.h>

#include <cmath>
#include <utility>

namespace drone_mapper {
namespace {

void markDroneFootprintEmpty(IMutableMap3D& map,
                             const Position3D& centre,
                             PhysicalLength radius) {
    const types::MapConfig config = map.getMapConfig();
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (resolution_cm <= 0.0) {
        return;
    }

    const double radius_cm = radius.force_numerical_value_in(cm);
    const double cx = centre.x.force_numerical_value_in(cm);
    const double cy = centre.y.force_numerical_value_in(cm);
    const double cz = centre.z.force_numerical_value_in(cm);
    const int steps = static_cast<int>(std::ceil(radius_cm / resolution_cm));
    const double radius_sq = radius_cm * radius_cm;

    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const double ox = dx * resolution_cm;
                const double oy = dy * resolution_cm;
                const double oz = dz * resolution_cm;
                if (ox * ox + oy * oy + oz * oz > radius_sq) {
                    continue;
                }
                const Position3D sample{
                    (cx + ox) * x_extent[cm],
                    (cy + oy) * y_extent[cm],
                    (cz + oz) * z_extent[cm],
                };
                if (map.atVoxel(sample) != types::VoxelOccupancy::Occupied) {
                    map.set(sample, types::VoxelOccupancy::Empty);
                }
            }
        }
    }
}

[[nodiscard]] bool movementWithinLimits(const types::MovementCommand& command,
                                        const types::DroneConfigData& drone) {
    switch (command.type) {
    case types::MovementCommandType::Hover:
        return true;
    case types::MovementCommandType::Rotate:
        return command.angle <= drone.max_rotate;
    case types::MovementCommandType::Advance:
        return command.distance <= drone.max_advance;
    case types::MovementCommandType::Elevate:
        return std::abs(command.distance.force_numerical_value_in(cm)) <=
               drone.max_elevate.force_numerical_value_in(cm);
    }
    return false;
}

[[nodiscard]] types::MovementResult executeMovement(IDroneMovement& movement,
                                                    const types::MovementCommand& command) {
    switch (command.type) {
    case types::MovementCommandType::Hover:
        return types::MovementResult{true, {}};
    case types::MovementCommandType::Rotate:
        return movement.rotate(command.rotation, command.angle);
    case types::MovementCommandType::Advance:
        return movement.advance(command.distance);
    case types::MovementCommandType::Elevate:
        return movement.elevate(command.distance);
    }
    return types::MovementResult{false, "Unsupported movement command."};
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   types::LidarConfigData lidar,
                                   ILidar& lidar_sensor,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(std::move(lidar)),
      lidar_sensor_(lidar_sensor),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    const types::DroneState current_state = state();
    markDroneFootprintEmpty(output_map_, current_state.position, drone_.radius);

    const types::LidarScanResult* latest_scan_ptr = has_latest_scan_ ? &latest_scan_ : nullptr;
    const types::MappingStepCommand command =
        mapping_algorithm_.nextStep(current_state, latest_scan_ptr);

    if (command.status == types::AlgorithmStatus::Finished ||
        command.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return types::DroneStepResult{types::DroneStepStatus::Completed, {}};
    }

    if (command.movement.has_value()) {
        if (!movementWithinLimits(*command.movement, drone_)) {
            return types::DroneStepResult{
                types::DroneStepStatus::Error,
                "Movement command exceeds drone limits.",
            };
        }

        const types::MovementResult movement_result = executeMovement(movement_, *command.movement);
        if (!movement_result) {
            return types::DroneStepResult{
                types::DroneStepStatus::Error,
                movement_result.message.empty() ? "Movement failed." : movement_result.message,
            };
        }
    }

    if (command.scan_orientation.has_value()) {
        latest_scan_ = lidar_sensor_.scan(*command.scan_orientation);
        has_latest_scan_ = true;
        ScanResultToVoxels::applyToMap(
            output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_);
    }

    ++step_index_;
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
