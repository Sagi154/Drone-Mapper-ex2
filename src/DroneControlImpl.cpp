// DroneControlImpl.cpp
// Per-step movement, scan, and voxel fusion orchestration for MissionControlImpl.

#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <cmath>
#include <limits>
#include <mp-units/systems/si/math.h>
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

[[nodiscard]] Orientation absoluteBeamOrientation(const Orientation& drone_heading,
                                                  const Orientation& relative_beam) {
    return Orientation{
        relative_beam.horizontal + drone_heading.horizontal,
        relative_beam.altitude + drone_heading.altitude,
    };
}

[[nodiscard]] Position3D pointAlongBeam(const Position3D& origin,
                                        const Orientation& beam_orientation,
                                        PhysicalLength distance) {
    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    const double distance_cm = distance.force_numerical_value_in(cm);
    const double dir_x = dx.force_numerical_value_in(mp::one);
    const double dir_y = dy.force_numerical_value_in(mp::one);
    const double dir_z = dz.force_numerical_value_in(mp::one);

    return Position3D{
        origin.x + dir_x * distance_cm * x_extent[cm],
        origin.y + dir_y * distance_cm * y_extent[cm],
        origin.z + dir_z * distance_cm * z_extent[cm],
    };
}

[[nodiscard]] double normalProximityMaxCm(const types::DroneConfigData& drone,
                                          const types::LidarConfigData& lidar,
                                          const types::MapConfig& config) {
    const double radius_cm = drone.radius.force_numerical_value_in(cm);
    const double cell_cm = config.resolution.force_numerical_value_in(cm);
    const double z_max_cm = lidar.z_max.force_numerical_value_in(cm);
    if (cell_cm <= 0.0) {
        return z_max_cm;
    }
    const double normal_cm = (std::ceil(radius_cm / cell_cm) + 1.0) * cell_cm;
    return std::min(z_max_cm, normal_cm);
}

void setEmptyIfNotOccupied(IMutableMap3D& map, const Position3D& position) {
    if (!map.isInBounds(position)) {
        return;
    }
    if (map.atVoxel(position) != types::VoxelOccupancy::Occupied) {
        map.set(position, types::VoxelOccupancy::Empty);
    }
}

void setOccupied(IMutableMap3D& map, const Position3D& position) {
    if (!map.isInBounds(position)) {
        return;
    }
    map.set(position, types::VoxelOccupancy::Occupied);
}

// Ex1 fused lidar hits on grid-aligned steps (ray_step = cell size). ScanResultToVoxels
// is still called first per skeleton contract; this supplements empty paths that the
// sub-voxel march cannot write because Map3DImpl only accepts voxel-centre positions.
void supplementGridAlignedScanFusion(IMutableMap3D& output_map,
                                     const Position3D& scan_origin,
                                     const Orientation& drone_heading,
                                     const types::LidarScanResult& scan,
                                     double fusion_max_cm) {
    const double grid_step_cm = output_map.getMapConfig().resolution.force_numerical_value_in(cm);
    if (grid_step_cm <= 0.0) {
        return;
    }

    constexpr double kMissDistance = std::numeric_limits<double>::max();

    for (const types::LidarHit& hit : scan) {
        const Orientation beam_orientation = absoluteBeamOrientation(drone_heading, hit.angle);
        const double distance_cm = hit.distance.force_numerical_value_in(cm);

        if (distance_cm == kMissDistance) {
            for (double t_cm = grid_step_cm; t_cm <= fusion_max_cm + 1e-9; t_cm += grid_step_cm) {
                setEmptyIfNotOccupied(
                    output_map, pointAlongBeam(scan_origin, beam_orientation, t_cm * cm));
            }
            continue;
        }

        if (distance_cm == 0.0 || distance_cm > fusion_max_cm) {
            continue;
        }

        for (double t_cm = grid_step_cm; t_cm < distance_cm + 1e-9; t_cm += grid_step_cm) {
            setEmptyIfNotOccupied(
                output_map, pointAlongBeam(scan_origin, beam_orientation, t_cm * cm));
        }
        setOccupied(output_map, pointAlongBeam(scan_origin, beam_orientation, hit.distance));
    }
}

[[nodiscard]] bool isRecoverableMovementFailure(const std::string& message) {
    return message.find("blocked") != std::string::npos ||
           message.find("boundary") != std::string::npos;
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
            if (isRecoverableMovementFailure(movement_result.message)) {
                ++step_index_;
                return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
            }
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
        const double fusion_max_cm =
            normalProximityMaxCm(drone_, lidar_, output_map_.getMapConfig());
        supplementGridAlignedScanFusion(
            output_map_, gps_.position(), gps_.heading(), latest_scan_, fusion_max_cm);
    }

    ++step_index_;
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
