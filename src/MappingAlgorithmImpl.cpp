// MappingAlgorithmImpl — frontier BFS exploration ported from ex1 ExplorationFrontier ideas.
// nextStep drives scan / plan / move phases; reads output map only (no direct voxel writes).
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <queue>
#include <unordered_map>
#include <utility>

namespace drone_mapper {
namespace {

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;

    auto operator==(const VoxelKey&) const -> bool = default;
};

struct VoxelKeyHash {
    [[nodiscard]] std::size_t operator()(const VoxelKey& key) const noexcept {
        const std::size_t hx = static_cast<std::size_t>(key.x) * 73856093U;
        const std::size_t hy = static_cast<std::size_t>(key.y) * 19349663U;
        const std::size_t hz = static_cast<std::size_t>(key.z) * 83492791U;
        return hx ^ hy ^ hz;
    }
};

constexpr double kHalfStepTolerance = 0.5;
constexpr double kPositionEpsilon = 1e-6;

[[nodiscard]] VoxelKey toVoxelKey(const Position3D& pos, const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);
    return VoxelKey{
        static_cast<int>(std::lround((px - ox) / resolution_cm)),
        static_cast<int>(std::lround((py - oy) / resolution_cm)),
        static_cast<int>(std::lround((pz - oz) / resolution_cm)),
    };
}

[[nodiscard]] Position3D toPosition(const VoxelKey& key, const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    return Position3D{
        (ox + static_cast<double>(key.x) * resolution_cm) * x_extent[cm],
        (oy + static_cast<double>(key.y) * resolution_cm) * y_extent[cm],
        (oz + static_cast<double>(key.z) * resolution_cm) * z_extent[cm],
    };
}

[[nodiscard]] bool isSpherePassable(const IMap3D& map,
                                    const Position3D& centre,
                                    PhysicalLength radius,
                                    const types::MapConfig& config) {
    if (map.atVoxel(centre) != types::VoxelOccupancy::Empty) {
        return false;
    }

    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const double radius_cm = radius.force_numerical_value_in(cm);
    const double cx = centre.x.force_numerical_value_in(cm);
    const double cy = centre.y.force_numerical_value_in(cm);
    const double cz = centre.z.force_numerical_value_in(cm);
    const int steps = static_cast<int>(std::ceil(radius_cm / resolution_cm));
    const double radius_sq = radius_cm * radius_cm;

    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
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
                if (map.atVoxel(sample) == types::VoxelOccupancy::Unmapped) {
                    return false;
                }
            }
        }
    }
    return true;
}

[[nodiscard]] bool sphereContainsUnmapped(const IMap3D& map,
                                          const Position3D& centre,
                                          PhysicalLength radius,
                                          const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
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
                if (map.atVoxel(sample) == types::VoxelOccupancy::Unmapped) {
                    return true;
                }
            }
        }
    }
    return false;
}

[[nodiscard]] bool isFrontier(const IMap3D& map,
                              const Position3D& cell,
                              PhysicalLength radius,
                              const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const double cx = cell.x.force_numerical_value_in(cm);
    const double cy = cell.y.force_numerical_value_in(cm);
    const double cz = cell.z.force_numerical_value_in(cm);

    const std::array<Position3D, 6> neighbours{{
        {(cx + resolution_cm) * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
        {(cx - resolution_cm) * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
        {cx * x_extent[cm], (cy + resolution_cm) * y_extent[cm], cz * z_extent[cm]},
        {cx * x_extent[cm], (cy - resolution_cm) * y_extent[cm], cz * z_extent[cm]},
        {cx * x_extent[cm], cy * y_extent[cm], (cz + resolution_cm) * z_extent[cm]},
        {cx * x_extent[cm], cy * y_extent[cm], (cz - resolution_cm) * z_extent[cm]},
    }};

    for (const Position3D& neighbour : neighbours) {
        if (sphereContainsUnmapped(map, neighbour, radius, config)) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] std::vector<Position3D> findFrontierPath(const IMap3D& map,
                                                       const Position3D& start,
                                                       PhysicalLength radius) {
    const types::MapConfig config = map.getMapConfig();
    const VoxelKey start_key = toVoxelKey(start, config);
    std::unordered_map<VoxelKey, VoxelKey, VoxelKeyHash> parent_of;
    std::queue<VoxelKey> queue;

    const Position3D start_pos = toPosition(start_key, config);
    if (!isSpherePassable(map, start_pos, radius, config)) {
        return {};
    }

    parent_of[start_key] = start_key;
    queue.push(start_key);

    constexpr std::array<VoxelKey, 6> offsets{{
        {1, 0, 0},
        {-1, 0, 0},
        {0, 1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1},
    }};

    while (!queue.empty()) {
        const VoxelKey current = queue.front();
        queue.pop();
        const Position3D current_pos = toPosition(current, config);

        if (current != start_key && isFrontier(map, current_pos, radius, config)) {
            std::vector<Position3D> path;
            VoxelKey step = current;
            while (step != start_key) {
                path.push_back(toPosition(step, config));
                step = parent_of.at(step);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const VoxelKey& offset : offsets) {
            const VoxelKey neighbour{current.x + offset.x, current.y + offset.y, current.z + offset.z};
            if (parent_of.contains(neighbour)) {
                continue;
            }
            const Position3D neighbour_pos = toPosition(neighbour, config);
            if (!isSpherePassable(map, neighbour_pos, radius, config)) {
                continue;
            }
            parent_of[neighbour] = current;
            queue.push(neighbour);
        }
    }

    return {};
}

[[nodiscard]] bool hasUnmappedInBounds(const IMap3D& map) {
    const types::MapConfig config = map.getMapConfig();
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (resolution_cm <= 0.0) {
        return false;
    }

    const double min_x = config.boundaries.min_x.force_numerical_value_in(cm);
    const double max_x = config.boundaries.max_x.force_numerical_value_in(cm);
    const double min_y = config.boundaries.min_y.force_numerical_value_in(cm);
    const double max_y = config.boundaries.max_y.force_numerical_value_in(cm);
    const double min_z = config.boundaries.min_height.force_numerical_value_in(cm);
    const double max_z = config.boundaries.max_height.force_numerical_value_in(cm);

    for (double x = min_x; x <= max_x + 1e-9; x += resolution_cm) {
        for (double y = min_y; y <= max_y + 1e-9; y += resolution_cm) {
            for (double z = min_z; z <= max_z + 1e-9; z += resolution_cm) {
                const Position3D pos{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
                if (map.atVoxel(pos) == types::VoxelOccupancy::Unmapped) {
                    return true;
                }
            }
        }
    }
    return false;
}

[[nodiscard]] bool reachedWaypoint(const Position3D& pos,
                                   const Position3D& target,
                                   const types::MapConfig& config) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const double dx = std::abs(pos.x.force_numerical_value_in(cm) - target.x.force_numerical_value_in(cm));
    const double dy = std::abs(pos.y.force_numerical_value_in(cm) - target.y.force_numerical_value_in(cm));
    const double dz = std::abs(pos.z.force_numerical_value_in(cm) - target.z.force_numerical_value_in(cm));
    return dx <= resolution_cm * kHalfStepTolerance && dy <= resolution_cm * kHalfStepTolerance &&
           dz <= resolution_cm * kHalfStepTolerance;
}

[[nodiscard]] types::MappingStepCommand makeScanCommand() {
    types::MappingStepCommand command{};
    command.scan_orientation = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    command.status = types::AlgorithmStatus::Working;
    return command;
}

[[nodiscard]] types::MappingStepCommand makeFinishedCommand(types::AlgorithmStatus status) {
    types::MappingStepCommand command{};
    command.status = status;
    return command;
}

[[nodiscard]] types::MovementCommand movementToward(const types::DroneState& state,
                                                    const Position3D& target,
                                                    const types::DroneConfigData& drone) {
    const double target_x = target.x.force_numerical_value_in(cm);
    const double target_y = target.y.force_numerical_value_in(cm);
    const double target_z = target.z.force_numerical_value_in(cm);
    const double pos_x = state.position.x.force_numerical_value_in(cm);
    const double pos_y = state.position.y.force_numerical_value_in(cm);
    const double pos_z = state.position.z.force_numerical_value_in(cm);

    const double dh = target_z - pos_z;
    if (std::abs(dh) > kPositionEpsilon) {
        const double limit = drone.max_elevate.force_numerical_value_in(cm);
        types::MovementCommand command{};
        command.type = types::MovementCommandType::Elevate;
        command.distance = std::clamp(dh, -limit, limit) * cm;
        return command;
    }

    const double dx = target_x - pos_x;
    const double dy = target_y - pos_y;
    if (std::abs(dx) < kPositionEpsilon && std::abs(dy) < kPositionEpsilon) {
        types::MovementCommand command{};
        command.type = types::MovementCommandType::Hover;
        return command;
    }

    const double target_heading = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
    const double current_heading = state.heading.horizontal.force_numerical_value_in(deg);
    double delta = std::fmod(target_heading - current_heading, 360.0);
    if (delta > 180.0) {
        delta -= 360.0;
    }
    if (delta < -180.0) {
        delta += 360.0;
    }

    const double rot_limit = drone.max_rotate.force_numerical_value_in(deg);
    if (std::abs(delta) > kPositionEpsilon) {
        types::MovementCommand command{};
        command.type = types::MovementCommandType::Rotate;
        command.rotation = (delta > 0.0) ? types::RotationDirection::Right : types::RotationDirection::Left;
        command.angle = std::min(std::abs(delta), rot_limit) * horizontal_angle[deg];
        return command;
    }

    const double dist_cm = std::sqrt(dx * dx + dy * dy);
    const double adv_limit = drone.max_advance.force_numerical_value_in(cm);
    types::MovementCommand command{};
    command.type = types::MovementCommandType::Advance;
    command.distance = std::min(dist_cm, adv_limit) * cm;
    return command;
}

} // namespace

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    if (finished_) {
        return makeFinishedCommand(types::AlgorithmStatus::Finished);
    }

    const types::MapConfig config = output_map_.getMapConfig();

    if (phase_ == Phase::AwaitingScan) {
        if (awaiting_planning_after_scan_ && latest_scan != nullptr) {
            awaiting_planning_after_scan_ = false;
            path_ = findFrontierPath(output_map_, state.position, drone_config_.radius);
            path_index_ = 0;

            if (path_.empty()) {
                finished_ = true;
                if (hasUnmappedInBounds(output_map_)) {
                    return makeFinishedCommand(types::AlgorithmStatus::FinishedWithUnmappableVoxels);
                }
                return makeFinishedCommand(types::AlgorithmStatus::Finished);
            }

            phase_ = Phase::Moving;
            types::MappingStepCommand command{};
            command.movement = movementToward(state, path_.front(), drone_config_);
            command.status = types::AlgorithmStatus::Working;
            return command;
        }

        awaiting_planning_after_scan_ = true;
        return makeScanCommand();
    }

    if (path_index_ < path_.size() && reachedWaypoint(state.position, path_[path_index_], config)) {
        ++path_index_;
        if (path_index_ >= path_.size()) {
            phase_ = Phase::AwaitingScan;
            awaiting_planning_after_scan_ = true;
            return makeScanCommand();
        }
    }

    if (path_index_ >= path_.size()) {
        finished_ = true;
        return makeFinishedCommand(types::AlgorithmStatus::Finished);
    }

    types::MappingStepCommand command{};
    command.movement = movementToward(state, path_[path_index_], drone_config_);
    command.status = types::AlgorithmStatus::Working;
    return command;
}

} // namespace drone_mapper
