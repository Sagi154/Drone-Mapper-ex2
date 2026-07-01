// MappingAlgorithmImpl.cpp
// Frontier exploration: full-range 26-direction scan + BFS frontier planning.

#include <drone_mapper/MappingAlgorithmImpl.h>

#include "MappingAlgorithmFrontier.h"

#include <drone_mapper/IMutableMap3D.h>

#include <cmath>
#include <numbers>
#include <optional>
#include <unordered_set>
#include <utility>

namespace drone_mapper {

namespace {

constexpr double kHalfStepTolerance = 0.5;
constexpr double kPositionEpsilon = 1e-6;

[[nodiscard]] double gridStepCm(const types::MapConfig& config) {
    return config.resolution.force_numerical_value_in(cm);
}

} // namespace

struct MappingAlgorithmImpl::Impl {
    Phase phase = Phase::Scanning;
    detail::MappingAlgorithmFrontier frontier{};

    std::vector<Orientation> scan_orientations{};
    std::size_t scan_index = 0;

    std::vector<Position3D> current_path{};
    std::size_t path_index = 0;
    int moving_stall_ticks = 0;
    Position3D last_position{};
    bool has_last_position = false;

    std::unordered_set<detail::GridKey, detail::GridKeyHash> blocked_cells{};
    bool finished = false;
    IMutableMap3D* mutable_output = nullptr;
};

MappingAlgorithmImpl::~MappingAlgorithmImpl() = default;

MappingAlgorithmImpl::MappingAlgorithmImpl(const types::MissionConfigData& mission_config,
                                           const types::LidarConfigData& lidar_config,
                                           const types::DroneConfigData& drone_config,
                                           const IMap3D& output_map)
    : IMappingAlgorithm(mission_config, lidar_config, drone_config, output_map),
      impl_(std::make_unique<Impl>()) {
    impl_->mutable_output =
        dynamic_cast<IMutableMap3D*>(const_cast<IMap3D*>(&output_map));
}

void MappingAlgorithmImpl::buildScanOrientations(const Orientation& heading) {
    impl_->scan_orientations.clear();
    impl_->scan_orientations.reserve(26);

    const auto addDirection = [&](double dx, double dy, double dz) {
        const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-9) {
            return;
        }
        dx /= len;
        dy /= len;
        dz /= len;

        const double az_deg = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
        const double horiz_len = std::sqrt(dx * dx + dy * dy);
        const double el_deg = std::atan2(dz, horiz_len) * (180.0 / std::numbers::pi);

        double az_norm = az_deg;
        while (az_norm < 0.0) {
            az_norm += 360.0;
        }
        while (az_norm >= 360.0) {
            az_norm -= 360.0;
        }

        impl_->scan_orientations.push_back(
            Orientation{az_norm * deg - heading.horizontal, el_deg * deg - heading.altitude});
    };

    addDirection(1.0, 0.0, 0.0);
    addDirection(-1.0, 0.0, 0.0);
    addDirection(0.0, 1.0, 0.0);
    addDirection(0.0, -1.0, 0.0);
    addDirection(0.0, 0.0, 1.0);
    addDirection(0.0, 0.0, -1.0);

    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    for (const int sx : {-1, 1}) {
        for (const int sy : {-1, 1}) {
            addDirection(static_cast<double>(sx) * inv_sqrt2,
                         static_cast<double>(sy) * inv_sqrt2,
                         0.0);
        }
    }
    for (const int sx : {-1, 1}) {
        for (const int sz : {-1, 1}) {
            addDirection(static_cast<double>(sx) * inv_sqrt2,
                         0.0,
                         static_cast<double>(sz) * inv_sqrt2);
        }
    }
    for (const int sy : {-1, 1}) {
        for (const int sz : {-1, 1}) {
            addDirection(0.0,
                         static_cast<double>(sy) * inv_sqrt2,
                         static_cast<double>(sz) * inv_sqrt2);
        }
    }

    const double inv_sqrt3 = 1.0 / std::sqrt(3.0);
    for (const int sx : {-1, 1}) {
        for (const int sy : {-1, 1}) {
            for (const int sz : {-1, 1}) {
                addDirection(static_cast<double>(sx) * inv_sqrt3,
                             static_cast<double>(sy) * inv_sqrt3,
                             static_cast<double>(sz) * inv_sqrt3);
            }
        }
    }
}

bool MappingAlgorithmImpl::samePosition(const Position3D& a, const Position3D& b) const {
    const double dx = std::abs(a.x.force_numerical_value_in(cm) - b.x.force_numerical_value_in(cm));
    const double dy = std::abs(a.y.force_numerical_value_in(cm) - b.y.force_numerical_value_in(cm));
    const double dz = std::abs(a.z.force_numerical_value_in(cm) - b.z.force_numerical_value_in(cm));
    return dx <= kPositionEpsilon && dy <= kPositionEpsilon && dz <= kPositionEpsilon;
}

bool MappingAlgorithmImpl::reachedWaypoint(const types::DroneState& state,
                                           const Position3D& target) const {
    const double step = gridStepCm(output_map_.getMapConfig());
    const double dx = std::abs(state.position.x.force_numerical_value_in(cm) -
                               target.x.force_numerical_value_in(cm));
    const double dy = std::abs(state.position.y.force_numerical_value_in(cm) -
                               target.y.force_numerical_value_in(cm));
    const double dz = std::abs(state.position.z.force_numerical_value_in(cm) -
                               target.z.force_numerical_value_in(cm));
    return dx <= step * kHalfStepTolerance && dy <= step * kHalfStepTolerance &&
           dz <= step * kHalfStepTolerance;
}

std::optional<types::MovementCommand> MappingAlgorithmImpl::movementToward(
    const types::DroneState& state, const Position3D& target) const {
    const double dh =
        target.z.force_numerical_value_in(cm) - state.position.z.force_numerical_value_in(cm);
    if (std::abs(dh) > 1e-6) {
        const double limit = drone_config_.max_elevate.force_numerical_value_in(cm);
        const double step_cm = gridStepCm(output_map_.getMapConfig());
        const double step_cap = step_cm > 0.0 ? step_cm : limit;
        const double capped = std::min(limit, step_cap);
        types::MovementCommand cmd{};
        cmd.type = types::MovementCommandType::Elevate;
        cmd.distance = std::clamp(dh, -capped, capped) * cm;
        return cmd;
    }

    const double dx =
        target.x.force_numerical_value_in(cm) - state.position.x.force_numerical_value_in(cm);
    const double dy =
        target.y.force_numerical_value_in(cm) - state.position.y.force_numerical_value_in(cm);
    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
        types::MovementCommand cmd{};
        cmd.type = types::MovementCommandType::Hover;
        return cmd;
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

    const double rot_limit = drone_config_.max_rotate.force_numerical_value_in(deg);
    if (std::abs(delta) > 1e-6) {
        types::MovementCommand cmd{};
        cmd.type = types::MovementCommandType::Rotate;
        cmd.rotation =
            (delta > 0.0) ? types::RotationDirection::Left : types::RotationDirection::Right;
        cmd.angle = std::min(std::abs(delta), rot_limit) * deg;
        return cmd;
    }

    const double dist_cm = std::sqrt(dx * dx + dy * dy);
    const double adv_limit = drone_config_.max_advance.force_numerical_value_in(cm);
    const double step_cm = gridStepCm(output_map_.getMapConfig());
    const double step_cap = step_cm > 0.0 ? step_cm : adv_limit;
    types::MovementCommand cmd{};
    cmd.type = types::MovementCommandType::Advance;
    cmd.distance = std::min(dist_cm, std::min(adv_limit, step_cap)) * cm;
    return cmd;
}

types::MappingStepCommand MappingAlgorithmImpl::handleScanningPhase(const types::DroneState& state) {
    if (impl_->scan_orientations.empty()) {
        buildScanOrientations(state.heading);
        impl_->scan_index = 0;
    }

    if (impl_->scan_index < impl_->scan_orientations.size()) {
        types::MappingStepCommand cmd{};
        cmd.scan_orientation = impl_->scan_orientations[impl_->scan_index++];
        cmd.status = types::AlgorithmStatus::Working;
        return cmd;
    }

    impl_->scan_orientations.clear();
    impl_->scan_index = 0;
    impl_->phase = Phase::Planning;
    return handlePlanningPhase(state);
}

types::MappingStepCommand MappingAlgorithmImpl::handlePlanningPhase(const types::DroneState& state) {
    const detail::FrontierPathResult result = impl_->frontier.findPath(
        output_map_, state.position, drone_config_.radius, impl_->blocked_cells);

    if (!result.found) {
        const detail::PlanningDiagnostics diag = impl_->frontier.diagnose(
            output_map_, state.position, drone_config_.radius, impl_->blocked_cells);

        const bool mission_has_unknown = detail::hasAnyNotMappedInBounds(output_map_);

        if (!mission_has_unknown) {
            impl_->finished = true;
            types::MappingStepCommand cmd{};
            cmd.status = types::AlgorithmStatus::Finished;
            return cmd;
        }

        if (diag.explore_path_available) {
            const detail::FrontierPathResult explore = impl_->frontier.findExplorePath(
                output_map_, state.position, drone_config_.radius, impl_->blocked_cells);
            if (explore.found && !explore.path.empty()) {
                impl_->current_path = explore.path;
                impl_->path_index = 0;
                impl_->moving_stall_ticks = 0;
                impl_->phase = Phase::Moving;
                return handleMovingPhase(state);
            }
        }

        if (!diag.start_passable) {
            const detail::FrontierPathResult unstick =
                impl_->frontier.findUnstickPath(output_map_, state.position, drone_config_.radius);
            if (unstick.found && !unstick.path.empty()) {
                impl_->current_path = unstick.path;
                impl_->path_index = 0;
                impl_->moving_stall_ticks = 0;
                impl_->phase = Phase::Moving;
                return handleMovingPhase(state);
            }
        }

        const detail::FrontierPathResult wander = impl_->frontier.findAnyPassableNeighbor(
            output_map_, state.position, drone_config_.radius, impl_->blocked_cells);
        if (wander.found && !wander.path.empty()) {
            impl_->current_path = wander.path;
            impl_->path_index = 0;
            impl_->moving_stall_ticks = 0;
            impl_->phase = Phase::Moving;
            return handleMovingPhase(state);
        }

        impl_->finished = true;
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::FinishedWithUnmappableVoxels;
        return cmd;
    }

    impl_->current_path = result.path;
    impl_->path_index = 0;
    impl_->moving_stall_ticks = 0;
    impl_->phase = Phase::Moving;
    return handleMovingPhase(state);
}

types::MappingStepCommand MappingAlgorithmImpl::handleMovingPhase(const types::DroneState& state) {
    if (impl_->path_index >= impl_->current_path.size()) {
        impl_->phase = Phase::Scanning;
        return handleScanningPhase(state);
    }

    if (reachedWaypoint(state, impl_->current_path[impl_->path_index])) {
        ++impl_->path_index;
        impl_->moving_stall_ticks = 0;
        if (impl_->path_index >= impl_->current_path.size()) {
            impl_->phase = Phase::Scanning;
            return handleScanningPhase(state);
        }
    } else if (impl_->has_last_position && samePosition(impl_->last_position, state.position)) {
        ++impl_->moving_stall_ticks;
        if (impl_->moving_stall_ticks >= kMaxMovingStallTicks) {
            const auto blocked_key = detail::quantizePosition(
                impl_->current_path[impl_->path_index], output_map_.getMapConfig());
            impl_->blocked_cells.insert(blocked_key);
            if (impl_->mutable_output != nullptr) {
                impl_->mutable_output->set(impl_->current_path[impl_->path_index],
                                           types::VoxelOccupancy::Occupied);
            }
            impl_->moving_stall_ticks = 0;
            impl_->phase = Phase::Planning;
            return handlePlanningPhase(state);
        }
    } else {
        impl_->moving_stall_ticks = 0;
    }

    impl_->last_position = state.position;
    impl_->has_last_position = true;

    types::MappingStepCommand cmd{};
    cmd.movement = movementToward(state, impl_->current_path[impl_->path_index]);
    cmd.status = types::AlgorithmStatus::Working;
    return cmd;
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    (void)latest_scan;

    if (impl_->finished) {
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::Finished;
        return cmd;
    }

    switch (impl_->phase) {
    case Phase::Scanning:
        return handleScanningPhase(state);
    case Phase::Planning:
        return handlePlanningPhase(state);
    case Phase::Moving:
        return handleMovingPhase(state);
    }

    types::MappingStepCommand cmd{};
    cmd.status = types::AlgorithmStatus::Finished;
    return cmd;
}

} // namespace drone_mapper
