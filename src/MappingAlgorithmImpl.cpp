// MappingAlgorithmImpl.cpp
// Frontier exploration state machine ported from ex1 DroneAlgorithm.
// Emits scan orientations and movement commands for DroneControlImpl to execute.

#include <drone_mapper/MappingAlgorithmImpl.h>

#include "MappingAlgorithmFrontier.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <unordered_set>

namespace drone_mapper {

namespace {

enum class ProximityMode {
    Normal,
    Expanded,
};

constexpr double kHalfStepTolerance = 0.5;
constexpr double kCoarsePassAngularScale = 2.0;
constexpr double kBeamLatticeHalfOffset = 0.5;
constexpr double kPositionEpsilon = 1e-6;
constexpr double kExpandedProximityRadiusFactor = 2.0;

[[nodiscard]] double gridStepCm(const types::MapConfig& config) {
    return config.resolution.force_numerical_value_in(cm);
}

[[nodiscard]] double computeProximityMaxCm(const types::DroneConfigData& drone,
                                           const types::LidarConfigData& lidar,
                                           const types::MapConfig& map_config,
                                           ProximityMode mode,
                                           int expanded_rescue_pass) {
    const double radius_cm = drone.radius.force_numerical_value_in(cm);
    const double z_max_cm = lidar.z_max.force_numerical_value_in(cm);
    const double cell_cm = gridStepCm(map_config);
    const double normal_cm = (std::ceil(radius_cm / cell_cm) + 1.0) * cell_cm;
    if (mode == ProximityMode::Normal) {
        return std::min(z_max_cm, normal_cm);
    }
    const int pass = std::max(1, expanded_rescue_pass);
    const double expanded_cm =
        (kExpandedProximityRadiusFactor * radius_cm) +
        (static_cast<double>(pass - 1) * radius_cm);
    return std::min(z_max_cm, expanded_cm);
}

} // namespace

struct MappingAlgorithmImpl::Impl {
    Phase phase = Phase::Scanning;
    detail::MappingAlgorithmFrontier frontier{};

    std::vector<Orientation> scan_orientations{};
    std::size_t scan_index = 0;
    int scan_pass = 0;
    bool last_scan_made_progress = true;
    std::size_t cells_before_scan = 0;

    std::vector<Position3D> current_path{};
    std::size_t path_index = 0;
    int moving_stall_ticks = 0;
    Position3D last_position{};
    bool has_last_position = false;

    std::unordered_set<detail::GridKey, detail::GridKeyHash> blocked_cells{};
    bool finished = false;
};

MappingAlgorithmImpl::~MappingAlgorithmImpl() = default;

MappingAlgorithmImpl::MappingAlgorithmImpl(const types::MissionConfigData& mission_config,
                                           const types::LidarConfigData& lidar_config,
                                           const types::DroneConfigData& drone_config,
                                           const IMap3D& output_map)
    : IMappingAlgorithm(mission_config, lidar_config, drone_config, output_map),
      impl_(std::make_unique<Impl>()) {}

std::size_t MappingAlgorithmImpl::countMappedCells() const {
    const types::MapConfig config = output_map_.getMapConfig();
    const double step = gridStepCm(config);
    if (step <= 0.0) {
        return 0;
    }
    const types::MappingBounds& bounds = config.boundaries;

    const double min_x = bounds.min_x.force_numerical_value_in(cm);
    const double max_x = bounds.max_x.force_numerical_value_in(cm);
    const double min_y = bounds.min_y.force_numerical_value_in(cm);
    const double max_y = bounds.max_y.force_numerical_value_in(cm);
    const double min_z = bounds.min_height.force_numerical_value_in(cm);
    const double max_z = bounds.max_height.force_numerical_value_in(cm);

    std::size_t count = 0;
    for (double x = min_x; x <= max_x + 1e-9; x += step) {
        for (double y = min_y; y <= max_y + 1e-9; y += step) {
            for (double z = min_z; z <= max_z + 1e-9; z += step) {
                const Position3D pos{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
                if (!output_map_.isInBounds(pos)) {
                    continue;
                }
                if (output_map_.atVoxel(pos) != types::VoxelOccupancy::Unmapped) {
                    ++count;
                }
            }
        }
    }
    return count;
}

void MappingAlgorithmImpl::buildScanOrientations() {
    impl_->scan_orientations.clear();

    const types::MapConfig map_config = output_map_.getMapConfig();
    const double cell_cm = gridStepCm(map_config);
    const double radius_cm = drone_config_.radius.force_numerical_value_in(cm);
    const ProximityMode mode =
        (impl_->scan_pass == 0) ? ProximityMode::Normal : ProximityMode::Expanded;

    const double denom = (radius_cm > 0.0) ? radius_cm : cell_cm;
    const double el_step_base = std::atan(cell_cm / denom) * (180.0 / std::numbers::pi);

    double angular_scale = 1.0;
    double az_start = 0.0;
    double el_start = -90.0;
    if (impl_->scan_pass == 0 && mode == ProximityMode::Normal) {
        angular_scale = kCoarsePassAngularScale;
    } else {
        const int pass_offset = (impl_->scan_pass <= 0) ? 0 : impl_->scan_pass - 1;
        if (pass_offset == 1) {
            const double base_el_step = el_step_base;
            az_start = base_el_step * kBeamLatticeHalfOffset;
        } else if (pass_offset >= 2) {
            const double base_el_step = el_step_base;
            el_start = -90.0 + base_el_step * kBeamLatticeHalfOffset;
        }
    }

    const double el_step = el_step_base * angular_scale;
    std::vector<double> elevations;
    elevations.reserve(static_cast<std::size_t>((180.0 / el_step) + 3.0));
    for (double el = el_start; el <= 90.0 + 1e-9; el += el_step) {
        elevations.push_back(std::clamp(el, -90.0, 90.0));
    }
    if (std::find_if(elevations.begin(), elevations.end(),
                     [](double el) { return std::abs(el) < 1e-6; }) == elevations.end()) {
        elevations.push_back(0.0);
    }
    std::sort(elevations.begin(), elevations.end());

    for (const double el_clamped : elevations) {
        const double cos_el = std::cos(el_clamped * (std::numbers::pi / 180.0));
        const double az_step = (cos_el > 1e-6) ? (el_step / cos_el) : 360.0;
        for (double az = az_start; az < az_start + 360.0; az += az_step) {
            impl_->scan_orientations.push_back(Orientation{az * deg, el_clamped * deg});
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
        types::MovementCommand cmd{};
        cmd.type = types::MovementCommandType::Elevate;
        cmd.distance = std::clamp(dh, -limit, limit) * cm;
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
            (delta > 0.0) ? types::RotationDirection::Right : types::RotationDirection::Left;
        cmd.angle = std::min(std::abs(delta), rot_limit) * deg;
        return cmd;
    }

    const double dist_cm = std::sqrt(dx * dx + dy * dy);
    const double adv_limit = drone_config_.max_advance.force_numerical_value_in(cm);
    types::MovementCommand cmd{};
    cmd.type = types::MovementCommandType::Advance;
    cmd.distance = std::min(dist_cm, adv_limit) * cm;
    return cmd;
}

types::MappingStepCommand MappingAlgorithmImpl::handleScanningPhase(const types::DroneState& state) {
    (void)state;
    if (impl_->scan_orientations.empty()) {
        impl_->cells_before_scan = countMappedCells();
        buildScanOrientations();
        impl_->scan_index = 0;
    }

    if (impl_->scan_index < impl_->scan_orientations.size()) {
        types::MappingStepCommand cmd{};
        cmd.scan_orientation = impl_->scan_orientations[impl_->scan_index++];
        cmd.status = types::AlgorithmStatus::Working;
        return cmd;
    }

    const std::size_t cells_after = countMappedCells();
    impl_->last_scan_made_progress = (cells_after > impl_->cells_before_scan);
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

        const types::MapConfig map_config = output_map_.getMapConfig();
        const double normal_cm = computeProximityMaxCm(
            drone_config_, lidar_config_, map_config, ProximityMode::Normal, 1);
        const int next_rescue_pass = std::max(1, impl_->scan_pass + 1);
        const double expanded_cm = computeProximityMaxCm(
            drone_config_, lidar_config_, map_config, ProximityMode::Expanded, next_rescue_pass);

        const bool local_unknown_normal =
            detail::hasNotMappedInSphere(output_map_, state.position, normal_cm * cm);
        const bool local_unknown_expanded =
            detail::hasNotMappedInSphere(output_map_, state.position, expanded_cm * cm);
        const bool mission_has_unknown = detail::hasAnyNotMappedInBounds(output_map_);
        const bool can_retry = impl_->scan_pass < kMaxScanPassIndex &&
                               (impl_->scan_pass == 0 || impl_->last_scan_made_progress);
        const bool should_rescan =
            can_retry &&
            ((impl_->scan_pass == 0 && (local_unknown_normal || mission_has_unknown)) ||
             (impl_->scan_pass == 1 && local_unknown_expanded) ||
             (impl_->scan_pass == 2 && local_unknown_expanded && local_unknown_normal));

        if (should_rescan) {
            ++impl_->scan_pass;
            impl_->phase = Phase::Scanning;
            return handleScanningPhase(state);
        }

        if (mission_has_unknown && diag.explore_path_available) {
            const detail::FrontierPathResult explore = impl_->frontier.findExplorePath(
                output_map_, state.position, drone_config_.radius, impl_->blocked_cells);
            if (explore.found && !explore.path.empty()) {
                impl_->current_path = explore.path;
                impl_->path_index = 0;
                impl_->scan_pass = 0;
                impl_->moving_stall_ticks = 0;
                impl_->phase = Phase::Moving;
                return handleMovingPhase(state);
            }
        }

        if (mission_has_unknown && !diag.start_passable) {
            const detail::FrontierPathResult unstick =
                impl_->frontier.findUnstickPath(output_map_, state.position, drone_config_.radius);
            if (unstick.found && !unstick.path.empty()) {
                impl_->current_path = unstick.path;
                impl_->path_index = 0;
                impl_->scan_pass = 0;
                impl_->moving_stall_ticks = 0;
                impl_->phase = Phase::Moving;
                return handleMovingPhase(state);
            }
        }

        impl_->finished = true;
        types::MappingStepCommand cmd{};
        cmd.status = mission_has_unknown ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                         : types::AlgorithmStatus::Finished;
        return cmd;
    }

    impl_->current_path = result.path;
    impl_->path_index = 0;
    impl_->scan_pass = 0;
    impl_->moving_stall_ticks = 0;
    impl_->phase = Phase::Moving;
    return handleMovingPhase(state);
}

types::MappingStepCommand MappingAlgorithmImpl::handleMovingPhase(const types::DroneState& state) {
    if (impl_->path_index >= impl_->current_path.size()) {
        impl_->scan_pass = 0;
        impl_->phase = Phase::Scanning;
        return handleScanningPhase(state);
    }

    if (reachedWaypoint(state, impl_->current_path[impl_->path_index])) {
        ++impl_->path_index;
        impl_->moving_stall_ticks = 0;
        if (impl_->path_index >= impl_->current_path.size()) {
            impl_->scan_pass = 0;
            impl_->phase = Phase::Scanning;
            return handleScanningPhase(state);
        }
    } else if (impl_->has_last_position && samePosition(impl_->last_position, state.position)) {
        ++impl_->moving_stall_ticks;
        if (impl_->moving_stall_ticks >= kMaxMovingStallTicks) {
            impl_->blocked_cells.insert(detail::quantizePosition(
                impl_->current_path[impl_->path_index], output_map_.getMapConfig()));
            impl_->moving_stall_ticks = 0;
            impl_->scan_pass = 0;
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
