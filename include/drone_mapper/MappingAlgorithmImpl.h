#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <memory>
#include <optional>
#include <vector>

namespace drone_mapper {

/// Frontier-based exploration algorithm ported from ex1 DroneAlgorithm.
/// Each nextStep emits at most one scan orientation or one movement command.
class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    /// @param mission_config Mission limits such as max_steps (used by MissionControl).
    /// @param lidar_config LiDAR geometry for scan sweep density.
    /// @param drone_config Drone radius and per-command movement limits.
    /// @param output_map Read-only discovered map used for frontier planning.
    MappingAlgorithmImpl(const types::MissionConfigData& mission_config,
                         const types::LidarConfigData& lidar_config,
                         const types::DroneConfigData& drone_config,
                         const IMap3D& output_map);

    /// Returns the next scan orientation and/or movement for DroneControl to execute.
    /// @param state Current drone pose from GPS.
    /// @param latest_scan Previous scan result, or nullptr on the first step.
    /// @return Command with optional movement and/or scan_orientation.
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

    ~MappingAlgorithmImpl() override;

    MappingAlgorithmImpl(const MappingAlgorithmImpl&) = delete;
    MappingAlgorithmImpl& operator=(const MappingAlgorithmImpl&) = delete;
    MappingAlgorithmImpl(MappingAlgorithmImpl&&) noexcept;
    MappingAlgorithmImpl& operator=(MappingAlgorithmImpl&&) noexcept;

private:
    enum class Phase { Scanning, Planning, Moving };

    struct Impl;

    [[nodiscard]] types::MappingStepCommand handleScanningPhase(const types::DroneState& state);
    [[nodiscard]] types::MappingStepCommand handlePlanningPhase(const types::DroneState& state);
    [[nodiscard]] types::MappingStepCommand handleMovingPhase(const types::DroneState& state);

    void buildScanOrientations();
    [[nodiscard]] std::size_t countMappedCells() const;
    [[nodiscard]] std::optional<types::MovementCommand> movementToward(
        const types::DroneState& state, const Position3D& target) const;
    [[nodiscard]] bool reachedWaypoint(const types::DroneState& state,
                                       const Position3D& target) const;
    [[nodiscard]] bool samePosition(const Position3D& a, const Position3D& b) const;

    std::unique_ptr<Impl> impl_;

    static constexpr int kMaxScanPassIndex = 2;
    static constexpr int kMaxMovingStallTicks = 8;
};

} // namespace drone_mapper
