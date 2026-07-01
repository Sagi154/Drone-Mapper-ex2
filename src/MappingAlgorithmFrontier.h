#pragma once

// Internal BFS frontier planner ported from ex1 ExplorationFrontier.
// Used only by MappingAlgorithmImpl — not part of the public API.

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Units.h>

#include <cstddef>
#include <unordered_set>
#include <vector>

namespace drone_mapper::detail {

struct GridKey {
    int qx = 0;
    int qy = 0;
    int qz = 0;

    [[nodiscard]] bool operator==(const GridKey& other) const noexcept {
        return qx == other.qx && qy == other.qy && qz == other.qz;
    }
};

struct GridKeyHash {
    [[nodiscard]] std::size_t operator()(const GridKey& key) const noexcept {
        const std::size_t hx = static_cast<std::size_t>(key.qx);
        const std::size_t hy = static_cast<std::size_t>(key.qy);
        const std::size_t hz = static_cast<std::size_t>(key.qz);
        return (hx * 73856093U) ^ (hy * 19349663U) ^ (hz * 83492791U);
    }
};

struct FrontierPathResult {
    bool found = false;
    std::vector<Position3D> path{};
};

struct PlanningDiagnostics {
    bool start_passable = false;
    std::size_t passable_reached = 0;
    std::size_t reachable_frontiers = 0;
    bool explore_path_available = false;
    int nearest_unknown_steps = -1;
};

/// BFS frontier search through confirmed-empty cells on a read-only output map.
class MappingAlgorithmFrontier {
public:
    [[nodiscard]] FrontierPathResult findPath(
        const IMap3D& map,
        const Position3D& start,
        PhysicalLength drone_radius,
        const std::unordered_set<GridKey, GridKeyHash>& blocked_cells = {}) const;

    /// Like findPath but targets the deepest frontier in the passable component.
    [[nodiscard]] FrontierPathResult findFarthestPath(
        const IMap3D& map,
        const Position3D& start,
        PhysicalLength drone_radius,
        const std::unordered_set<GridKey, GridKeyHash>& blocked_cells = {}) const;

    [[nodiscard]] FrontierPathResult findExplorePath(
        const IMap3D& map,
        const Position3D& start,
        PhysicalLength drone_radius,
        const std::unordered_set<GridKey, GridKeyHash>& blocked_cells = {}) const;

    [[nodiscard]] FrontierPathResult findUnstickPath(
        const IMap3D& map,
        const Position3D& start,
        PhysicalLength drone_radius) const;

    /// One grid step toward a passable neighbor with lower (or equal) unknown distance.
    [[nodiscard]] FrontierPathResult findGreedyUnknownStep(
        const IMap3D& map,
        const Position3D& start,
        PhysicalLength drone_radius,
        const std::unordered_set<GridKey, GridKeyHash>& blocked_cells = {}) const;

    /// Any single passable grid step — breaks deadlocks when unknown remains.
    [[nodiscard]] FrontierPathResult findAnyPassableNeighbor(
        const IMap3D& map,
        const Position3D& start,
        PhysicalLength drone_radius,
        const std::unordered_set<GridKey, GridKeyHash>& blocked_cells = {}) const;

    [[nodiscard]] PlanningDiagnostics diagnose(
        const IMap3D& map,
        const Position3D& start,
        PhysicalLength drone_radius,
        const std::unordered_set<GridKey, GridKeyHash>& blocked_cells = {}) const;
};

[[nodiscard]] GridKey quantizePosition(const Position3D& pos, const types::MapConfig& config);

[[nodiscard]] bool hasNotMappedInSphere(const IMap3D& map,
                                        const Position3D& centre,
                                        PhysicalLength radius);

[[nodiscard]] bool hasAnyNotMappedInBounds(const IMap3D& map);

} // namespace drone_mapper::detail
