#pragma once

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/types/SimulationTypes.h>

namespace drone_mapper {

/// Mission-local initial pose from YAML is shifted by `map_offset.z` to hidden-map world Z.
[[nodiscard]] Position3D worldInitialDronePosition(const types::SimulationConfigData& simulation);

/// True when the drone sphere at `center` is in-bounds and no voxel within `radius` is Occupied.
[[nodiscard]] bool isDroneSpawnPassable(const IMap3D& hidden_map,
                                        PhysicalLength radius,
                                        const Position3D& center);

} // namespace drone_mapper
