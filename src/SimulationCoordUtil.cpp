#include <drone_mapper/SimulationCoordUtil.h>

#include <cmath>

namespace drone_mapper {

namespace {

bool sphereCollides(const IMap3D& map, const Position3D& center, PhysicalLength radius) {
    const types::MapConfig cfg = map.getMapConfig();
    const double res_cm = cfg.resolution.numerical_value_in(cm);
    if (res_cm <= 0.0) {
        return false;
    }
    const double r_cm = radius.numerical_value_in(cm);
    const double cx = center.x.numerical_value_in(cm);
    const double cy = center.y.numerical_value_in(cm);
    const double cz = center.z.numerical_value_in(cm);

    const int steps = static_cast<int>(std::ceil(r_cm / res_cm));
    const double r2 = r_cm * r_cm;

    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const double ox = dx * res_cm;
                const double oy = dy * res_cm;
                const double oz = dz * res_cm;
                if (ox * ox + oy * oy + oz * oz > r2) {
                    continue;
                }
                const Position3D sample{
                    (cx + ox) * x_extent[cm],
                    (cy + oy) * y_extent[cm],
                    (cz + oz) * z_extent[cm],
                };
                if (!map.isInBounds(sample)) {
                    return true;
                }
                if (map.atVoxel(sample) == types::VoxelOccupancy::Occupied) {
                    return true;
                }
            }
        }
    }
    return false;
}

} // namespace

Position3D worldInitialDronePosition(const types::SimulationConfigData& simulation) {
    Position3D world = simulation.initial_drone_position;
    world.z = world.z + simulation.map_offset.z;
    return world;
}

bool isDroneSpawnPassable(const IMap3D& hidden_map,
                          PhysicalLength radius,
                          const Position3D& center) {
    return !sphereCollides(hidden_map, center, radius);
}

} // namespace drone_mapper
