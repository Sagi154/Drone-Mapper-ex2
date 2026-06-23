// MapsComparison.cpp
// Scores drone output maps against hidden reference maps using the ex1 Scorer
// union-of-known-cells model adapted to ex2 IMap3D / VoxelOccupancy.

#include <drone_mapper/MapsComparison.h>

#include <cmath>
#include <cstddef>
#include <unordered_set>
#include <vector>

namespace drone_mapper {

namespace {

struct GridKey {
    int x = 0;
    int y = 0;
    int z = 0;

    [[nodiscard]] bool operator==(const GridKey& other) const = default;
};

struct GridKeyHash {
    [[nodiscard]] std::size_t operator()(const GridKey& key) const noexcept {
        const std::size_t hx = static_cast<std::size_t>(key.x);
        const std::size_t hy = static_cast<std::size_t>(key.y);
        const std::size_t hz = static_cast<std::size_t>(key.z);
        return (hx * 73856093U) ^ (hy * 19349663U) ^ (hz * 83492791U);
    }
};

[[nodiscard]] bool isKnownOccupancy(const types::VoxelOccupancy value) {
    return value == types::VoxelOccupancy::Empty ||
           value == types::VoxelOccupancy::Occupied ||
           value == types::VoxelOccupancy::PotentiallyOccupied;
}

[[nodiscard]] GridKey quantizePosition(const Position3D& pos, const types::MapConfig& config) {
    const double step = config.resolution.force_numerical_value_in(cm);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);
    return GridKey{
        static_cast<int>(std::lround((px - ox) / step)),
        static_cast<int>(std::lround((py - oy) / step)),
        static_cast<int>(std::lround((pz - oz) / step)),
    };
}

void forEachGridCenter(const types::MapConfig& config, const auto& visitor) {
    const double step = config.resolution.force_numerical_value_in(cm);
    if (step <= 0.0) {
        return;
    }

    const types::MappingBounds& bounds = config.boundaries;
    const double min_x = bounds.min_x.force_numerical_value_in(cm);
    const double max_x = bounds.max_x.force_numerical_value_in(cm);
    const double min_y = bounds.min_y.force_numerical_value_in(cm);
    const double max_y = bounds.max_y.force_numerical_value_in(cm);
    const double min_z = bounds.min_height.force_numerical_value_in(cm);
    const double max_z = bounds.max_height.force_numerical_value_in(cm);

    for (double x = min_x; x <= max_x + 1e-9; x += step) {
        for (double y = min_y; y <= max_y + 1e-9; y += step) {
            for (double z = min_z; z <= max_z + 1e-9; z += step) {
                visitor(Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]});
            }
        }
    }
}

[[nodiscard]] double scoreSingleTarget(const IMap3D& reference, const IMap3D& produced) {
    const types::MapConfig ref_config = reference.getMapConfig();
    std::unordered_set<GridKey, GridKeyHash> ref_keys;
    ref_keys.reserve(256);

    std::size_t correct = 0;
    std::size_t total = 0;

    forEachGridCenter(ref_config, [&](const Position3D& pos) {
        if (!reference.isInBounds(pos)) {
            return;
        }

        const types::VoxelOccupancy ref_value = reference.atVoxel(pos);
        if (ref_value == types::VoxelOccupancy::OutOfBounds || !isKnownOccupancy(ref_value)) {
            return;
        }

        ref_keys.insert(quantizePosition(pos, ref_config));

        const types::VoxelOccupancy prod_value = produced.atVoxel(pos);
        if (prod_value == types::VoxelOccupancy::OutOfBounds) {
            return;
        }

        ++total;
        if (prod_value == ref_value) {
            ++correct;
        }
    });

    const types::MapConfig prod_config = produced.getMapConfig();
    forEachGridCenter(prod_config, [&](const Position3D& pos) {
        if (!produced.isInBounds(pos)) {
            return;
        }

        const types::VoxelOccupancy prod_value = produced.atVoxel(pos);
        if (prod_value == types::VoxelOccupancy::OutOfBounds || !isKnownOccupancy(prod_value)) {
            return;
        }

        if (ref_keys.contains(quantizePosition(pos, ref_config))) {
            return;
        }

        if (reference.atVoxel(pos) == types::VoxelOccupancy::OutOfBounds) {
            return;
        }

        ++total;
        if (prod_value == types::VoxelOccupancy::Empty) {
            ++correct;
        }
    });

    if (total == 0) {
        return 100.0;
    }

    return 100.0 * static_cast<double>(correct) / static_cast<double>(total);
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& original,
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());

    for (const IMap3D* target : targets) {
        scores.push_back(scoreSingleTarget(original, *target));
    }

    return scores;
}

} // namespace drone_mapper
