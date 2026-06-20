#include <drone_mapper/Map3DImpl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

namespace {

[[nodiscard]] bool isZeroResolution(const PhysicalLength resolution) {
    return resolution <= 0.0 * cm;
}

[[nodiscard]] bool isUnsetBoundaries(const types::MappingBounds& bounds) {
    return bounds.min_x == 0.0 * x_extent[cm] && bounds.max_x == 0.0 * x_extent[cm] &&
           bounds.min_y == 0.0 * y_extent[cm] && bounds.max_y == 0.0 * y_extent[cm] &&
           bounds.min_height == 0.0 * z_extent[cm] && bounds.max_height == 0.0 * z_extent[cm];
}

[[nodiscard]] types::MappingBounds deriveBoundsFromShape(const NpyArray& map,
                                                         const Position3D& offset,
                                                         const PhysicalLength resolution) {
    types::MappingBounds bounds{};
    if (map.IsEmpty() || map.Shape().size() != 3 || isZeroResolution(resolution)) {
        return bounds;
    }

    const auto span = [&](std::size_t voxel_count) {
        if (voxel_count == 0) {
            return 0.0;
        }
        return static_cast<double>(voxel_count - 1) * resolution.force_numerical_value_in(cm);
    };

    const double max_x = offset.x.force_numerical_value_in(cm) + span(map.Shape()[0]);
    const double max_y = offset.y.force_numerical_value_in(cm) + span(map.Shape()[1]);
    const double max_z = offset.z.force_numerical_value_in(cm) + span(map.Shape()[2]);

    bounds.min_x = offset.x;
    bounds.max_x = max_x * x_extent[cm];
    bounds.min_y = offset.y;
    bounds.max_y = max_y * y_extent[cm];
    bounds.min_height = offset.z;
    bounds.max_height = max_z * z_extent[cm];
    return bounds;
}

[[nodiscard]] types::MapConfig finalizeConfig(const NpyArray& map, types::MapConfig config) {
    if (isUnsetBoundaries(config.boundaries)) {
        config.boundaries = deriveBoundsFromShape(map, config.offset, config.resolution);
    }
    return config;
}

[[nodiscard]] bool isWithinMappingBounds(const Position3D& pos, const types::MappingBounds& bounds) {
    return pos.x >= bounds.min_x && pos.x <= bounds.max_x && pos.y >= bounds.min_y &&
           pos.y <= bounds.max_y && pos.z >= bounds.min_height && pos.z <= bounds.max_height;
}

[[nodiscard]] std::optional<std::array<std::size_t, 3>>
toVoxelIndex(const Position3D& pos, const types::MapConfig& config) {
    if (isZeroResolution(config.resolution)) {
        return std::nullopt;
    }

    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const auto toIndex = [&](auto coordinate, auto offset) -> std::optional<std::size_t> {
        const double relative_cm = (coordinate - offset).force_numerical_value_in(cm);
        if (relative_cm < 0.0) {
            return std::nullopt;
        }
        const double raw_index = relative_cm / resolution_cm;
        const auto index = static_cast<std::size_t>(raw_index);
        if (std::fabs(raw_index - static_cast<double>(index)) > 1e-9) {
            return std::nullopt;
        }
        return index;
    };

    const auto ix = toIndex(pos.x, config.offset.x);
    const auto iy = toIndex(pos.y, config.offset.y);
    const auto iz = toIndex(pos.z, config.offset.z);
    if (!ix || !iy || !iz) {
        return std::nullopt;
    }

    return std::array<std::size_t, 3>{*ix, *iy, *iz};
}

[[nodiscard]] bool isIndexWithinShape(const NpyArray& map, const std::array<std::size_t, 3>& index) {
    if (map.IsEmpty() || map.Shape().size() != 3) {
        return false;
    }
    return index[0] < map.Shape()[0] && index[1] < map.Shape()[1] && index[2] < map.Shape()[2];
}

[[nodiscard]] std::size_t linearIndex(const NpyArray& map, const std::array<std::size_t, 3>& index) {
    return index[0] * (map.Shape()[1] * map.Shape()[2]) + index[1] * map.Shape()[2] + index[2];
}

[[nodiscard]] types::VoxelOccupancy occupancyFromStored(std::int64_t stored_value) {
    switch (stored_value) {
    case static_cast<std::int64_t>(types::VoxelOccupancy::PotentiallyOccupied):
    case static_cast<std::int64_t>(types::VoxelOccupancy::OutOfBounds):
    case static_cast<std::int64_t>(types::VoxelOccupancy::Unmapped):
    case static_cast<std::int64_t>(types::VoxelOccupancy::Empty):
    case static_cast<std::int64_t>(types::VoxelOccupancy::Occupied):
        return static_cast<types::VoxelOccupancy>(stored_value);
    default:
        return types::VoxelOccupancy::Unmapped;
    }
}

[[nodiscard]] types::VoxelOccupancy readStoredValue(const NpyArray& map, std::size_t linear_index) {
    if (map.SizeValueBytes() == 1) {
        return occupancyFromStored(static_cast<std::int64_t>(map.Data<std::uint8_t>()[linear_index]));
    }
    if (map.SizeValueBytes() == sizeof(std::int8_t)) {
        return occupancyFromStored(static_cast<std::int64_t>(map.Data<std::int8_t>()[linear_index]));
    }
    return types::VoxelOccupancy::Unmapped;
}

void writeStoredValue(NpyArray& map, std::size_t linear_index, types::VoxelOccupancy value) {
    const auto stored = static_cast<std::int8_t>(value);
    if (map.SizeValueBytes() == 1) {
        map.Data<std::uint8_t>()[linear_index] = static_cast<std::uint8_t>(stored);
        return;
    }
    if (map.SizeValueBytes() == sizeof(std::int8_t)) {
        map.Data<std::int8_t>()[linear_index] = stored;
    }
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_ ? finalizeConfig(*map_, map_config) : map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    if (!isInBounds(pos)) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    const std::optional<std::array<std::size_t, 3>> index = toVoxelIndex(pos, config_);
    if (!index || !isIndexWithinShape(*map_, *index)) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    return readStoredValue(*map_, linearIndex(*map_, *index));
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    if (!isWithinMappingBounds(pos, config_.boundaries)) {
        return false;
    }

    const std::optional<std::array<std::size_t, 3>> index = toVoxelIndex(pos, config_);
    if (!index) {
        return false;
    }

    return isIndexWithinShape(*map_, *index);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    if (!isInBounds(pos)) {
        return;
    }

    const std::array<std::size_t, 3> index = *toVoxelIndex(pos, config_);
    writeStoredValue(*map_, linearIndex(*map_, index), value);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    if (map_->IsEmpty()) {
        throw std::runtime_error("Cannot save an empty map.");
    }

    const LPCSTR error = map_->SaveNPY(path.string());
    if (error != nullptr) {
        throw std::runtime_error(error);
    }
}

} // namespace drone_mapper
