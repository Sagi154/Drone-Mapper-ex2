#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/types/MissionTypes.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace drone_mapper::types {

struct LidarConfigData {
    PhysicalLength z_min{};
    PhysicalLength z_max{};
    PhysicalLength d{};
    std::size_t fov_circles = 0;
    std::optional<ErrorRef> config_load_error{};
};

struct LidarHit {
    // Misses use max double centimeters;
    PhysicalLength distance{};
    Orientation angle{}; // relative angle 
};

using LidarScanResult = std::vector<LidarHit>;

} // namespace drone_mapper::types
