#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/io/RunErrorLog.h>
#include <drone_mapper/io/RunPathHelpers.h>

#include <TinyNPY.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace drone_mapper {

namespace {

[[nodiscard]] types::MapConfig hiddenMapConfig(const types::SimulationConfigData& simulation) {
    return types::MapConfig{
        .boundaries = {},
        .offset = simulation.map_offset,
        .resolution = simulation.map_resolution,
    };
}

[[nodiscard]] types::MapConfig outputMapConfig(const types::SimulationConfigData& simulation,
                                               const types::MissionConfigData& mission,
                                               const Map3DImpl& hidden_map) {
    types::MapConfig config = hidden_map.getMapConfig();
    const double factor = mission.output_mapping_resolution_factor >= 1.0
                              ? mission.output_mapping_resolution_factor
                              : 1.0;
    const double hidden_cm = simulation.map_resolution.force_numerical_value_in(cm);
    config.resolution = (hidden_cm / factor) * cm;
    return config;
}

[[nodiscard]] std::size_t voxelAxisCount(PhysicalLength min_bound,
                                         PhysicalLength max_bound,
                                         PhysicalLength resolution) {
    const double res_cm = resolution.force_numerical_value_in(cm);
    if (res_cm <= 0.0) {
        return 0;
    }
    const double span_cm = (max_bound - min_bound).force_numerical_value_in(cm);
    if (span_cm < 0.0) {
        return 0;
    }
    return static_cast<std::size_t>(std::lround(span_cm / res_cm)) + 1;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyOutputArray(const types::MapConfig& config) {
    const std::size_t nx =
        voxelAxisCount(config.boundaries.min_x, config.boundaries.max_x, config.resolution);
    const std::size_t ny =
        voxelAxisCount(config.boundaries.min_y, config.boundaries.max_y, config.resolution);
    const std::size_t nz = voxelAxisCount(config.boundaries.min_height,
                                          config.boundaries.max_height,
                                          config.resolution);
    if (nx == 0 || ny == 0 || nz == 0) {
        return std::make_shared<NpyArray>();
    }

    auto map = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz},
                                          sizeof(std::int8_t),
                                          NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(),
                map->NumValue(),
                static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return map;
}

[[nodiscard]] std::filesystem::path resolveMapPath(const std::filesystem::path& map_filename) {
    if (map_filename.empty()) {
        return map_filename;
    }
    if (std::filesystem::exists(map_filename)) {
        return map_filename;
    }
    if (map_filename.is_absolute()) {
        return map_filename;
    }
    const std::filesystem::path relative_to_cwd = std::filesystem::current_path() / map_filename;
    if (std::filesystem::exists(relative_to_cwd)) {
        return relative_to_cwd;
    }
    return map_filename;
}

struct HiddenMapLoadResult {
    std::unique_ptr<Map3DImpl> map;
    std::optional<types::ErrorRef> error;
};

[[nodiscard]] HiddenMapLoadResult loadHiddenMap(const types::SimulationConfigData& simulation) {
    const std::filesystem::path map_path = resolveMapPath(simulation.map_filename);
    auto map_array = std::make_shared<NpyArray>();
    const LPCSTR load_error = map_array->LoadNPY(map_path.string());
    if (load_error != nullptr) {
        return {
            {},
            types::ErrorRef{
                "MAP_FILE_NOT_FOUND",
                map_path.string(),
            },
        };
    }

    return {
        std::make_unique<Map3DImpl>(std::move(map_array), hiddenMapConfig(simulation)),
        std::nullopt,
    };
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    const int run_id = next_run_id_++;
    const std::filesystem::path run_dir = io::runOutputDir(output_path, run_id);
    const std::filesystem::path output_map_file = io::runOutputMap(output_path, run_id);
    const std::filesystem::path error_log_file = io::runErrorLog(output_path, run_id);

    std::filesystem::create_directories(run_dir);
    io::RunErrorLog error_log{error_log_file};

    std::vector<types::ErrorRef> startup_errors;
    HiddenMapLoadResult hidden_map_load = loadHiddenMap(simulation);
    if (hidden_map_load.error) {
        error_log.log(*hidden_map_load.error);
        startup_errors.push_back(*hidden_map_load.error);
    }

    std::unique_ptr<Map3DImpl> hidden_map =
        hidden_map_load.map
            ? std::move(hidden_map_load.map)
            : std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), hiddenMapConfig(simulation));
    const types::MapConfig output_config = outputMapConfig(simulation, mission, *hidden_map);
    auto output_map =
        std::make_unique<Map3DImpl>(makeEmptyOutputArray(output_config), output_config);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        lidar,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file,
        std::move(startup_errors));
}

} // namespace drone_mapper
