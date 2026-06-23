#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file,
                                     std::vector<types::ErrorRef> startup_errors)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)),
      startup_errors_(std::move(startup_errors)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    if (!startup_errors_.empty()) {
        types::SimulationResult result{};
        result.simulation_config = simulation_config_;
        result.mission_config = mission_config_;
        result.output_map_file = output_map_file_;
        result.mission_score = -1.0;
        result.mission_results.push_back(types::MissionRunResult{
            types::MissionRunStatus::Error,
            0,
            startup_errors_,
        });
        return result;
    }

    // Stub: touch injected deps so -Wunused-private-field stays clean under -Werror on Linux CI.
    (void)hidden_map_;
    (void)output_map_;
    (void)gps_;
    (void)movement_;
    (void)lidar_;
    (void)mapping_algorithm_;
    (void)drone_control_;
    (void)mission_control_;
    (void)simulation_config_;
    (void)mission_config_;
    (void)output_map_file_;
    return types::SimulationResult{};
}

} // namespace drone_mapper
