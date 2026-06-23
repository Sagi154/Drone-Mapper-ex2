#include <drone_mapper/SimulationManager.h>

#include <drone_mapper/io/SimulationOutputYaml.h>
#include <drone_mapper/io/TimeFormat.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;
    std::vector<io::SimulationRunYamlEntry> yaml_entries;
    yaml_entries.reserve(composition.simulations.size() * composition.missions.size() *
                         composition.drones.size() * composition.lidars.size());

    int run_id = 1;
    for (std::size_t simulation_index = 0; simulation_index < composition.simulations.size();
         ++simulation_index) {
        const types::SimulationConfigData& simulation = composition.simulations[simulation_index];
        for (std::size_t mission_index = 0; mission_index < composition.missions.size(); ++mission_index) {
            const types::MissionConfigData& mission = composition.missions[mission_index];
            for (std::size_t drone_index = 0; drone_index < composition.drones.size(); ++drone_index) {
                const types::DroneConfigData& drone = composition.drones[drone_index];
                for (std::size_t lidar_index = 0; lidar_index < composition.lidars.size(); ++lidar_index) {
                    const types::LidarConfigData& lidar = composition.lidars[lidar_index];
                    std::unique_ptr<ISimulationRun> run =
                        run_factory_->create(simulation, mission, drone, lidar, output_path);
                    types::SimulationResult result = run->run();
                    runs.push_back(result);
                    yaml_entries.push_back(io::SimulationRunYamlEntry{
                        run_id,
                        simulation_index,
                        mission_index,
                        drone_index,
                        lidar_index,
                        std::move(result),
                    });
                    ++run_id;
                }
            }
        }
    }

    types::SimulationManagerReport report{};
    report.generated_at_utc = io::currentUtcTimestamp();
    report.metric = "maps_comparison_score";
    report.score_range = {0.0, 100.0};
    report.error_score = -1;
    report.runs = std::move(runs);

    io::writeSimulationOutputYaml(output_path, report, yaml_entries);
    return report;
}

} // namespace drone_mapper
