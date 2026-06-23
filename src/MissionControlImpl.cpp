// MissionControlImpl — mission-level step loop; saves output map before returning.
#include <drone_mapper/MissionControlImpl.h>

#include <utility>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    (void)drone_;
    (void)hidden_map_;

    std::size_t steps = 0;
    for (std::size_t attempt = 0; attempt < mission_.max_steps; ++attempt) {
        const types::DroneStepResult step_result = drone_control_.step();
        ++steps;

        if (step_result.status == types::DroneStepStatus::Completed) {
            output_map_.save(output_map_file_);
            return types::MissionRunResult{
                types::MissionRunStatus::Completed,
                steps,
                {},
            };
        }

        if (step_result.status == types::DroneStepStatus::Error) {
            output_map_.save(output_map_file_);
            return types::MissionRunResult{
                types::MissionRunStatus::Error,
                steps,
                {types::ErrorRef{
                    "DRONE_STEP_ERROR",
                    step_result.message.empty() ? "Drone step failed." : step_result.message,
                }},
            };
        }
    }

    output_map_.save(output_map_file_);
    return types::MissionRunResult{
        types::MissionRunStatus::MaxSteps,
        steps,
        {},
    };
}

} // namespace drone_mapper
