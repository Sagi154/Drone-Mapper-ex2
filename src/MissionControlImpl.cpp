// MissionControlImpl.cpp
// Mission-level step loop: drives IDroneControl until completion, error, or max_steps.

#include <drone_mapper/MissionControlImpl.h>

#include <exception>
#include <utility>

namespace drone_mapper {

namespace {

[[nodiscard]] types::MissionRunResult finalizeMission(types::MissionRunStatus status,
                                                      std::size_t steps,
                                                      std::vector<types::ErrorRef> errors,
                                                      IMutableMap3D& output_map,
                                                      const std::filesystem::path& output_file) {
    try {
        output_map.save(output_file);
    } catch (const std::exception& ex) {
        errors.push_back(types::ErrorRef{"MAP_SAVE_FAILED", ex.what()});
        return types::MissionRunResult{
            types::MissionRunStatus::Error,
            steps,
            std::move(errors),
        };
    }

    return types::MissionRunResult{
        status,
        steps,
        std::move(errors),
    };
}

} // namespace

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
    (void)hidden_map_;
    (void)drone_;

    std::size_t steps = 0;
    types::MissionRunStatus status = types::MissionRunStatus::MaxSteps;
    std::vector<types::ErrorRef> errors;

    while (steps < mission_.max_steps) {
        const types::DroneStepResult step_result = drone_control_.step();
        ++steps;

        if (step_result.status == types::DroneStepStatus::Error) {
            status = types::MissionRunStatus::Error;
            errors.push_back(types::ErrorRef{
                "DRONE_STEP_FAILED",
                step_result.message.empty() ? "Drone step failed." : step_result.message,
            });
            return finalizeMission(status, steps, std::move(errors), output_map_, output_map_file_);
        }

        if (step_result.status == types::DroneStepStatus::Completed) {
            status = types::MissionRunStatus::Completed;
            return finalizeMission(status, steps, std::move(errors), output_map_, output_map_file_);
        }
    }

    return finalizeMission(status, steps, std::move(errors), output_map_, output_map_file_);
}

} // namespace drone_mapper
