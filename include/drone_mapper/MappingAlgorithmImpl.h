#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    enum class Phase {
        AwaitingScan,
        Moving,
    };

    Phase phase_ = Phase::AwaitingScan;
    bool finished_ = false;
    bool awaiting_planning_after_scan_ = false;
    std::vector<Position3D> path_{};
    std::size_t path_index_ = 0;
};

} // namespace drone_mapper
