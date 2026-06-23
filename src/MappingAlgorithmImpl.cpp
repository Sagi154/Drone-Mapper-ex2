#include <drone_mapper/MappingAlgorithmImpl.h>

namespace drone_mapper {

// MappingAlgorithmImpl::MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map)
//     : IMappingAlgorithm(drone_config, output_map) {}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                       const types::LidarScanResult* latest_scan) {
    (void)state;
    (void)latest_scan;
    (void)mission_config_;
    (void)lidar_config_;
    (void)drone_config_;
    (void)output_map_;
    return types::MappingStepCommand{};
}

} // namespace drone_mapper
