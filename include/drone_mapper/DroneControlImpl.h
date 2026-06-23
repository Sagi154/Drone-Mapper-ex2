#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMutableMap3D.h>

namespace drone_mapper {

/// Executes one drone step: algorithm command, movement, scan, and voxel fusion.
class DroneControlImpl final : public IDroneControl {
public:
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     types::LidarConfigData lidar,
                     ILidar& lidar_sensor,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    types::LidarConfigData lidar_;
    ILidar& lidar_sensor_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    types::LidarScanResult latest_scan_{};
    bool has_latest_scan_ = false;
    std::size_t step_index_ = 0;
};

} // namespace drone_mapper
