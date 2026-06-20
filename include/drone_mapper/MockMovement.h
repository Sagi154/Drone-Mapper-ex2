#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/Types.h>

namespace drone_mapper {

/// Simulated drone movement driver.
/// Validates per-command limits and performs sphere-based collision detection
/// against the hidden truth map before committing each move to the GPS.
class MockMovement final : public IDroneMovement {
public:
    MockMovement(MockGPS& gps,
                 const IMap3D& hidden_map,
                 types::DroneConfigData drone_config);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    MockGPS& gps_;
    const IMap3D& hidden_map_;
    types::DroneConfigData drone_;
};

} // namespace drone_mapper
