// MockGPS.cpp
// Simulated GPS: stores drone position and heading, snapping position
// coordinates to the nearest resolution-grid multiple on every write.
// Snapping ensures the algorithm and movement layers agree on a quantized
// coordinate system, matching the gps_resolution field in MissionConfigData.

#include <drone_mapper/MockGPS.h>

#include <cmath>

namespace drone_mapper {

namespace {

/// Snap a raw centimetre value to the nearest multiple of res_cm.
/// If res_cm is zero or negative the value is returned unchanged.
double snapToCm(double value, double res_cm) {
    if (res_cm <= 0.0) {
        return value;
    }
    return std::round(value / res_cm) * res_cm;
}

} // namespace

MockGPS::MockGPS(Position3D position, Orientation heading, PhysicalLength resolution)
    : position_(position), heading_(heading), resolution_(resolution) {}

Position3D MockGPS::position() const {
    return position_;
}

Orientation MockGPS::heading() const {
    return heading_;
}

void MockGPS::setPosition(Position3D position) {
    const double res_cm = resolution_.numerical_value_in(cm);
    position_ = Position3D{
        snapToCm(position.x.numerical_value_in(cm), res_cm) * x_extent[cm],
        snapToCm(position.y.numerical_value_in(cm), res_cm) * y_extent[cm],
        snapToCm(position.z.numerical_value_in(cm), res_cm) * z_extent[cm],
    };
}

void MockGPS::setHeading(Orientation heading) {
    heading_ = heading;
}

} // namespace drone_mapper
