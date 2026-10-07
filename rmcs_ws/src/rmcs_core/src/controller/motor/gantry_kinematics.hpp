#pragma once

#include <cmath>
#include <utility>

namespace rmcs_core::controller::gantry {

// O is the fixed pivot. The front attachment is (front_distance, lateral, height).
inline double pitch_from_position(double front_distance, double lateral, double height) {
    return std::atan2(height, std::hypot(front_distance, lateral));
}

inline double yaw_from_position(double front_distance, double lateral) {
    return std::atan2(lateral, front_distance);
}

inline double height_for_pitch(double front_distance, double lateral, double pitch) {
    return std::hypot(front_distance, lateral) * std::tan(pitch);
}

inline double lateral_for_yaw(double front_distance, double yaw) {
    return front_distance * std::tan(yaw);
}

// Return left/right screw displacement errors in metres.
inline std::pair<double, double>
    synchronized_errors(double target_displacement, double left, double right, double sync_gain) {
    const double average = (left + right) / 2.0;
    const double difference = left - right;
    const double position_error = target_displacement - average;
    return {
        position_error - sync_gain * difference,
        position_error + sync_gain * difference,
    };
}

} // namespace rmcs_core::controller::gantry
