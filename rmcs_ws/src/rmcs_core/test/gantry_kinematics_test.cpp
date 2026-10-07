#include <cmath>
#include <numbers>

#include "controller/motor/gantry_kinematics.hpp"

using namespace rmcs_core::controller::gantry;

int main() {
    constexpr double front = 0.45;
    constexpr double pitch_target = 0.2;
    constexpr double tolerance = 1e-12;

    // A yaw-only sweep must retain the same pitch after height compensation.
    for (double yaw : {-0.15, -0.08, 0.0, 0.08, 0.15}) {
        const double lateral = lateral_for_yaw(front, yaw);
        const double height = height_for_pitch(front, lateral, pitch_target);
        if (std::abs(yaw_from_position(front, lateral) - yaw) > tolerance
            || std::abs(pitch_from_position(front, lateral, height) - pitch_target) > tolerance)
            return 1;
    }

    // When the left lift screw leads, its correction must point backward.
    const auto [left_error, right_error] = synchronized_errors(0.01, 0.011, 0.009, 1.0);
    if (!(left_error < 0.0 && right_error > 0.0))
        return 2;

    // Equal travel must not introduce a differential command.
    const auto [equal_left, equal_right] = synchronized_errors(0.02, 0.01, 0.01, 1.0);
    if (std::abs(equal_left - equal_right) > tolerance)
        return 3;
    return 0;
}
