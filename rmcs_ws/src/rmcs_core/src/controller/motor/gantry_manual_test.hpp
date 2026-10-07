#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace rmcs_core::controller::gantry {

// Continuous manual speed control without a calibrated launcher geometry.
// Angles are output-shaft travel relative to program startup, positive lift up/yaw left.
class ManualTest {
public:
    static constexpr double max_speed = 10.0;          // output-shaft rad/s
    static constexpr double max_lift_difference = 3.0; // radians (~0.095 mm)
    static constexpr double max_measured_speed = 20.0; // rad/s
    static constexpr double sync_gain = 2.0;           // 1/s

    struct Input {
        bool valid;
        double pitch_stick, yaw_stick;
        std::array<double, 3> angle, velocity;
    };
    struct Output {
        std::array<double, 3> velocity{};
        std::array<bool, 3> enabled{};
        // 0=disarmed, 1=ready, 2=lift, 3=yaw, 4=interlock, 5=both, 6=skew recovery.
        int state = 0;
    };

    Output update(const Input& in) {
        if (!in.valid || !std::isfinite(in.pitch_stick) || !std::isfinite(in.yaw_stick)
            || !finite(in.angle) || !finite(in.velocity)) {
            ready_ = false;
            return {};
        }
        const double lift_difference = in.angle[0] - in.angle[1];
        if (std::abs(lift_difference) > max_lift_difference)
            lift_recovery_ = true;
        else if (std::abs(lift_difference) < max_lift_difference * 0.5)
            lift_recovery_ = false;
        const bool centered = in.pitch_stick == 0.0 && in.yaw_stick == 0.0;
        if (centered) {
            ready_ = true;
            stopped_ = false;
            return Output{.state = 1};
        }
        if (stopped_)
            return Output{.state = 4};
        if (!ready_)
            return {};
        for (std::size_t i = 0; i < 3; ++i) {
            if (std::abs(in.velocity[i]) > max_measured_speed)
                return stop();
        }
        Output out;
        const bool pitch_requested = in.pitch_stick != 0.0;
        const bool yaw_requested = in.yaw_stick != 0.0;
        out.state = lift_recovery_ ? 6 : (pitch_requested ? (yaw_requested ? 5 : 2) : 3);
        if (pitch_requested && lift_recovery_) {
            // Move only the side that reduces the *relative* skew. Lower speed
            // limits stress while recovering from a stalled lift motor.
            const double speed = std::clamp(in.pitch_stick, -1.0, 1.0) * 2.0;
            const std::size_t side = lift_difference > 0.0
                                         ? (speed > 0.0 ? 1 : 0)
                                         : (speed > 0.0 ? 0 : 1);
            out.velocity[side] = speed;
            out.enabled[side] = true;
        } else if (pitch_requested) {
            const double speed = std::clamp(in.pitch_stick, -1.0, 1.0) * max_speed;
            const double correction = (in.angle[0] - in.angle[1]) * sync_gain * 0.5;
            out.velocity = {std::clamp(speed - correction, -max_speed, max_speed),
                            std::clamp(speed + correction, -max_speed, max_speed), 0.0};
            out.enabled = {true, true, false};
        }
        if (yaw_requested) {
            out.velocity[2] = std::clamp(in.yaw_stick, -1.0, 1.0) * max_speed;
            out.enabled[2] = true;
        }
        return out;
    }

private:
    static bool finite(const std::array<double, 3>& values) {
        return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
    }
    Output stop() {
        stopped_ = true;
        ready_ = false;
        return Output{.state = 4};
    }
    bool ready_ = false, stopped_ = false, lift_recovery_ = false;
};

} // namespace rmcs_core::controller::gantry
