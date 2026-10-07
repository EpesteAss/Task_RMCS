#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace rmcs_core::hardware {

// Sensorless homing against two independent, repeatable upper hard stops.
// A stalled screw elsewhere is indistinguishable from a hard stop.
class GantryHoming {
public:
    enum class State { Waiting = 0, Seeking = 1, Complete = 2, Failed = 3 };

    struct Config {
        double contact_torque_nm;
        double contact_speed_rad_s;
        double contact_hold_s;
        double timeout_s;
        double max_travel_rad;
    };
    struct Input {
        bool valid;
        double time_s;
        std::array<double, 2> angle_rad, speed_rad_s;
        std::array<double, 2> torque_nm, commanded_torque_nm;
    };
    struct Output {
        State state;
        std::array<bool, 2> drive_up;
        std::array<bool, 2> reached;
    };

    explicit GantryHoming(Config config) : config_(config) {}

    Output update(const Input& in) {
        if (state_ == State::Complete || state_ == State::Failed)
            return output();
        if (!std::isfinite(in.time_s) || !finite(in.angle_rad) || !finite(in.speed_rad_s)
            || !finite(in.torque_nm) || !finite(in.commanded_torque_nm)) {
            state_ = State::Failed;
            return output();
        }
        if (state_ == State::Waiting) {
            if (in.valid) {
                started_s_ = in.time_s;
                initial_angle_ = in.angle_rad;
                state_ = State::Seeking;
            }
            return output();
        }
        if (!in.valid || in.time_s < started_s_ || in.time_s - started_s_ > config_.timeout_s) {
            state_ = State::Failed;
            return output();
        }
        for (std::size_t i = 0; i < 2; ++i) {
            if (reached_[i])
                continue;
            const double travel = in.angle_rad[i] - initial_angle_[i];
            if (travel < -0.1 || travel > config_.max_travel_rad) {
                state_ = State::Failed;
                return output();
            }
            const bool contact = in.commanded_torque_nm[i] >= config_.contact_torque_nm
                              && in.torque_nm[i] >= config_.contact_torque_nm
                              && std::abs(in.speed_rad_s[i]) <= config_.contact_speed_rad_s;
            if (!contact) {
                contact_since_s_[i] = -1.0;
                continue;
            }
            if (contact_since_s_[i] < 0.0)
                contact_since_s_[i] = in.time_s;
            if (in.time_s - contact_since_s_[i] >= config_.contact_hold_s)
                reached_[i] = true;
        }
        if (reached_[0] && reached_[1])
            state_ = State::Complete;
        return output();
    }

    State state() const { return state_; }
    std::array<bool, 2> reached() const { return reached_; }

private:
    template<std::size_t N>
    static bool finite(const std::array<double, N>& values) {
        for (double value : values)
            if (!std::isfinite(value))
                return false;
        return true;
    }
    Output output() const {
        return {.state = state_,
                .drive_up = {state_ == State::Seeking && !reached_[0],
                             state_ == State::Seeking && !reached_[1]},
                .reached = reached_};
    }

    Config config_;
    State state_ = State::Waiting;
    double started_s_ = 0.0;
    std::array<double, 2> initial_angle_{};
    std::array<double, 2> contact_since_s_{-1.0, -1.0};
    std::array<bool, 2> reached_{};
};

} // namespace rmcs_core::hardware
