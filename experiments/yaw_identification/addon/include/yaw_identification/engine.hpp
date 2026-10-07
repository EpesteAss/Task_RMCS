#pragma once
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>
#include "yaw_identification/pid_calculator.hpp"

namespace yaw_identification {
using Pid = rmcs_core::controller::pid::PidCalculator;
enum class State { Off, Raising, Ready, Sweep, Fault, Releasing };
enum class Command { None, Arm, Sweep, Hold, Off };
struct Config {
    bool world_pitch_control = false;
    // Mechanical re-indexing changes the single-turn encoder phase.  When
    // world-pitch feedback is used, enforce travel around the encoder value
    // captured by Arm instead of assuming a permanently calibrated zero.
    bool pitch_limits_relative_to_arm = false;
    bool tracking_test = false;
    bool rich_identification = false;
    double world_pitch_target = 5.0 * std::numbers::pi / 180.0;
    double pitch_target = -10.0 * std::numbers::pi / 180.0;
    double pitch_min = -0.60, pitch_max = 0.10;
    double pitch_ramp = 0.0872665, pitch_torque_limit = 3.0;
    double pitch_target_lead = 0.8 * std::numbers::pi / 180.0;
    double pitch_target_soft_tau = 0.0, pitch_stiction_compensation = 0.0;
    double pitch_precharge_s = 0.0, pitch_precharge_torque_slew = 8.0;
    double pitch_start_ramp_s = 0.0;
    double yaw_torque_limit = 0.3, yaw_span = 0.139626;
    double torque_slew = 5.0, torque_release_slew = 30.0, off_slew = 1.0;
    double pitch_velocity_limit = 6.0;
    double pitch_speed_trip = 0.8, yaw_speed_trip = 1.0;
    double pitch_velocity_filter_tau = 0.012, yaw_velocity_filter_tau = 0.012;
    double gravity_gain = 0.0, gravity_phase = 0.0;
    bool staged_gravity_gain = false;
    double gravity_raise_gain = 0.0;
    double pitch_ready_tolerance = 1.5 * std::numbers::pi / 180.0;
    double min_world_pitch = 0.0;
    double amplitude = 0.01, start_hz = 0.2, end_hz = 3.0, duration = 20.0;
    double tracking_amplitude = 5.0 * std::numbers::pi / 180.0;
    double tracking_frequency = 0.25, tracking_ramp = 1.0;
    std::vector<double> tracking_frequencies;
    double yaw_ff_velocity = 0.0, yaw_ff_acceleration = 0.0, yaw_ff_bias = 0.0;
    double yaw_torque_slew = 1e6, yaw_torque_release_slew = 1e6;
    double raise_timeout = 10.0;
};
struct Input {
    double yaw = 0, pitch = 0, pitch_world = 0, yaw_velocity = 0, pitch_velocity = 0;
    double yaw_temperature = 0, pitch_temperature = 0;
    bool heartbeat = false;
};
struct Output {
    double yaw_torque = 0, pitch_torque = 0, pitch_torque_unlimited = 0, pitch_target = 0;
    double excitation = 0, elapsed = 0;
    double yaw_target_offset = 0, yaw_error = 0, yaw_feedforward = 0;
    double tracking_frequency = 0;
    int excitation_profile = 0;
    State state = State::Off;
    int fault = 0;
    bool completed = false;
};

// PID implementation is an unmodified copy of RMCS's PidCalculator.
// Only this engine's update thread touches PID state.
class Engine {
public:
    // Matches the configured MG4010Ei10 maximum torque reference in RMCS.
    static constexpr double kPitchMotorMaxTorque = 4.5;
    Config config;
    Pid pitch_angle{35.0, 0.02, 0.3}, pitch_velocity{2.0, 0.0, 0.0};
    Pid yaw_angle{2.0, 0.0, 0.0}, yaw_velocity{1.0, 0.0, 0.0};

    void validate() const {
        for (double v : {config.pitch_target, config.pitch_min, config.pitch_max,
                        config.gravity_gain, config.gravity_phase, config.gravity_raise_gain,
                        config.min_world_pitch,
                        config.world_pitch_target, config.yaw_ff_velocity,
                        config.yaw_ff_acceleration, config.yaw_ff_bias})
            if (!std::isfinite(v)) throw std::runtime_error("Non-finite experiment parameter");
        for (double v : {config.pitch_ramp, config.pitch_torque_limit,
                        config.pitch_target_lead, config.pitch_ready_tolerance,
                        config.yaw_torque_limit,
                        config.yaw_span, config.torque_slew, config.torque_release_slew,
                        config.off_slew,
                        config.pitch_velocity_limit,
                        config.pitch_speed_trip, config.yaw_speed_trip,
                        config.pitch_velocity_filter_tau, config.yaw_velocity_filter_tau,
                        config.start_hz,
                        config.end_hz, config.duration, config.raise_timeout,
                        config.tracking_frequency, config.tracking_ramp,
                        config.yaw_torque_slew, config.yaw_torque_release_slew})
            if (!std::isfinite(v) || v <= 0) throw std::runtime_error("Invalid positive experiment parameter");
        if (!std::isfinite(config.pitch_precharge_s) || config.pitch_precharge_s < 0
            || config.pitch_precharge_s > 1.0
            || !std::isfinite(config.pitch_start_ramp_s)
            || config.pitch_start_ramp_s < 0 || config.pitch_start_ramp_s > 1.0
            || !std::isfinite(config.pitch_precharge_torque_slew)
            || config.pitch_precharge_torque_slew <= 0
            || !std::isfinite(config.pitch_target_soft_tau) || config.pitch_target_soft_tau < 0
            || !std::isfinite(config.pitch_stiction_compensation)
            || config.pitch_stiction_compensation < 0
            || config.pitch_stiction_compensation > 0.2)
            throw std::runtime_error("Invalid pitch smoothing or friction compensation");
        if (config.pitch_torque_limit > kPitchMotorMaxTorque
            || config.yaw_torque_limit > kPitchMotorMaxTorque)
            throw std::runtime_error("Pitch torque limit exceeds the configured MG4010Ei10 maximum");
        if (config.pitch_target <= config.pitch_min || config.pitch_target >= config.pitch_max
            || config.pitch_ready_tolerance > 0.1
            || config.min_world_pitch < 0 || config.min_world_pitch > 0.6
            || (config.world_pitch_control && (std::abs(config.world_pitch_target) > 0.6
                || config.world_pitch_target < config.min_world_pitch))
            || !std::isfinite(config.amplitude) || config.amplitude < 0
            || (!config.tracking_test && config.amplitude >= config.yaw_torque_limit)
            || !std::isfinite(config.tracking_amplitude)
            // A target exactly on the configured bound is allowed. Actual
            // motion beyond that bound still trips fault 5 below.
            || config.tracking_amplitude <= 0 || config.tracking_amplitude > config.yaw_span
            || config.tracking_ramp * 2 >= config.duration)
            throw std::runtime_error("Pitch target or sweep amplitude outside limits");
        for (double frequency : config.tracking_frequencies)
            if (!std::isfinite(frequency) || frequency <= 0)
                throw std::runtime_error("Invalid tracking frequency sequence");
    }

    Output update(Input in, Command cmd, double dt) {
        // LkMotor single-turn feedback uses [0, 2*pi); raised pitch is near 2*pi.
        in.pitch = std::remainder(in.pitch, 2 * std::numbers::pi);
        // Raising decreases the motor coordinate and increases world pitch.
        const double pitch_measurement = config.world_pitch_control ? -in.pitch_world : in.pitch;
        const double pitch_goal = config.world_pitch_control ? -config.world_pitch_target : config.pitch_target;
        if (cmd == Command::Off) {
            if (out_.state == State::Off || out_.state == State::Fault) {
                reset(State::Off);
                return out_;
            }
            // An operator-requested OFF lowers the supporting pitch effort
            // gradually, so the launcher does not drop when holding torque is
            // removed. Yaw motion stops immediately. Fault paths still call
            // fault(), which removes both torques immediately.
            out_.state = State::Releasing;
            out_.yaw_torque = 0.0;
            out_.excitation = 0.0;
            out_.yaw_target_offset = 0.0;
            out_.yaw_error = 0.0;
            out_.yaw_feedforward = 0.0;
            out_.pitch_torque_unlimited = 0.0;
        }
        if (out_.state == State::Releasing) {
            if (!in.heartbeat) return fault(1);
            if (!finite(in) || !std::isfinite(dt) || dt <= 0 || dt > 0.05) return fault(2);
            if (!pitch_within_limits(in.pitch))
                return fault(3);
            if (std::abs(in.pitch_velocity) > config.pitch_speed_trip
                || std::abs(in.yaw_velocity) > config.yaw_speed_trip) return fault(4);
            if (in.yaw_temperature >= 65 || in.pitch_temperature >= 65) return fault(6);
            out_.pitch_torque += std::clamp(-out_.pitch_torque,
                                            -config.off_slew * dt,
                                            config.off_slew * dt);
            if (std::abs(out_.pitch_torque) <= config.off_slew * dt)
                reset(State::Off);
            return out_;
        }
        if (cmd == Command::Arm && out_.state == State::Off) {
            if (!in.heartbeat || !finite(in)) return out_;
            sweep_count_ = 0;
            excitation_trial_ = 0;
            center_ = in.yaw;
            pitch_arm_origin_ = in.pitch;
            out_.pitch_target = pitch_measurement;
            pitch_control_velocity_ = in.pitch_velocity;
            yaw_control_velocity_ = in.yaw_velocity;
            stall_anchor_ = in.pitch;
            readiness_pitch_velocity_ = in.pitch_velocity;
            out_.state = State::Raising;
        }
        if (out_.state == State::Off || out_.state == State::Fault) return out_;
        if (!in.heartbeat) return fault(1);
        if (!finite(in) || !std::isfinite(dt) || dt <= 0 || dt > 0.05) return fault(2);
        if (!pitch_within_limits(in.pitch))
            return fault(3);
        if (std::abs(in.pitch_velocity) > config.pitch_speed_trip
            || std::abs(in.yaw_velocity) > config.yaw_speed_trip) return fault(4);
        const double center_error = std::remainder(center_ - in.yaw, 2 * std::numbers::pi);
        if (std::abs(center_error) > config.yaw_span) return fault(5);
        if (in.yaw_temperature >= 65 || in.pitch_temperature >= 65) return fault(6);
        const double pitch_velocity_alpha = dt / (config.pitch_velocity_filter_tau + dt);
        const double yaw_velocity_alpha = dt / (config.yaw_velocity_filter_tau + dt);
        pitch_control_velocity_ += pitch_velocity_alpha
                                   * (in.pitch_velocity - pitch_control_velocity_);
        yaw_control_velocity_ += yaw_velocity_alpha
                                 * (in.yaw_velocity - yaw_control_velocity_);
        const double target_error = pitch_goal - pitch_measurement;
        // Only readiness uses a 50 ms low-pass filter. PID feedback and the
        // hard overspeed trip above continue to use the raw IMU velocity.
        readiness_pitch_velocity_ += dt / (0.05 + dt)
                                     * (in.pitch_velocity - readiness_pitch_velocity_);
        const bool settled = std::abs(target_error) < config.pitch_ready_tolerance
                             && std::abs(readiness_pitch_velocity_) < 0.05;
        stable_ = settled ? stable_ + dt : 0.0;
        if ((out_.state == State::Ready || out_.state == State::Sweep) && !settled) {
            out_.state = State::Raising;
            raising_ = 0.0;
        }
        // The launcher must clear the chassis before yaw motion. The encoder
        // target is relative to the yaw frame and is not the world pitch.
        if (out_.state == State::Sweep && in.pitch_world < config.min_world_pitch)
            out_.state = State::Ready;
        raising_ = out_.state == State::Raising ? raising_ + dt : 0.0;
        if (out_.state == State::Raising && raising_ > config.raise_timeout) return fault(7);
        if (out_.state == State::Raising && stable_ >= 1.0) out_.state = State::Ready;
        if (cmd == Command::Hold && out_.state == State::Sweep) out_.state = State::Ready;
        if (cmd == Command::Sweep && out_.state == State::Ready && settled
            && in.pitch_world >= config.min_world_pitch) {
            out_.state = State::Sweep;
            out_.elapsed = 0.0;
            out_.completed = false;
            out_.tracking_frequency = config.tracking_frequency;
            if (config.tracking_test && !config.tracking_frequencies.empty()) {
                const auto profile = sweep_count_ % config.tracking_frequencies.size();
                out_.tracking_frequency = config.tracking_frequencies[profile];
                out_.excitation_profile = static_cast<int>(profile);
                ++sweep_count_;
            } else if (!config.tracking_test) {
                excitation_trial_ = sweep_count_++;
                out_.excitation_profile = config.rich_identification
                    ? static_cast<int>(excitation_trial_ % 6) : 0;
            }
        }
        const double pitch_direction = pitch_goal >= out_.pitch_target ? 1.0 : -1.0;
        const double pitch_distance = std::abs(pitch_goal - out_.pitch_target);
        const double pitch_lead = pitch_direction * (out_.pitch_target - pitch_measurement);
        // Build the gravity-supporting effort while the reference is still at
        // its initial position. Starting the position ramp at zero torque made
        // the reference run ahead, then the launcher broke free in a lurch.
        const bool precharging = out_.state == State::Raising
            && std::abs(target_error) >= config.pitch_ready_tolerance
            && pitch_precharge_elapsed_ < config.pitch_precharge_s;
        if (out_.state == State::Raising) pitch_precharge_elapsed_ += dt;
        // Ease into the normal reference speed after gravity support is ready.
        // The older step from zero straight to the full ramp was visible as a
        // small lurch even though precharging removed the large breakaway.
        const double start_fraction = config.pitch_start_ramp_s > 0
            ? std::clamp((pitch_precharge_elapsed_ - config.pitch_precharge_s)
                             / config.pitch_start_ramp_s, 0.0, 1.0)
            : 1.0;
        const double pitch_ramp_now = config.pitch_ramp * start_fraction;
        double pitch_step = precharging ? 0.0
                                        : std::min(pitch_distance, pitch_ramp_now * dt);
        if (!precharging && config.pitch_target_soft_tau > 0) {
            // Near the lead bound, move with the filtered mechanism velocity.
            // The correction closes the remaining lead smoothly; a feedback
            // step cannot jump the reference by more than one ramp step.
            const double measured_progress = pitch_direction * pitch_control_velocity_;
            const double lead_correction = (config.pitch_target_lead - pitch_lead)
                                           / config.pitch_target_soft_tau;
            const double reference_speed = std::clamp(measured_progress + lead_correction,
                                                      0.0, pitch_ramp_now);
            pitch_step = std::min(pitch_step, reference_speed * dt);
            if (pitch_lead >= 0)
                pitch_step = std::min(pitch_step,
                                      std::max(0.0, config.pitch_target_lead - pitch_lead));
        } else if (!precharging && pitch_lead + pitch_step > config.pitch_target_lead
                   && pitch_lead >= 0) {
            pitch_step = 0.0;
        }
        out_.pitch_target += pitch_direction * pitch_step;
        const double pitch_ref = std::clamp(pitch_angle.update(out_.pitch_target - pitch_measurement),
                                           -config.pitch_velocity_limit, config.pitch_velocity_limit);
        // Preserve the previously tested raising torque until the launcher is
        // near clearance, then blend in the measured holding feedforward.
        const double blend = std::clamp(
            (in.pitch_world - (config.min_world_pitch - std::numbers::pi / 180.0))
                / (1.5 * std::numbers::pi / 180.0), 0.0, 1.0);
        const double gravity_gain = config.staged_gravity_gain
            ? config.gravity_raise_gain + blend * (config.gravity_gain - config.gravity_raise_gain)
            : config.gravity_gain;
        const double motion_gate = std::clamp(
            (std::abs(target_error) - 1.5 * std::numbers::pi / 180.0)
                / (1.0 * std::numbers::pi / 180.0), 0.0, 1.0);
        const double speed_gate = std::clamp(
            1.0 - std::abs(pitch_control_velocity_) / (4.0 * std::numbers::pi / 180.0),
            0.0, 1.0);
        const double pitch_command = pitch_velocity.update(pitch_ref - pitch_control_velocity_)
            + gravity_gain * std::sin(in.pitch_world - config.gravity_phase)
            + pitch_direction * config.pitch_stiction_compensation * motion_gate * speed_gate;
        out_.pitch_torque_unlimited = pitch_command;
        const double limited_pitch = std::clamp(pitch_command, -config.pitch_torque_limit,
                                               config.pitch_torque_limit);
        // Limit how quickly lifting effort is applied, while allowing it to be
        // removed much faster. A symmetric slew limiter kept applying nearly
        // saturated torque after the PID had already requested braking, which
        // could accelerate pitch into the overspeed trip during a long raise.
        const double pitch_delta = limited_pitch - out_.pitch_torque;
        const bool releasing = pitch_delta * out_.pitch_torque < 0.0;
        const double pitch_slew = releasing ? config.torque_release_slew
            : precharging ? config.pitch_precharge_torque_slew : config.torque_slew;
        out_.pitch_torque += std::clamp(pitch_delta, -pitch_slew * dt,
                                       pitch_slew * dt);
        // Use encoder displacement over a window. IMU vibration was resetting the
        // former per-tick stall timer even when the encoder was stationary.
        // Fault 8 means sustained torque saturation without motion; it cannot
        // distinguish insufficient lifting torque from a mechanical obstruction.
        stall_window_ += dt;
        if (stall_window_ >= 0.25) {
            const bool stalled = std::abs(target_error) > 0.035
                && std::abs(in.pitch - stall_anchor_) < 0.00174533
                && std::abs(out_.pitch_torque) > config.pitch_torque_limit * 0.95;
            stalled_ = stalled ? stalled_ + stall_window_ : 0.0;
            stall_anchor_ = in.pitch;
            stall_window_ = 0.0;
        }
        if (stalled_ > 1.0) return fault(8);
        out_.excitation = 0.0;
        out_.yaw_target_offset = 0.0;
        out_.yaw_feedforward = 0.0;
        if (out_.state == State::Sweep) {
            const double t = out_.elapsed;
            if (t >= config.duration) {
                out_.state = State::Ready;
                out_.completed = true;
            }
            else if (config.tracking_test) {
                double envelope = 1.0, envelope_velocity = 0.0, envelope_acceleration = 0.0;
                const double k = std::numbers::pi / config.tracking_ramp;
                if (t < config.tracking_ramp) {
                    envelope = 0.5 * (1.0 - std::cos(k * t));
                    envelope_velocity = 0.5 * k * std::sin(k * t);
                    envelope_acceleration = 0.5 * k * k * std::cos(k * t);
                } else if (config.duration - t < config.tracking_ramp) {
                    const double remaining = config.duration - t;
                    envelope = 0.5 * (1.0 - std::cos(k * remaining));
                    envelope_velocity = -0.5 * k * std::sin(k * remaining);
                    envelope_acceleration = 0.5 * k * k * std::cos(k * remaining);
                }
                const double omega = 2 * std::numbers::pi * out_.tracking_frequency;
                const double sine = std::sin(omega * t), cosine = std::cos(omega * t);
                out_.yaw_target_offset = config.tracking_amplitude * envelope * sine;
                const double reference_velocity = config.tracking_amplitude
                    * (envelope_velocity * sine + envelope * omega * cosine);
                const double reference_acceleration = config.tracking_amplitude
                    * (envelope_acceleration * sine + 2 * envelope_velocity * omega * cosine
                       - envelope * omega * omega * sine);
                out_.yaw_feedforward = config.yaw_ff_velocity * reference_velocity
                    + config.yaw_ff_acceleration * reference_acceleration + config.yaw_ff_bias;
                out_.elapsed += dt;
            } else {
                // Repeating one identical chirp adds samples but little new
                // information. Rich mode cycles three chirps and three smooth
                // multisines while preserving the configured amplitude bound.
                const double ramp = config.rich_identification ? 1.0 : 0.5;
                const double envelope = std::min({1.0, t / ramp,
                                                   (config.duration - t) / ramp});
                if (!config.rich_identification || out_.excitation_profile == 0) {
                    const double f0 = config.rich_identification
                        ? 0.5 * config.start_hz : config.start_hz;
                    const double f1 = config.rich_identification
                        ? 0.4 * config.end_hz : config.end_hz;
                    const double rate = (f1 - f0) / config.duration;
                    const double factor = config.rich_identification ? 0.55 : 1.0;
                    out_.excitation = factor * config.amplitude * envelope
                        * std::sin(2 * std::numbers::pi * (f0*t + 0.5*rate*t*t));
                } else if (out_.excitation_profile <= 2) {
                    const bool reverse = out_.excitation_profile == 2;
                    const double low = config.start_hz;
                    const double high = 0.8 * config.end_hz;
                    const double f0 = reverse ? high : low;
                    const double f1 = reverse ? low : high;
                    const double rate = (f1 - f0) / config.duration;
                    out_.excitation = 0.75 * config.amplitude * envelope
                        * std::sin(2 * std::numbers::pi * (f0*t + 0.5*rate*t*t));
                } else {
                    constexpr double frequencies[] = {0.13, 0.29, 0.53, 0.91, 1.57};
                    constexpr double weights[] = {1.0, 0.9, 0.8, 0.65, 0.5};
                    constexpr double weight_sum = 3.85;
                    const double phase = 0.37 * static_cast<double>(excitation_trial_);
                    double multisine = 0.0;
                    for (std::size_t i = 0; i < 5; ++i)
                        multisine += weights[i] * std::sin(
                            2 * std::numbers::pi * frequencies[i] * t
                            + phase * static_cast<double>(i + 1));
                    constexpr double factors[] = {0.65, 0.82, 1.0};
                    const double factor = factors[out_.excitation_profile - 3];
                    out_.excitation = factor * config.amplitude * envelope
                        * multisine / weight_sum;
                }
                out_.elapsed += dt;
            }
        }
        out_.yaw_error = std::remainder(
            center_ + out_.yaw_target_offset - in.yaw, 2 * std::numbers::pi);
        const double raw_yaw_torque = yaw_velocity.update(
            yaw_angle.update(out_.yaw_error) - yaw_control_velocity_)
            + out_.excitation + out_.yaw_feedforward;
        const double limited_yaw_torque = std::clamp(
            raw_yaw_torque, -config.yaw_torque_limit, config.yaw_torque_limit);
        if (in.pitch_world >= config.min_world_pitch) {
            const double yaw_delta = limited_yaw_torque - out_.yaw_torque;
            // Remove accelerating torque promptly when feedback calls for
            // braking. The old symmetric 12 Nm/s limiter kept pushing yaw
            // toward an overspeed fault after the velocity loop reversed.
            const bool yaw_releasing = yaw_delta * out_.yaw_torque < 0.0;
            const double yaw_slew = yaw_releasing ? config.yaw_torque_release_slew
                                                  : config.yaw_torque_slew;
            out_.yaw_torque += std::clamp(yaw_delta, -yaw_slew * dt, yaw_slew * dt);
        } else
            out_.yaw_torque = 0.0;
        return out_;
    }

private:
    bool pitch_within_limits(double pitch) const {
        const double checked = config.pitch_limits_relative_to_arm
            ? std::remainder(pitch - pitch_arm_origin_, 2 * std::numbers::pi)
            : pitch;
        return checked >= config.pitch_min - 0.02 && checked <= config.pitch_max + 0.02;
    }
    static bool finite(const Input& in) {
        for (double v : {in.yaw, in.pitch, in.pitch_world, in.yaw_velocity, in.pitch_velocity,
                        in.yaw_temperature, in.pitch_temperature})
            if (!std::isfinite(v)) return false;
        return true;
    }
    void reset(State state) {
        out_ = {}; out_.state = state;
        stable_ = raising_ = stalled_ = stall_window_ = 0.0;
        readiness_pitch_velocity_ = 0.0;
        pitch_control_velocity_ = yaw_control_velocity_ = 0.0;
        pitch_precharge_elapsed_ = 0.0;
        pitch_arm_origin_ = 0.0;
        pitch_angle.reset(); pitch_velocity.reset(); yaw_angle.reset(); yaw_velocity.reset();
    }
    Output fault(int reason) { reset(State::Fault); out_.fault = reason; return out_; }
    Output out_;
    double center_ = 0, stable_ = 0, raising_ = 0, stalled_ = 0;
    double stall_anchor_ = 0, stall_window_ = 0;
    double readiness_pitch_velocity_ = 0;
    double pitch_control_velocity_ = 0, yaw_control_velocity_ = 0;
    double pitch_precharge_elapsed_ = 0;
    double pitch_arm_origin_ = 0;
    std::size_t sweep_count_ = 0;
    std::size_t excitation_trial_ = 0;
};
}
