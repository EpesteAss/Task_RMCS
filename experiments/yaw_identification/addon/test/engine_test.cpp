#include <cmath>
#include <stdexcept>
#include "yaw_identification/engine.hpp"
using namespace yaw_identification;
void check(bool ok) { if (!ok) throw std::runtime_error("Safety/state-machine test failed"); }
int main() {
    Engine e;
    e.validate();
    e.config.pitch_torque_limit = 6.0;
    bool rejected_oversized_pitch_limit = false;
    try { e.validate(); }
    catch (const std::runtime_error&) { rejected_oversized_pitch_limit = true; }
    check(rejected_oversized_pitch_limit);
    e.config.pitch_torque_limit = 4.0;
    e.validate();
    Input in{.yaw=1.2, .pitch=0, .yaw_temperature=20, .pitch_temperature=20};
    auto o = e.update(in, Command::None, .001);
    check(o.state == State::Off && o.pitch_torque == 0 && o.yaw_torque == 0);
    o = e.update(in, Command::Arm, .001);
    check(o.state == State::Off);
    in.heartbeat = true;
    o = e.update(in, Command::Arm, .001);
    check(o.state == State::Raising && o.pitch_target < 0 && o.pitch_target > -.001);
    check(std::abs(o.yaw_torque) < 1e-10); // Captures current yaw, not absolute zero.
    o = e.update(in, Command::Sweep, .001);
    check(o.state != State::Sweep);
    for (int i=0; i<3500; ++i) {
        in.pitch = o.pitch_target; // Ideal tracking of the ramp.
        o = e.update(in, Command::None, .001);
        check(std::abs(o.pitch_torque) <= e.config.pitch_torque_limit);
    }
    check(o.state == State::Ready);
    Engine noisy;
    noisy.config.duration = 0.25;
    Input stationary{.pitch=noisy.config.pitch_target, .yaw_temperature=30,
                     .pitch_temperature=30, .heartbeat=true};
    noisy.update(stationary, Command::Arm, .001);
    for (int i=0; i<1500; ++i) {
        stationary.pitch_velocity = i % 20 == 0 ? 0.064 : 0.0;
        o = noisy.update(stationary, Command::None, .001);
    }
    check(o.state == State::Ready);
    stationary.pitch_velocity = 0.064;
    o = noisy.update(stationary, Command::Sweep, .001);
    check(o.state == State::Sweep);
    for (int i=0; i<300; ++i) {
        stationary.pitch_velocity = i % 20 == 0 ? 0.064 : 0.0;
        o = noisy.update(stationary, Command::None, .001);
    }
    check(o.state == State::Ready && o.completed);
    Engine clearance;
    clearance.config.pitch_target = -18.0 * std::numbers::pi / 180.0;
    clearance.config.min_world_pitch = 8.0 * std::numbers::pi / 180.0;
    clearance.validate();
    Input low{.pitch=clearance.config.pitch_target,
              .pitch_world=3.4 * std::numbers::pi / 180.0,
              .yaw_temperature=30, .pitch_temperature=30, .heartbeat=true};
    clearance.update(low, Command::Arm, .001);
    for (int i=0; i<1100; ++i)
        o = clearance.update(low, Command::None, .001);
    check(o.state == State::Ready && o.yaw_torque == 0);
    o = clearance.update(low, Command::Sweep, .001);
    check(o.state == State::Ready && o.excitation == 0 && o.yaw_torque == 0);
    low.pitch_world = 10.0 * std::numbers::pi / 180.0;
    o = clearance.update(low, Command::Sweep, .001);
    check(o.state == State::Sweep);
    low.pitch_world = 3.4 * std::numbers::pi / 180.0;
    o = clearance.update(low, Command::None, .001);
    check(o.state == State::Ready && o.excitation == 0 && o.yaw_torque == 0);
    stationary.pitch += 0.05;
    stationary.pitch_velocity = 0;
    o = noisy.update(stationary, Command::Sweep, .001);
    check(o.state == State::Raising && o.excitation == 0);
    stationary.pitch_velocity = 0.9;
    o = noisy.update(stationary, Command::None, .001);
    check(o.state == State::Fault && o.fault == 4 && o.pitch_torque == 0);
    Engine near_target;
    near_target.config.pitch_target = -10.0 * std::numbers::pi / 180.0;
    Input supported{.pitch=-8.93 * std::numbers::pi / 180.0,
                    .yaw_temperature=30, .pitch_temperature=30, .heartbeat=true};
    near_target.update(supported, Command::Arm, .001);
    for (int i=0; i<1100; ++i)
        o = near_target.update(supported, Command::None, .001);
    check(o.state == State::Ready);
    o = e.update(in, Command::Sweep, .001);
    check(o.state == State::Sweep);
    in.pitch += .05;
    o = e.update(in, Command::None, .001);
    check(o.state == State::Raising && o.excitation == 0);
    in.heartbeat = false;
    o = e.update(in, Command::None, .001);
    check(o.state == State::Fault && o.fault == 1 && o.pitch_torque == 0 && o.yaw_torque == 0);
    in.heartbeat = true;
    o = e.update(in, Command::Arm, .001);
    check(o.state == State::Fault); // A fault requires explicit off/reset.
    o = e.update(in, Command::Off, .001);
    check(o.state == State::Off);
    e.update(in, Command::Arm, .001);
    in.yaw += e.config.yaw_span + .01;
    o = e.update(in, Command::None, .001);
    check(o.state == State::Fault && o.fault == 5 && o.yaw_torque == 0);
    Engine high_load;
    high_load.config.gravity_gain = 100;
    high_load.config.gravity_phase = 1;
    high_load.config.duration = .02;
    in = Input{.pitch=high_load.config.pitch_target, .yaw_temperature=20,
               .pitch_temperature=20, .heartbeat=true};
    high_load.update(in, Command::Arm, .001);
    for (int i=0; i<1500; ++i) {
        o = high_load.update(in, Command::None, .001);
        check(std::abs(o.pitch_torque) <= high_load.config.pitch_torque_limit);
    }
    check(o.state == State::Ready);
    high_load.update(in, Command::Sweep, .001);
    o = high_load.update(in, Command::Hold, .001);
    check(o.state == State::Ready && !o.completed && o.pitch_torque != 0);
    high_load.update(in, Command::Sweep, .001);
    for (int i=0; i<30; ++i) o = high_load.update(in, Command::None, .001);
    check(o.state == State::Ready && o.completed);
    Engine wrapped;
    in = Input{.pitch=350.0 * std::numbers::pi / 180.0, .yaw_temperature=20,
               .pitch_temperature=20, .heartbeat=true};
    o = wrapped.update(in, Command::Arm, .001);
    check(o.state == State::Raising && std::abs(o.pitch_target - wrapped.config.pitch_target) < 1e-10);
    for (int i=0; i<1100; ++i) o = wrapped.update(in, Command::None, .001);
    check(o.state == State::Ready);
    Engine jammed;
    jammed.config.gravity_gain = 3.5;
    jammed.config.gravity_phase = 1.784;
    in = Input{.pitch=0.05, .yaw_temperature=20, .pitch_temperature=20, .heartbeat=true};
    jammed.update(in, Command::Arm, .001);
    for (int i=0; i<2500; ++i) {
        in.pitch_velocity = (i % 2) ? .03 : -.03; // IMU noise, fixed encoder.
        o = jammed.update(in, Command::None, .001);
    }
    check(o.state == State::Fault && o.fault == 8 && o.pitch_torque == 0);
    Engine level_chassis, tilted_chassis;
    for (Engine* engine : {&level_chassis, &tilted_chassis}) {
        engine->config.gravity_gain = 2.575;
        engine->config.gravity_phase = 1.784;
        engine->pitch_angle.output_min = -6.0;
        engine->pitch_angle.output_max = 6.0;
        engine->pitch_velocity.output_min = -20.0;
        engine->pitch_velocity.output_max = 20.0;
    }
    Input level{.pitch=0.0, .pitch_world=0.0, .yaw_temperature=34,
                .pitch_temperature=34, .heartbeat=true};
    Input tilted = level;
    tilted.pitch_world = 1.0;
    level_chassis.update(level, Command::Arm, .001);
    tilted_chassis.update(tilted, Command::Arm, .001);
    const auto level_output = level_chassis.update(level, Command::None, .001);
    const auto tilted_output = tilted_chassis.update(tilted, Command::None, .001);
    check(std::abs(level_output.pitch_torque_unlimited
                   - tilted_output.pitch_torque_unlimited) > 0.1);
    // Regression: the measured load stopped at -5.6 deg with -3.5 Nm feedback.
    // Raising only the total cap would leave the old velocity loop below 4 Nm.
    Engine lifting;
    lifting.config.pitch_torque_limit = 4.0;
    lifting.config.gravity_gain = 2.575;
    lifting.config.gravity_phase = 1.784;
    lifting.pitch_angle.output_min = -0.25;
    lifting.pitch_angle.output_max = 0.25;
    lifting.pitch_angle.integral_min = -0.5;
    lifting.pitch_angle.integral_max = 0.5;
    lifting.pitch_velocity.kp = 7.0;
    lifting.pitch_velocity.output_min = -2.0;
    lifting.pitch_velocity.output_max = 1.0;
    in = Input{.pitch=-5.6 * std::numbers::pi / 180.0, .yaw_temperature=34,
               .pitch_temperature=34, .heartbeat=true};
    o = lifting.update(in, Command::Arm, .001);
    double peak_torque = 0;
    for (int i=0; i<2500; ++i) {
        const double previous_torque = o.pitch_torque;
        o = lifting.update(in, Command::None, .001);
        peak_torque = std::max(peak_torque, std::abs(o.pitch_torque));
        check(std::abs(o.pitch_torque) <= 4.0);
        if (o.state != State::Fault)
            check(std::abs(o.pitch_torque - previous_torque) <= .005 + 1e-12);
    }
    check(std::abs(peak_torque - 4.0) < 1e-10);
    // No motion at the increased cap still trips, regardless of the cause.
    check(o.state == State::Fault && o.fault == 8 && o.pitch_torque == 0);
    // Applying load remains gentle, but releasing obsolete lifting torque must
    // be fast enough for the feedback loop to brake a moving launcher.
    Engine release;
    release.config.pitch_torque_limit = 4.0;
    release.config.torque_slew = 5.0;
    release.config.torque_release_slew = 30.0;
    release.config.gravity_gain = 4.0;
    release.config.gravity_phase = std::numbers::pi / 2;
    release.pitch_angle.kp = release.pitch_angle.ki = release.pitch_angle.kd = 0.0;
    release.pitch_velocity.kp = release.pitch_velocity.ki = release.pitch_velocity.kd = 0.0;
    Input loaded{.pitch=release.config.pitch_target, .yaw_temperature=30,
                 .pitch_temperature=30, .heartbeat=true};
    o = release.update(loaded, Command::Arm, .001);
    for (int i=0; i<800; ++i) o = release.update(loaded, Command::None, .001);
    check(std::abs(o.pitch_torque + 4.0) < 1e-10);
    release.config.gravity_gain = 0.0;
    o = release.update(loaded, Command::None, .01);
    check(std::abs(o.pitch_torque + 3.7) < 1e-10);
    // Exercise the increased sweep across a full trial and its stop paths.
    Engine doubled;
    doubled.config.amplitude = 0.96;
    doubled.config.yaw_torque_limit = 3.6;
    doubled.config.pitch_target = -18.0 * std::numbers::pi / 180.0;
    doubled.config.min_world_pitch = 8.0 * std::numbers::pi / 180.0;
    doubled.yaw_velocity.output_min = -2.4;
    doubled.yaw_velocity.output_max = 2.4;
    doubled.validate();
    Input held{.pitch=doubled.config.pitch_target, .pitch_world=0.2,
               .yaw_temperature=30, .pitch_temperature=30, .heartbeat=true};
    doubled.update(held, Command::Arm, .001);
    for (int i=0; i<1100; ++i) doubled.update(held, Command::None, .001);
    o = doubled.update(held, Command::Sweep, .001);
    check(o.state == State::Sweep);
    double positive_peak = 0, negative_peak = 0;
    for (int i=0; i<20100; ++i) {
        o = doubled.update(held, Command::None, .001);
        check(std::isfinite(o.yaw_torque) && std::abs(o.yaw_torque) <= 3.6);
        positive_peak = std::max(positive_peak, o.excitation);
        negative_peak = std::min(negative_peak, o.excitation);
    }
    check(o.state == State::Ready && o.completed);
    check(positive_peak > 0.959 && negative_peak < -0.959);
    doubled.update(held, Command::Sweep, .001);
    held.heartbeat = false;
    o = doubled.update(held, Command::None, .001);
    check(o.state == State::Fault && o.fault == 1 && o.yaw_torque == 0 && o.pitch_torque == 0);
    held.heartbeat = true;
    doubled.update(held, Command::Off, .001);
    doubled.update(held, Command::Arm, .001);
    held.yaw += doubled.config.yaw_span + .001;
    o = doubled.update(held, Command::None, .001);
    check(o.state == State::Fault && o.fault == 5 && o.yaw_torque == 0);
    // World-pitch target uses the opposite sign to the motor encoder.
    Engine world;
    world.config.world_pitch_control = true;
    world.config.world_pitch_target = 5.0 * std::numbers::pi / 180.0;
    world.config.min_world_pitch = 3.0 * std::numbers::pi / 180.0;
    world.config.yaw_span = 15.0 * std::numbers::pi / 180.0;
    world.config.amplitude = .96;
    world.config.yaw_torque_limit = 3.6;
    world.validate();
    Input at_level{.pitch=-.10, .pitch_world=0,
                   .yaw_temperature=30, .pitch_temperature=30, .heartbeat=true};
    o = world.update(at_level, Command::Arm, .001);
    check(o.pitch_target < 0 && o.pitch_torque < 0);
    at_level.pitch_world = world.config.world_pitch_target;
    for (int i=0; i<1600; ++i) world.update(at_level, Command::None, .001);
    // 72 full sweeps plus holding pauses: simulate a 30-minute session without hardware.
    for (int cycle=0; cycle<72; ++cycle) {
        o = world.update(at_level, Command::Sweep, .001);
        check(o.state == State::Sweep);
        for (int i=0; i<20010; ++i) {
            o = world.update(at_level, Command::None, .001);
            check(o.state != State::Fault && std::abs(o.yaw_torque) <= 3.6);
        }
        check(o.state == State::Ready && o.completed);
        for (int i=0; i<5000; ++i) world.update(at_level, Command::None, .001);
    }
    at_level.yaw = 14.9 * std::numbers::pi / 180.0;
    o = world.update(at_level, Command::None, .001);
    check(o.state != State::Fault);
    at_level.yaw = 15.1 * std::numbers::pi / 180.0;
    o = world.update(at_level, Command::None, .001);
    check(o.state == State::Fault && o.fault == 5);
    world.update(at_level, Command::Off, .001);
    at_level.yaw_temperature = 65;
    o = world.update(at_level, Command::Arm, .001);
    check(o.state == State::Fault && o.fault == 6 && o.pitch_torque == 0);
    // Raising with staged feedforward must retain the previously tested
    // low-angle command, then add holding compensation near clearance.
    Engine staged, previous;
    for (Engine* engine : {&staged, &previous}) {
        engine->config.world_pitch_control = true;
        engine->config.min_world_pitch = 3.0 * std::numbers::pi / 180.0;
        engine->config.gravity_gain = 2.575;
        engine->config.gravity_phase = 1.784;
    }
    staged.config.staged_gravity_gain = true;
    staged.config.gravity_raise_gain = 2.575;
    staged.config.gravity_gain = 3.5;
    Input before_clearance{.pitch_world=-9.0 * std::numbers::pi / 180.0,
                           .yaw_temperature=30, .pitch_temperature=30, .heartbeat=true};
    const auto low_staged = staged.update(before_clearance, Command::Arm, .001);
    const auto low_previous = previous.update(before_clearance, Command::Arm, .001);
    check(std::abs(low_staged.pitch_torque_unlimited - low_previous.pitch_torque_unlimited) < 1e-10);
    before_clearance.pitch_world = 4.0 * std::numbers::pi / 180.0;
    const auto high_staged = staged.update(before_clearance, Command::None, .001);
    const auto high_previous = previous.update(before_clearance, Command::None, .001);
    const double expected_extra = (3.5 - 2.575)
        * std::sin(before_clearance.pitch_world - 1.784);
    check(std::abs((high_staged.pitch_torque_unlimited - high_previous.pitch_torque_unlimited)
                   - expected_extra) < 1e-10);
    Engine tracking;
    tracking.config.world_pitch_control = true;
    tracking.config.world_pitch_target = 5.0 * std::numbers::pi / 180.0;
    tracking.config.min_world_pitch = 3.0 * std::numbers::pi / 180.0;
    tracking.config.tracking_test = true;
    tracking.config.tracking_amplitude = 5.0 * std::numbers::pi / 180.0;
    tracking.config.tracking_frequency = 0.25;
    tracking.config.tracking_ramp = 0.5;
    tracking.config.duration = 4.0;
    tracking.config.yaw_span = 15.0 * std::numbers::pi / 180.0;
    tracking.config.yaw_torque_limit = 3.6;
    tracking.config.yaw_torque_slew = 12.0;
    tracking.config.yaw_ff_velocity = 3.3;
    tracking.config.yaw_ff_acceleration = 0.3;
    tracking.config.yaw_ff_bias = 0.05;
    tracking.validate();
    Input tracking_input{.pitch_world=tracking.config.world_pitch_target,
                         .yaw_temperature=30, .pitch_temperature=30, .heartbeat=true};
    tracking.update(tracking_input, Command::Arm, .001);
    for (int i=0; i<1100; ++i) tracking.update(tracking_input, Command::None, .001);
    o = tracking.update(tracking_input, Command::Sweep, .001);
    check(o.state == State::Sweep);
    double target_peak = 0.0;
    double previous_yaw_torque = o.yaw_torque;
    for (int i=0; i<4100; ++i) {
        o = tracking.update(tracking_input, Command::None, .001);
        target_peak = std::max(target_peak, std::abs(o.yaw_target_offset));
        check(std::abs(o.yaw_torque) <= 3.6 + 1e-12);
        check(std::abs(o.yaw_torque - previous_yaw_torque) <= 0.012 + 1e-12);
        previous_yaw_torque = o.yaw_torque;
    }
    check(o.state == State::Ready && o.completed);
    check(target_peak > 4.99 * std::numbers::pi / 180.0);
    check(o.yaw_target_offset == 0.0 && o.yaw_feedforward == 0.0);
    return 0;
}
