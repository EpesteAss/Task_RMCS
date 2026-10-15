#include <cmath>
#include <limits>
#include <source_location>
#include <stdexcept>
#include "yaw_identification/engine.hpp"

using namespace yaw_identification;
void check(bool ok, std::source_location where = std::source_location::current()) {
    if (!ok) throw std::runtime_error("Physical identification test failed at line "
                                     + std::to_string(where.line()));
}

Engine engine() {
    Engine e;
    e.config.physical_identification = true;
    e.config.world_pitch_control = true;
    e.config.duration = 4.0;
    e.config.amplitude = 0.2;
    e.config.yaw_torque_limit = 0.4;
    e.config.yaw_speed_trip = 0.4;
    e.config.gravity_gain = 3.5;
    e.config.gravity_phase = 1.784;
    e.config.pitch_torque_limit = 4.5;
    // An accidental PID update would return NaN. Physical mode must ignore it.
    e.yaw_angle.kp = e.yaw_velocity.kp = std::numeric_limits<double>::quiet_NaN();
    e.validate();
    return e;
}

Input input() {
    return {.pitch_world=5.0 * std::numbers::pi / 180.0,
            .yaw_temperature=30, .pitch_temperature=30, .heartbeat=true};
}

void ready(Engine& e, Input in) {
    e.update(in, Command::Arm, .001);
    for (int i=0; i<1100; ++i) {
        const auto o = e.update(in, Command::None, .001);
        check(o.yaw_torque == 0 && o.yaw_pid_torque == 0 && o.yaw_feedforward == 0);
    }
}

int main() {
    Engine a = engine(), b = engine();
    Input in = input(), other = input();
    auto o = a.update(in, Command::None, .001);
    check(o.state == State::Off && o.yaw_torque == 0);
    ready(a, in); ready(b, other);
    for (int profile=0; profile<6; ++profile) {
        o = a.update(in, Command::Sweep, .001);
        auto different = b.update(other, Command::Sweep, .001);
        check(o.state == State::Sweep && o.excitation_profile == profile);
        double peak = 0, impulse = 0, moment = 0;
        for (int i=0; i<4100; ++i) {
            // Different measured positions/speeds must produce identical
            // torque while inside protective thresholds: no yaw feedback.
            other.yaw = .04 * std::sin(i * .013);
            other.yaw_velocity = .02 * std::cos(i * .017);
            o = a.update(in, Command::None, .001);
            different = b.update(other, Command::None, .001);
            check(std::isfinite(o.yaw_torque) && o.yaw_pid_torque == 0 && o.yaw_feedforward == 0);
            check(std::abs(o.yaw_torque - different.yaw_torque) < 1e-12);
            check(std::abs(o.yaw_torque) <= .2 + 1e-12);
            peak = std::max(peak, std::abs(o.yaw_torque));
            impulse += o.yaw_torque * .001;
            moment += i * .001 * o.yaw_torque * .001;
        }
        check(peak > .19 && std::abs(impulse) < 1e-6 && std::abs(moment) < 1e-6);
        check(o.state == State::Ready && o.completed && o.yaw_torque == 0);
        check(different.yaw_torque == 0);
        other.yaw_velocity = 0; // Rest before the next profile.
    }
    Engine pure = engine();
    pure.config.physical_pure_sine = true;
    pure.validate();
    Input pure_input = input();
    ready(pure, pure_input);
    o = pure.update(pure_input, Command::Sweep, .001);
    double pure_min = 0, pure_max = 0, pure_impulse = 0;
    for (int i=0; i<4100; ++i) {
        o = pure.update(pure_input, Command::None, .001);
        pure_min = std::min(pure_min, o.yaw_torque);
        pure_max = std::max(pure_max, o.yaw_torque);
        pure_impulse += o.yaw_torque * .001;
    }
    check(pure_min < -.19 && pure_max > .19);
    check(std::abs(pure_impulse) < 1e-6);
    check(o.state == State::Ready && o.completed && o.yaw_torque == 0);
    o = a.update(in, Command::Sweep, .001);
    o = a.update(in, Command::Hold, .001);
    check(o.state == State::Ready && !o.completed && o.yaw_torque == 0);
    a.update(in, Command::Sweep, .001);
    in.yaw = .9 * a.config.yaw_span;
    o = a.update(in, Command::None, .001);
    check(o.state == State::Ready && o.physical_guard_stop == 1 && !o.completed);
    check(o.yaw_torque == 0 && std::abs(o.pitch_torque) > 1);

    Engine boundary = engine(); in = input(); ready(boundary, in);
    o = boundary.update(in, Command::Sweep, .001);
    while (o.elapsed < boundary.config.duration)
        o = boundary.update(in, Command::None, .001);
    in.yaw = .9 * boundary.config.yaw_span;
    o = boundary.update(in, Command::None, .001);
    check(o.state == State::Ready && o.physical_guard_stop == 1 && !o.completed);

    Engine speed = engine(); in = input(); ready(speed, in);
    in.yaw_velocity = .04;
    o = speed.update(in, Command::Sweep, .001);
    check(o.state == State::Ready && o.yaw_torque == 0); // Require rest.
    in.yaw_velocity = 0;
    speed.update(in, Command::Sweep, .001);
    in.yaw_velocity = .35;
    o = speed.update(in, Command::None, .001);
    check(o.state == State::Ready && o.physical_guard_stop == 2 && !o.completed && o.yaw_torque == 0);
    in.yaw_velocity = .45;
    o = speed.update(in, Command::None, .001);
    check(o.state == State::Fault && o.fault == 4 && o.yaw_torque == 0);

    for (int reason : {1, 5, 6}) {
        Engine protected_engine = engine(); in = input(); ready(protected_engine, in);
        protected_engine.update(in, Command::Sweep, .001);
        if (reason == 1) in.heartbeat = false;
        if (reason == 5) in.yaw = protected_engine.config.yaw_span + .001;
        if (reason == 6) in.yaw_temperature = 65;
        o = protected_engine.update(in, Command::None, .001);
        check(o.state == State::Fault && o.fault == reason && o.yaw_torque == 0);
    }
    Engine hot_yaw = engine();
    hot_yaw.config.yaw_temperature_trip_c = 130;
    hot_yaw.validate();
    in = input();
    in.yaw_temperature = 125;
    ready(hot_yaw, in);
    o = hot_yaw.update(in, Command::Sweep, .001);
    check(o.state == State::Sweep);
    in.yaw_temperature = 130;
    o = hot_yaw.update(in, Command::None, .001);
    check(o.state == State::Fault && o.fault == 6 && o.yaw_torque == 0);
    Engine pitch_limit = engine();
    pitch_limit.config.yaw_temperature_trip_c = 130;
    in = input(); in.yaw_temperature = 125;
    ready(pitch_limit, in);
    pitch_limit.update(in, Command::Sweep, .001);
    in.pitch_temperature = 65;
    o = pitch_limit.update(in, Command::None, .001);
    check(o.state == State::Fault && o.fault == 6 && o.yaw_torque == 0);
    Engine clearing = engine(); in = input(); ready(clearing, in);
    clearing.update(in, Command::Sweep, .001);
    o = clearing.update(in, Command::Off, .001);
    check(o.state == State::Releasing && o.yaw_torque == 0 && o.yaw_pid_torque == 0);
    for (int i=0; i<5000; ++i) o = clearing.update(in, Command::None, .001);
    check(o.state == State::Off && o.yaw_torque == 0 && o.pitch_torque == 0);
    for (int invalid=0; invalid<3; ++invalid) {
        Engine bad = engine();
        if (invalid == 0) bad.config.tracking_test = true;
        if (invalid == 1) bad.config.yaw_ff_bias = .01;
        if (invalid == 2) bad.config.physical_frequencies = {.25};
        bool rejected = false;
        try { bad.validate(); } catch (const std::runtime_error&) { rejected = true; }
        check(rejected);
    }
    Engine passive = engine();
    passive.config.passive_pitch = true;
    passive.config.amplitude = .75;
    passive.config.yaw_torque_limit = .8;
    passive.config.yaw_temperature_trip_c = 130;
    passive.validate();
    in = input(); in.pitch_world = -8 * std::numbers::pi / 180;
    in.yaw_temperature = 125;
    in.pitch_velocity = .06;
    o = passive.update(in, Command::Arm, .001);
    check(o.state == State::Off && o.pitch_torque == 0);
    in.pitch_velocity = 0;
    o = passive.update(in, Command::Arm, .001);
    check(o.state == State::Ready && o.pitch_torque == 0);
    o = passive.update(in, Command::Sweep, .001);
    check(o.state == State::Sweep && o.pitch_torque == 0);
    double passive_peak = 0;
    for (int i=0; i<1000; ++i) {
        o = passive.update(in, Command::None, .001);
        check(o.pitch_torque == 0 && o.pitch_torque_unlimited == 0);
        passive_peak = std::max(passive_peak, std::abs(o.yaw_torque));
    }
    check(passive_peak > .1);
    o = passive.update(in, Command::Off, .001);
    check(o.state == State::Off && o.pitch_torque == 0 && o.yaw_torque == 0);
    in.pitch_world = -11 * std::numbers::pi / 180;
    o = passive.update(in, Command::Arm, .001);
    check(o.state == State::Off && o.pitch_torque == 0);
    in.pitch_world = -8 * std::numbers::pi / 180;
    passive.update(in, Command::Arm, .001);
    passive.update(in, Command::Sweep, .001);
    in.pitch_world = -11 * std::numbers::pi / 180;
    o = passive.update(in, Command::None, .001);
    check(o.state == State::Fault && o.fault == 3 && o.pitch_torque == 0 && o.yaw_torque == 0);
    Engine passive_guard = engine();
    passive_guard.config.passive_pitch = true;
    in = input(); in.pitch_world = -8 * std::numbers::pi / 180;
    passive_guard.update(in, Command::Arm, .001);
    passive_guard.update(in, Command::Sweep, .001);
    in.yaw = .9 * passive_guard.config.yaw_span;
    o = passive_guard.update(in, Command::None, .001);
    check(o.state == State::Ready && o.physical_guard_stop == 1 && o.pitch_torque == 0 && o.yaw_torque == 0);

    Engine fixed = engine();
    fixed.config.passive_pitch = true;
    fixed.config.mechanically_fixed_pitch = true;
    fixed.config.amplitude = .75;
    fixed.config.yaw_torque_limit = .8;
    fixed.config.yaw_temperature_trip_c = 130;
    fixed.config.pitch_torque_limit = .01;
    fixed.validate();
    in = input();
    in.pitch = .680033;          // Stale encoder value while Pitch CAN0 is silent.
    in.pitch_world = -8 * std::numbers::pi / 180; // Also not a trusted pose measurement.
    in.pitch_temperature = 0;   // No Pitch motor feedback.
    in.yaw_temperature = 125;
    o = fixed.update(in, Command::Arm, .001);
    check(o.state == State::Ready && o.pitch_torque == 0);
    o = fixed.update(in, Command::Sweep, .001);
    check(o.state == State::Sweep && o.pitch_torque == 0 && o.yaw_pid_torque == 0);
    for (int i = 0; i < 1000; ++i) {
        o = fixed.update(in, Command::None, .001);
        check(o.pitch_torque == 0 && o.pitch_torque_unlimited == 0);
    }
    check(std::abs(o.yaw_torque) <= .75 + 1e-12);
    in.yaw = .9 * fixed.config.yaw_span;
    o = fixed.update(in, Command::None, .001);
    check(o.state == State::Ready && o.physical_guard_stop == 1
          && o.yaw_torque == 0 && o.pitch_torque == 0);
    Engine invalid_fixed = engine();
    invalid_fixed.config.mechanically_fixed_pitch = true;
    bool fixed_rejected = false;
    try { invalid_fixed.validate(); } catch (const std::runtime_error&) { fixed_rejected = true; }
    check(fixed_rejected);
    Engine reverse = engine();
    reverse.config.passive_pitch = true;
    reverse.config.mechanically_fixed_pitch = true;
    reverse.config.physical_start_profile = 3;
    reverse.config.yaw_span = 4.5 * std::numbers::pi / 180.0;
    reverse.config.tracking_amplitude = 4.0 * std::numbers::pi / 180.0;
    reverse.validate();
    in = input();
    reverse.update(in, Command::Arm, .001);
    o = reverse.update(in, Command::Sweep, .001);
    check(o.state == State::Sweep && o.excitation_profile == 3
          && o.tracking_frequency == .5);
    for (int i = 0; i < 2200; ++i) o = reverse.update(in, Command::None, .001);
    check(o.excitation < 0 && o.yaw_pid_torque == 0 && o.pitch_torque == 0);
    Engine invalid_profile = engine();
    invalid_profile.config.physical_start_profile = 6;
    bool profile_rejected = false;
    try { invalid_profile.validate(); } catch (const std::runtime_error&) { profile_rejected = true; }
    check(profile_rejected);
    Engine step = engine();
    step.config.passive_pitch = true;
    step.config.mechanically_fixed_pitch = true;
    step.config.physical_step_pulse = true;
    step.config.physical_step_end_s = .65;
    step.config.amplitude = .9;
    step.config.yaw_torque_limit = 1.0;
    step.config.yaw_torque_slew = 20.0;
    step.config.yaw_torque_release_slew = 20.0;
    step.validate();
    in = input();
    step.update(in, Command::Arm, .001);
    o = step.update(in, Command::Sweep, .001);
    check(o.state == State::Sweep && o.excitation_profile == 6
          && o.yaw_torque == 0 && o.pitch_torque == 0);
    double step_peak = 0;
    for (int i = 0; i < 4100; ++i) {
        o = step.update(in, Command::None, .001);
        if (i < 490 || i > 1000) check(o.yaw_torque == 0);
        step_peak = std::max(step_peak, o.yaw_torque);
        check(o.yaw_torque >= 0 && o.yaw_torque <= .9 + 1e-12
              && o.yaw_pid_torque == 0 && o.pitch_torque == 0);
    }
    check(step_peak > .89 && o.state == State::Ready && o.completed);
    Engine step_bad = step;
    step_bad.config.physical_step_end_s = step_bad.config.duration;
    bool step_rejected = false;
    try { step_bad.validate(); } catch (const std::runtime_error&) { step_rejected = true; }
    check(step_rejected);
    Engine negative_step = engine();
    negative_step.config.passive_pitch = true;
    negative_step.config.mechanically_fixed_pitch = true;
    negative_step.config.physical_step_pulse = true;
    negative_step.config.physical_step_direction = -1;
    negative_step.config.physical_step_end_s = .65;
    negative_step.config.amplitude = .9;
    negative_step.config.yaw_torque_limit = 1.0;
    negative_step.config.yaw_torque_slew = 20.0;
    negative_step.config.yaw_torque_release_slew = 20.0;
    negative_step.config.yaw_span = 20.0 * std::numbers::pi / 180.0;
    negative_step.config.yaw_speed_trip = .8;
    negative_step.validate();
    in = input();
    negative_step.update(in, Command::Arm, .001);
    o = negative_step.update(in, Command::Sweep, .001);
    check(o.state == State::Sweep && o.excitation_profile == 7);
    double negative_peak = 0;
    for (int i = 0; i < 700; ++i) {
        o = negative_step.update(in, Command::None, .001);
        negative_peak = std::min(negative_peak, o.yaw_torque);
        check(o.yaw_torque <= 0 && o.yaw_torque >= -.9 - 1e-12
              && o.pitch_torque == 0 && o.yaw_pid_torque == 0);
    }
    check(negative_peak < -.89 && o.yaw_torque == 0);
    in.yaw_velocity = -.33;
    o = negative_step.update(in, Command::None, .001);
    check(o.state == State::Sweep && o.physical_guard_stop == 0);
    in.yaw_velocity = -.65;
    o = negative_step.update(in, Command::None, .001);
    check(o.state == State::Ready && o.physical_guard_stop == 2 && o.yaw_torque == 0);

    Engine alternating = engine();
    alternating.config.passive_pitch = true;
    alternating.config.mechanically_fixed_pitch = true;
    alternating.config.physical_alternating_pulses = true;
    alternating.config.amplitude = .9;
    alternating.config.yaw_torque_limit = 1.0;
    alternating.config.yaw_torque_slew = 20.0;
    alternating.config.yaw_torque_release_slew = 20.0;
    alternating.validate();
    in = input();
    alternating.update(in, Command::Arm, .001);
    o = alternating.update(in, Command::Sweep, .001);
    check(o.excitation_profile == 8 && o.pitch_torque == 0);
    int reversals = 0, previous_sign = 0;
    double minimum = 0, maximum = 0;
    for (int i = 0; i < 4100; ++i) {
        o = alternating.update(in, Command::None, .001);
        check(o.pitch_torque == 0 && o.yaw_pid_torque == 0
              && std::abs(o.yaw_torque) <= .9 + 1e-12);
        minimum = std::min(minimum, o.yaw_torque);
        maximum = std::max(maximum, o.yaw_torque);
        if (std::abs(o.yaw_torque) > .8) {
            const int sign = o.yaw_torque > 0 ? 1 : -1;
            if (previous_sign && sign != previous_sign) ++reversals;
            previous_sign = sign;
        }
    }
    check(minimum < -.89 && maximum > .89 && reversals == 5);
    check(o.state == State::Ready && o.completed && o.yaw_torque == 0);
    Engine invalid_alternating = alternating;
    invalid_alternating.config.physical_alternating_cycles = 4;
    bool alternating_rejected = false;
    try { invalid_alternating.validate(); } catch (const std::runtime_error&) { alternating_rejected = true; }
    check(alternating_rejected);

    Engine parking = engine();
    parking.config.controlled_pitch_parking = true;
    parking.config.pitch_limits_relative_to_arm = true;
    parking.config.world_pitch_target = 5 * std::numbers::pi / 180;
    parking.config.pitch_park_world = -8 * std::numbers::pi / 180;
    parking.config.pitch_park_ramp = 2 * std::numbers::pi / 180;
    parking.config.yaw_temperature_trip_c = 130;
    parking.validate();
    in = input();
    in.pitch_world = -8 * std::numbers::pi / 180;
    in.pitch = .18705;
    in.yaw_temperature = 125;
    const double arm_pitch = in.pitch;
    Input wrong_start = in;
    wrong_start.pitch_world = 5 * std::numbers::pi / 180;
    auto rejected = parking.update(wrong_start, Command::Arm, .001);
    check(rejected.state == State::Off && rejected.pitch_torque == 0);
    o = parking.update(in, Command::Arm, .001);
    auto follow_pitch_target = [&] {
        const double desired_world = -o.pitch_target;
        const double delta_world = std::clamp(desired_world - in.pitch_world,
                                              -.20 * .001, .20 * .001);
        in.pitch_world += delta_world;
        in.pitch_velocity = -delta_world / .001;
        in.pitch = arm_pitch - (in.pitch_world - parking.config.pitch_park_world);
        o = parking.update(in, Command::None, .001);
    };
    for (int i=0; i<9000 && o.state != State::Ready; ++i) follow_pitch_target();
    check(o.state == State::Ready && in.pitch_world > 4 * std::numbers::pi / 180);
    Engine slow_release = parking;
    slow_release.config.controlled_pitch_parking = false;
    slow_release.config.off_slew = .3;
    slow_release.validate();
    Input release_in = in;
    o = parking.update(in, Command::Off, .001);
    check(o.state == State::Parking && o.yaw_torque == 0 && o.pitch_torque != 0);
    for (int i=0; i<11000 && o.state != State::Off; ++i) {
        follow_pitch_target();
        check(o.state == State::Parking || o.state == State::Off);
        check(o.yaw_torque == 0);
    }
    check(o.state == State::Off && o.pitch_torque == 0 && o.yaw_torque == 0);
    check(std::abs(in.pitch_world - parking.config.pitch_park_world)
          <= parking.config.pitch_park_tolerance);

    o = slow_release.update(release_in, Command::Off, .001);
    check(o.state == State::Releasing && o.yaw_torque == 0);
    double previous_torque = o.pitch_torque;
    for (int i=0; i<20000 && o.state != State::Off; ++i) {
        o = slow_release.update(release_in, Command::None, .001);
        check(o.state == State::Releasing || o.state == State::Off);
        check(o.yaw_torque == 0);
        if (o.state == State::Releasing)
            check(o.pitch_torque >= previous_torque - 1e-12
                  && o.pitch_torque - previous_torque <= .0003001);
        previous_torque = o.pitch_torque;
    }
    check(o.state == State::Off && o.pitch_torque == 0);
}
