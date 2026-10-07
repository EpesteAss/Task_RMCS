#include <cassert>

#include "hardware/gantry_homing.hpp"

using rmcs_core::hardware::GantryHoming;

static GantryHoming make_homing() {
    return GantryHoming({.contact_torque_nm = 0.25,
                         .contact_speed_rad_s = 0.05,
                         .contact_hold_s = 0.5,
                         .timeout_s = 10.0,
                         .max_travel_rad = 20.0});
}

static GantryHoming::Input input() {
    return {.valid = true, .time_s = 1.0, .angle_rad = {}, .speed_rad_s = {},
            .torque_nm = {}, .commanded_torque_nm = {}};
}

int main() {
    auto homing = make_homing();
    auto in = input();
    in.valid = false;
    assert(homing.update(in).state == GantryHoming::State::Waiting);
    in.valid = true;
    auto out = homing.update(in);
    assert(out.state == GantryHoming::State::Seeking);
    assert(out.drive_up[0] && out.drive_up[1]);

    in.time_s = 1.2;
    in.commanded_torque_nm = {0.3, 0.3};
    in.torque_nm = {0.3, 0.1};
    in.angle_rad = {1.0, 1.0};
    in.speed_rad_s = {0.0, 0.0};
    out = homing.update(in);
    assert(!out.reached[0] && !out.reached[1]);
    in.time_s = 1.75;
    out = homing.update(in);
    assert(out.reached[0] && !out.reached[1]);
    assert(!out.drive_up[0] && out.drive_up[1]);

    in.time_s = 2.0;
    in.torque_nm[1] = 0.3;
    out = homing.update(in);
    assert(out.drive_up[1]);
    in.time_s = 2.55;
    out = homing.update(in);
    assert(out.state == GantryHoming::State::Complete);
    assert(!out.drive_up[0] && !out.drive_up[1]);
    in.valid = false;
    assert(homing.update(in).state == GantryHoming::State::Complete);

    auto timeout = make_homing();
    in = input();
    timeout.update(in);
    in.time_s = 11.1;
    assert(timeout.update(in).state == GantryHoming::State::Failed);
    assert(!timeout.update(in).drive_up[0]);

    auto lost_feedback = make_homing();
    in = input();
    lost_feedback.update(in);
    in.valid = false;
    assert(lost_feedback.update(in).state == GantryHoming::State::Failed);

    auto travel = make_homing();
    in = input();
    travel.update(in);
    in.angle_rad[0] = 20.1;
    assert(travel.update(in).state == GantryHoming::State::Failed);

    auto wrong_direction = make_homing();
    in = input();
    wrong_direction.update(in);
    in.angle_rad[1] = -0.2;
    assert(wrong_direction.update(in).state == GantryHoming::State::Failed);
}
