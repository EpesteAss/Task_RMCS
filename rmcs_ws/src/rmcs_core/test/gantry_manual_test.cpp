#include <cassert>
#include <limits>

#include "controller/motor/gantry_manual_test.hpp"

using rmcs_core::controller::gantry::ManualTest;

static ManualTest::Input input() {
    return {.valid = true, .pitch_stick = 0.0, .yaw_stick = 0.0,
            .angle = {}, .velocity = {}};
}

static void off(const ManualTest::Output& out) {
    for (int i = 0; i < 3; ++i) {
        assert(!out.enabled[i]);
        assert(out.velocity[i] == 0.0);
    }
}

int main() {
    ManualTest test;
    auto in = input();
    in.pitch_stick = 1.0;
    off(test.update(in)); // Require a centered stick once after startup.
    assert(test.update(input()).state == 1);
    auto out = test.update(in);
    assert(out.state == 2 && out.enabled[0] && out.enabled[1] && !out.enabled[2]);
    assert(out.velocity[0] == 10.0 && out.velocity[1] == 10.0);
    for (int i = 0; i < 10000; ++i) {
        in.angle[0] += 0.001;
        in.angle[1] += 0.001;
        assert(test.update(in).state == 2); // Holding the stick has no time/travel cutoff.
    }
    in.pitch_stick = 0.0;
    off(test.update(in)); // Releasing the stick removes motor torque.
    in.yaw_stick = -0.5;
    out = test.update(in);
    assert(out.state == 3 && !out.enabled[0] && !out.enabled[1] && out.enabled[2]);
    assert(out.velocity[2] == -5.0);

    ManualTest sync;
    assert(sync.update(input()).state == 1);
    in = input();
    in.pitch_stick = 0.5;
    in.angle[0] = 1.0; // Left side is ahead: slow left, speed up right.
    out = sync.update(in);
    assert(out.velocity[0] == 4.0 && out.velocity[1] == 6.0);
    in.angle[0] = 0.0;
    in.angle[1] = 1.0;
    out = sync.update(in);
    assert(out.velocity[0] == 6.0 && out.velocity[1] == 4.0);

    in = input();
    in.pitch_stick = in.yaw_stick = 1.0;
    out = sync.update(in);
    assert(out.state == 5 && out.enabled[0] && out.enabled[1] && out.enabled[2]);
    assert(out.velocity[0] == 10.0 && out.velocity[1] == 10.0
           && out.velocity[2] == 10.0);
    in.pitch_stick = 0.0;
    out = sync.update(in);
    assert(out.state == 3 && !out.enabled[0] && !out.enabled[1] && out.enabled[2]);
    in.pitch_stick = 1.0;
    in.yaw_stick = 0.0;
    out = sync.update(in);
    assert(out.state == 2 && out.enabled[0] && out.enabled[1] && !out.enabled[2]);
    off(sync.update(input()));

    ManualTest skew;
    assert(skew.update(input()).state == 1);
    in = input();
    in.pitch_stick = in.yaw_stick = 1.0;
    in.angle[0] = ManualTest::max_lift_difference + 0.001;
    out = skew.update(in);
    assert(out.state == 6 && !out.enabled[0] && out.enabled[1] && out.enabled[2]);
    assert(out.velocity[1] == 2.0);
    assert(out.velocity[2] == 10.0);
    in.pitch_stick = 0.0;
    out = skew.update(in);
    assert(out.state == 6 && !out.enabled[0] && !out.enabled[1] && out.enabled[2]);
    // Releasing pitch does not erase a real skew.
    in.pitch_stick = -1.0;
    out = skew.update(in);
    assert(out.state == 6 && out.enabled[0] && !out.enabled[1] && out.enabled[2]);
    assert(out.velocity[0] == -2.0);
    in.angle[0] = 0.0;
    out = skew.update(in);
    assert(out.state == 5 && out.enabled[0] && out.enabled[1] && out.enabled[2]);

    ManualTest negative_skew;
    assert(negative_skew.update(input()).state == 1);
    in = input();
    in.pitch_stick = -1.0;
    in.angle[1] = ManualTest::max_lift_difference + 0.001;
    out = negative_skew.update(in);
    assert(out.state == 6 && !out.enabled[0] && out.enabled[1]);
    assert(out.velocity[1] == -2.0);

    for (int reason = 0; reason < 4; ++reason) {
        ManualTest check;
        assert(check.update(input()).state == 1);
        in = input();
        in.yaw_stick = 1.0;
        assert(check.update(in).enabled[2]);
        switch (reason) {
        case 0: in.valid = false; break;
        case 1: in.velocity[2] = ManualTest::max_measured_speed + 0.001; break;
        case 2: in.angle[2] = std::numeric_limits<double>::quiet_NaN(); break;
        case 3: in.yaw_stick = std::numeric_limits<double>::infinity(); break;
        }
        off(check.update(in));
        in = input();
        in.yaw_stick = 1.0;
        off(check.update(in)); // Validity recovery alone cannot restart motion.
        assert(check.update(input()).state == 1);
        assert(check.update(in).enabled[2]);
    }
}
