#include <cassert>

#include "hardware/gantry_switches.hpp"

using rmcs_core::hardware::gantry_switches_enable_motion;
using rmcs_msgs::Switch;

int main() {
    constexpr Switch positions[] = {Switch::UP, Switch::MIDDLE, Switch::DOWN};
    for (Switch left : positions)
        for (Switch right : positions)
            assert(gantry_switches_enable_motion(left, right)
                   == !(left == Switch::DOWN && right == Switch::DOWN));
    for (Switch valid : positions) {
        assert(!gantry_switches_enable_motion(Switch::UNKNOWN, valid));
        assert(!gantry_switches_enable_motion(valid, Switch::UNKNOWN));
    }
}
