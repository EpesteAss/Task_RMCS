#pragma once

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::hardware {

constexpr bool gantry_switches_enable_motion(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
    return left != rmcs_msgs::Switch::UNKNOWN && right != rmcs_msgs::Switch::UNKNOWN
        && !(left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN);
}

} // namespace rmcs_core::hardware
