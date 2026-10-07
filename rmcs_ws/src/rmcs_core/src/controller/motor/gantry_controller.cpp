#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/motor/gantry_kinematics.hpp"
#include "controller/motor/gantry_manual_test.hpp"
#include "hardware/gantry_switches.hpp"

namespace rmcs_core::controller::gantry {
class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        geometry_ready_ = get_parameter("geometry_ready").as_bool();
        get_parameter_or("manual_test_mode", manual_test_mode_, false);
        if (manual_test_mode_ && geometry_ready_)
            throw std::invalid_argument("Manual test and calibrated position control are exclusive");
        front_distance_m_ = get_parameter("front_distance_m").as_double();
        pitch_height_at_start_m_ = get_parameter("pitch_height_at_start_m").as_double();
        yaw_position_at_start_m_ = get_parameter("yaw_position_at_start_m").as_double();
        pitch_lead_m_ = get_parameter("pitch_lead_m_per_rev").as_double();
        yaw_lead_m_ = get_parameter("yaw_lead_m_per_rev").as_double();
        pitch_min_rad_ = get_parameter("pitch_min_rad").as_double();
        pitch_max_rad_ = get_parameter("pitch_max_rad").as_double();
        yaw_min_rad_ = get_parameter("yaw_min_rad").as_double();
        yaw_max_rad_ = get_parameter("yaw_max_rad").as_double();
        pitch_travel_min_m_ = get_parameter("pitch_travel_min_m").as_double();
        pitch_travel_max_m_ = get_parameter("pitch_travel_max_m").as_double();
        yaw_travel_min_m_ = get_parameter("yaw_travel_min_m").as_double();
        yaw_travel_max_m_ = get_parameter("yaw_travel_max_m").as_double();
        max_pitch_rate_rad_s_ = get_parameter("max_pitch_rate_rad_s").as_double();
        max_yaw_rate_rad_s_ = get_parameter("max_yaw_rate_rad_s").as_double();
        joystick_deadband_ = get_parameter("joystick_deadband").as_double();
        sync_gain_ = get_parameter("sync_gain").as_double();
        max_sync_error_m_ = get_parameter("max_sync_error_m").as_double();
        max_pitch_tracking_error_m_ = get_parameter("max_pitch_tracking_error_m").as_double();
        get_parameter_or("homing_speed_rad_s", homing_speed_rad_s_, 5.0);
        if (!std::isfinite(pitch_lead_m_) || pitch_lead_m_ <= 0.0 || !std::isfinite(yaw_lead_m_)
            || yaw_lead_m_ <= 0.0 || !std::isfinite(max_pitch_rate_rad_s_)
            || max_pitch_rate_rad_s_ <= 0.0 || !std::isfinite(max_yaw_rate_rad_s_)
            || max_yaw_rate_rad_s_ <= 0.0 || !std::isfinite(joystick_deadband_)
            || joystick_deadband_ < 0.0 || joystick_deadband_ >= 1.0 || !std::isfinite(sync_gain_)
            || sync_gain_ < 0.0 || !std::isfinite(max_sync_error_m_) || max_sync_error_m_ <= 0.0
            || !std::isfinite(max_pitch_tracking_error_m_) || max_pitch_tracking_error_m_ <= 0.0
            || !std::isfinite(homing_speed_rad_s_) || homing_speed_rad_s_ <= 0.0)
            throw std::invalid_argument("Invalid gantry controller parameter");

        if (geometry_ready_
            && (!std::isfinite(front_distance_m_) || front_distance_m_ <= 0.0
                || !std::isfinite(pitch_height_at_start_m_)
                || !std::isfinite(yaw_position_at_start_m_) || !std::isfinite(pitch_min_rad_)
                || !std::isfinite(pitch_max_rad_) || pitch_min_rad_ >= pitch_max_rad_
                || pitch_min_rad_ <= -std::numbers::pi / 2 || pitch_max_rad_ >= std::numbers::pi / 2
                || !std::isfinite(yaw_min_rad_) || !std::isfinite(yaw_max_rad_)
                || yaw_min_rad_ >= yaw_max_rad_ || yaw_min_rad_ <= -std::numbers::pi / 2
                || yaw_max_rad_ >= std::numbers::pi / 2 || !std::isfinite(pitch_travel_min_m_)
                || !std::isfinite(pitch_travel_max_m_) || pitch_travel_min_m_ >= pitch_travel_max_m_
                || !std::isfinite(yaw_travel_min_m_) || !std::isfinite(yaw_travel_max_m_)
                || yaw_travel_min_m_ >= yaw_travel_max_m_))
            throw std::invalid_argument("Gantry geometry or travel limits are not calibrated");
        if (manual_test_mode_)
            RCLCPP_WARN(
                get_logger(), "Single-loop manual speed control: up to 10 rad/s while held; "
                              "both switches DOWN disables manual motion");
        else if (!geometry_ready_)
            RCLCPP_WARN(
                get_logger(), "Gantry motion locked: calibrate geometry and travel in gantry.yaml");

        register_input("/gantry/left_position_m", left_position_m_);
        register_input("/gantry/right_position_m", right_position_m_);
        register_input("/gantry/yaw_position_m", yaw_position_m_);
        register_input("/gantry/left_motor/velocity", left_velocity_);
        register_input("/gantry/right_motor/velocity", right_velocity_);
        register_input("/gantry/yaw_motor/velocity", yaw_velocity_);
        register_input("/gantry/feedback_valid", feedback_valid_);
        register_input("/gantry/remote_valid", remote_valid_);
        register_input("/gantry/fault", hardware_fault_);
        register_input("/gantry/homing_state", homing_state_);
        register_input("/remote/joystick/left", left_joystick_);
        register_input("/remote/joystick/right", right_joystick_);
        register_input("/remote/switch/left", left_switch_);
        register_input("/remote/switch/right", right_switch_);
        register_input("/predefined/update_rate", update_rate_);

        register_output("/gantry/control_enabled", control_enabled_, false);
        register_output("/gantry/left_motor/angle_error", left_angle_error_, 0.0);
        register_output("/gantry/right_motor/angle_error", right_angle_error_, 0.0);
        register_output("/gantry/yaw_motor/angle_error", yaw_angle_error_, 0.0);
        register_output("/gantry/average_height_m", average_height_m_, 0.0);
        register_output("/gantry/sync_error_m", sync_error_m_, 0.0);
        register_output("/gantry/pitch_rad", pitch_rad_, 0.0);
        register_output("/gantry/yaw_rad", yaw_rad_, 0.0);
        register_output("/gantry/pitch_target_rad", pitch_target_output_, 0.0);
        register_output("/gantry/yaw_target_rad", yaw_target_output_, 0.0);
        register_output("/gantry/left_motor/enabled", left_enabled_, false);
        register_output("/gantry/right_motor/enabled", right_enabled_, false);
        register_output("/gantry/yaw_motor/enabled", yaw_enabled_, false);
        register_output("/gantry/left_motor/target_velocity", left_target_velocity_, 0.0);
        register_output("/gantry/right_motor/target_velocity", right_target_velocity_, 0.0);
        register_output("/gantry/yaw_motor/target_velocity", yaw_target_velocity_, 0.0);
        register_output("/gantry/test/left_velocity", test_left_velocity_, 0.0);
        register_output("/gantry/test/right_velocity", test_right_velocity_, 0.0);
        register_output("/gantry/test/yaw_velocity", test_yaw_velocity_, 0.0);
        register_output("/gantry/debug/left_stick_x", debug_left_x_, 0.0);
        register_output("/gantry/debug/left_stick_y", debug_left_y_, 0.0);
        register_output("/gantry/debug/right_stick_x", debug_right_x_, 0.0);
        register_output("/gantry/debug/right_stick_y", debug_right_y_, 0.0);
        register_output("/gantry/debug/left_switch", debug_left_switch_, 0.0);
        register_output("/gantry/debug/right_switch", debug_right_switch_, 0.0);
        register_output("/gantry/debug/remote_valid", debug_remote_valid_, 0.0);
        register_output("/gantry/debug/feedback_valid", debug_feedback_valid_, 0.0);
        register_output("/gantry/debug/hardware_fault", debug_hardware_fault_, 0.0);
        register_output("/gantry/debug/test_state", debug_test_state_, 0.0);
    }

    void update() override {
        *left_enabled_ = *right_enabled_ = *yaw_enabled_ = false;
        *test_left_velocity_ = *test_right_velocity_ = *test_yaw_velocity_ = 0.0;
        *left_target_velocity_ = *right_target_velocity_ = *yaw_target_velocity_ = 0.0;
        *debug_left_x_ = left_joystick_->x();
        *debug_left_y_ = left_joystick_->y();
        *debug_right_x_ = right_joystick_->x();
        *debug_right_y_ = right_joystick_->y();
        *debug_left_switch_ = static_cast<double>(*left_switch_);
        *debug_right_switch_ = static_cast<double>(*right_switch_);
        *debug_remote_valid_ = *remote_valid_;
        *debug_feedback_valid_ = *feedback_valid_;
        *debug_hardware_fault_ = *hardware_fault_;
        const double left = *left_position_m_;
        const double right = *right_position_m_;
        const double yaw_displacement = *yaw_position_m_;
        const double average_travel = (left + right) / 2.0;
        const double sync_error = left - right;
        *average_height_m_ = pitch_height_at_start_m_ + average_travel;
        *sync_error_m_ = sync_error;

        const double lateral = yaw_position_at_start_m_ + yaw_displacement;
        if (geometry_ready_) {
            *pitch_rad_ = pitch_from_position(front_distance_m_, lateral, *average_height_m_);
            *yaw_rad_ = yaw_from_position(front_distance_m_, lateral);
        }

        const bool remote_enabled = rmcs_core::hardware::gantry_switches_enable_motion(
            *left_switch_, *right_switch_);
        // During homing, the joystick cannot affect any target. The hardware
        // component independently masks a lift as soon as it finds its stop.
        if (*homing_state_ != 2.0) {
            *control_enabled_ = false;
            *debug_test_state_ = 0.0;
            if (*homing_state_ == 1.0 && *feedback_valid_ && *remote_valid_
                && !*hardware_fault_) {
                *left_target_velocity_ = homing_speed_rad_s_;
                *right_target_velocity_ = homing_speed_rad_s_;
                *left_enabled_ = *right_enabled_ = true;
            }
            return;
        }
        if (manual_test_mode_) {
            constexpr double two_pi = 2.0 * std::numbers::pi;
            const auto command = manual_test_.update({
                .valid = *feedback_valid_ && *remote_valid_ && !*hardware_fault_
                      && left_joystick_->allFinite(),
                .pitch_stick = joystick(*left_joystick_).x(),
                .yaw_stick = joystick(*left_joystick_).y(),
                .angle = {left * two_pi / pitch_lead_m_, right * two_pi / pitch_lead_m_,
                          yaw_displacement * two_pi / yaw_lead_m_},
                .velocity = {*left_velocity_, *right_velocity_, *yaw_velocity_},
            });
            *left_angle_error_ = *right_angle_error_ = *yaw_angle_error_ = 0.0;
            if (remote_enabled) {
                *test_left_velocity_ = command.velocity[0];
                *test_right_velocity_ = command.velocity[1];
                *test_yaw_velocity_ = command.velocity[2];
                *left_target_velocity_ = command.velocity[0];
                *right_target_velocity_ = command.velocity[1];
                *yaw_target_velocity_ = command.velocity[2];
                *left_enabled_ = command.enabled[0];
                *right_enabled_ = command.enabled[1];
                *yaw_enabled_ = command.enabled[2];
            }
            *control_enabled_ = remote_enabled
                              && (command.enabled[0] || command.enabled[1] || command.enabled[2]);
            *debug_test_state_ = remote_enabled ? command.state : 0.0;
            if (command.state == 4 && last_test_state_ != 4)
                RCLCPP_WARN(get_logger(), "Manual test stopped by limit/interlock; center left stick "
                                          "to reset");
            if (command.state == 6 && last_test_state_ != 6)
                RCLCPP_WARN(get_logger(), "Lift skew limit reached; correcting one side at reduced speed");
            last_test_state_ = command.state;
            return;
        }
        const bool valid = geometry_ready_ && *feedback_valid_ && *remote_valid_
                        && !*hardware_fault_ && remote_enabled && std::isfinite(left)
                        && std::isfinite(right) && std::isfinite(yaw_displacement)
                        && std::isfinite(*update_rate_) && *update_rate_ > 0.0;

        if (!valid || sync_fault_ || std::abs(sync_error) > max_sync_error_m_) {
            if (std::abs(sync_error) > max_sync_error_m_)
                sync_fault_ = true;
            if (*left_switch_ == rmcs_msgs::Switch::DOWN
                && *right_switch_ == rmcs_msgs::Switch::DOWN)
                sync_fault_ = false;
            initialized_ = false;
            *control_enabled_ = false;
            *left_angle_error_ = *right_angle_error_ = *yaw_angle_error_ = 0.0;
            return;
        }

        if (!initialized_) {
            pitch_target_ = *pitch_rad_;
            yaw_target_ = *yaw_rad_;
            initialized_ = true;
        }
        const double dt = std::min(1.0 / *update_rate_, 0.02);
        // DR16 uses x for vertical stick motion and y for horizontal (positive left).
        pitch_target_ = std::clamp(
            pitch_target_ + joystick(*left_joystick_).x() * max_pitch_rate_rad_s_ * dt,
            pitch_min_rad_, pitch_max_rad_);
        yaw_target_ = std::clamp(
            yaw_target_ + joystick(*left_joystick_).y() * max_yaw_rate_rad_s_ * dt, yaw_min_rad_,
            yaw_max_rad_);

        const double height_min = pitch_height_at_start_m_ + pitch_travel_min_m_;
        const double height_max = pitch_height_at_start_m_ + pitch_travel_max_m_;
        const double pitch_lower =
            std::max(pitch_min_rad_, pitch_from_position(front_distance_m_, lateral, height_min));
        const double pitch_upper =
            std::min(pitch_max_rad_, pitch_from_position(front_distance_m_, lateral, height_max));
        if (pitch_lower > pitch_upper) {
            initialized_ = false;
            *control_enabled_ = false;
            return;
        }
        pitch_target_ = std::clamp(pitch_target_, pitch_lower, pitch_upper);

        const double yaw_target_lateral = lateral_for_yaw(front_distance_m_, yaw_target_);
        double yaw_target_displacement = std::clamp(
            yaw_target_lateral - yaw_position_at_start_m_, yaw_travel_min_m_, yaw_travel_max_m_);
        const double target_height_at_yaw = height_for_pitch(
            front_distance_m_, yaw_position_at_start_m_ + yaw_target_displacement, pitch_target_);
        const double height_error_now =
            height_for_pitch(front_distance_m_, lateral, pitch_target_) - *average_height_m_;
        if (target_height_at_yaw < height_min || target_height_at_yaw > height_max
            || std::abs(height_error_now) > max_pitch_tracking_error_m_)
            yaw_target_displacement = yaw_displacement;
        yaw_target_ = yaw_from_position(
            front_distance_m_, yaw_position_at_start_m_ + yaw_target_displacement);

        // Use measured yaw for pitch compensation. Its changing lateral offset
        // changes the pitch geometry even when the pitch stick is centered.
        const double height_target = height_for_pitch(front_distance_m_, lateral, pitch_target_);
        const double pitch_target_travel = std::clamp(
            height_target - pitch_height_at_start_m_, pitch_travel_min_m_, pitch_travel_max_m_);
        constexpr double two_pi = 2.0 * std::numbers::pi;
        auto [left_error_m, right_error_m] =
            synchronized_errors(pitch_target_travel, left, right, sync_gain_);
        const double yaw_error_m = yaw_target_displacement - yaw_displacement;
        // A correction cannot drive either lift motor beyond its own travel limit.
        if ((left >= pitch_travel_max_m_ && left_error_m > 0.0)
            || (left <= pitch_travel_min_m_ && left_error_m < 0.0))
            left_error_m = 0.0;
        if ((right >= pitch_travel_max_m_ && right_error_m > 0.0)
            || (right <= pitch_travel_min_m_ && right_error_m < 0.0))
            right_error_m = 0.0;

        *left_angle_error_ = left_error_m * two_pi / pitch_lead_m_;
        *right_angle_error_ = right_error_m * two_pi / pitch_lead_m_;
        *yaw_angle_error_ = yaw_error_m * two_pi / yaw_lead_m_;
        *pitch_target_output_ = pitch_target_;
        *yaw_target_output_ = yaw_target_;
        *control_enabled_ = true;
        *left_enabled_ = *right_enabled_ = *yaw_enabled_ = true;
    }

private:
    Eigen::Vector2d joystick(const Eigen::Vector2d& value) const {
        if (!value.allFinite())
            return Eigen::Vector2d::Zero();
        auto deadband = [this](double x) { return std::abs(x) <= joystick_deadband_ ? 0.0 : x; };
        return {deadband(value.x()), deadband(value.y())};
    }

    InputInterface<double> left_position_m_, right_position_m_, yaw_position_m_;
    InputInterface<double> left_velocity_, right_velocity_, yaw_velocity_;
    InputInterface<bool> feedback_valid_, remote_valid_, hardware_fault_;
    InputInterface<double> homing_state_;
    InputInterface<Eigen::Vector2d> left_joystick_, right_joystick_;
    InputInterface<rmcs_msgs::Switch> left_switch_, right_switch_;
    InputInterface<double> update_rate_;
    OutputInterface<bool> control_enabled_;
    OutputInterface<bool> left_enabled_, right_enabled_, yaw_enabled_;
    OutputInterface<double> left_target_velocity_, right_target_velocity_, yaw_target_velocity_;
    OutputInterface<double> test_left_velocity_, test_right_velocity_, test_yaw_velocity_;
    OutputInterface<double> debug_left_x_, debug_left_y_, debug_right_x_, debug_right_y_;
    OutputInterface<double> debug_left_switch_, debug_right_switch_;
    OutputInterface<double> debug_remote_valid_, debug_feedback_valid_, debug_hardware_fault_;
    OutputInterface<double> debug_test_state_;
    ManualTest manual_test_;
    bool manual_test_mode_ = false;
    int last_test_state_ = 0;
    OutputInterface<double> left_angle_error_, right_angle_error_, yaw_angle_error_;
    OutputInterface<double> average_height_m_, sync_error_m_, pitch_rad_, yaw_rad_;
    OutputInterface<double> pitch_target_output_, yaw_target_output_;
    bool geometry_ready_ = false, initialized_ = false, sync_fault_ = false;
    double front_distance_m_ = 0.0, pitch_height_at_start_m_ = 0.0;
    double yaw_position_at_start_m_ = 0.0;
    double pitch_lead_m_ = 0.0002, yaw_lead_m_ = 0.0002;
    double pitch_min_rad_ = 0.0, pitch_max_rad_ = 0.0;
    double yaw_min_rad_ = 0.0, yaw_max_rad_ = 0.0;
    double pitch_travel_min_m_ = 0.0, pitch_travel_max_m_ = 0.0;
    double yaw_travel_min_m_ = 0.0, yaw_travel_max_m_ = 0.0;
    double max_pitch_rate_rad_s_ = 0.1, max_yaw_rate_rad_s_ = 0.1;
    double joystick_deadband_ = 0.05, sync_gain_ = 1.0, max_sync_error_m_ = 0.005;
    double max_pitch_tracking_error_m_ = 0.003;
    double homing_speed_rad_s_ = 5.0;
    double pitch_target_ = 0.0, yaw_target_ = 0.0;
};
} // namespace rmcs_core::controller::gantry

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gantry::GantryController, rmcs_executor::Component)
