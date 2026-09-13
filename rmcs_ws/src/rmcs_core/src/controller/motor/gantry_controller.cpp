#include <cmath>
#include <algorithm>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gantry {

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        : Node{
            get_component_name(),
            rclcpp::NodeOptions{}
                .automatically_declare_parameters_from_overrides(true)} {

        target_position_ = get_parameter("target_position").as_double();
        max_velocity_ = get_parameter("max_velocity").as_double();
        sync_gain_ = get_parameter("sync_gain").as_double();
        max_sync_error_ = get_parameter("max_sync_error").as_double();
        position_min_ = get_parameter("position_min").as_double();
        position_max_ = get_parameter("position_max").as_double();

        
        register_input("/gantry/left_position_m", left_position_m_, 0.0);
        register_input("/gantry/right_position_m", right_position_m_, 0.0);

       
        register_output("/gantry/cmd_vel_left", cmd_vel_left_, 0.0);
        register_output("/gantry/cmd_vel_right", cmd_vel_right_, 0.0);

        
        register_output("/gantry/average_position", average_position_, 0.0);
        register_output("/gantry/sync_error", sync_error_, 0.0);
    }

    void update() override {
        
        double left_pos = *left_position_m_;
        double right_pos = *right_position_m_;
        double avg_pos = (left_pos + right_pos) / 2.0;
        double pos_diff = left_pos - right_pos;

        *average_position_ = avg_pos;
        *sync_error_ = pos_diff;

        
        double position_err = target_position_ - avg_pos;
        double base_velocity = std::clamp(position_err * 2.0, -max_velocity_, max_velocity_);

        
        if (avg_pos <= position_min_ && base_velocity < 0.0) {
            base_velocity = 0.0;
        }
        if (avg_pos >= position_max_ && base_velocity > 0.0) {
            base_velocity = 0.0;
        }

        
        double clamped_diff = std::clamp(pos_diff, -max_sync_error_, max_sync_error_);
        double sync_correction = sync_gain_ * clamped_diff;

        
        *cmd_vel_left_ = base_velocity + sync_correction;
        *cmd_vel_right_ = base_velocity - sync_correction;
    }

private:
    double target_position_ = 0.0;
    double max_velocity_ = 0.0;
    double sync_gain_ = 0.0;
    double max_sync_error_ = 0.0;
    double position_min_ = 0.0;
    double position_max_ = 0.0;

    InputInterface<double> left_position_m_;
    InputInterface<double> right_position_m_;

    OutputInterface<double> cmd_vel_left_;
    OutputInterface<double> cmd_vel_right_;
    OutputInterface<double> average_position_;
    OutputInterface<double> sync_error_;
};

} // namespace rmcs_core::controller::gantry

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gantry::GantryController,
    rmcs_executor::Component)
