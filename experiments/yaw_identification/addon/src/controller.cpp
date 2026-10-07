#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "yaw_identification/engine.hpp"

namespace yaw_identification {
class Controller : public rmcs_executor::Component, public rclcpp::Node {
    using Clock = std::chrono::steady_clock;
    using Trigger = std_srvs::srv::Trigger;
public:
    Controller() : Node(get_component_name(),
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        if (!has_parameter("allow_arm")) declare_parameter("allow_arm", true);
        allow_arm_ = get_parameter("allow_arm").as_bool();
        auto& c = engine_.config;
        if (!has_parameter("world_pitch_control")) declare_parameter("world_pitch_control", false);
        c.world_pitch_control = get_parameter("world_pitch_control").as_bool();
        if (!has_parameter("pitch_limits_relative_to_arm"))
            declare_parameter("pitch_limits_relative_to_arm", c.world_pitch_control);
        c.pitch_limits_relative_to_arm = get_parameter("pitch_limits_relative_to_arm").as_bool();
        if (!has_parameter("tracking_test")) declare_parameter("tracking_test", false);
        c.tracking_test = get_parameter("tracking_test").as_bool();
        if (!has_parameter("rich_identification")) declare_parameter("rich_identification", false);
        c.rich_identification = get_parameter("rich_identification").as_bool();
        c.world_pitch_target = parameter("world_pitch_target_deg", 5.0) * std::numbers::pi / 180.0;
        c.pitch_target = parameter("pitch_target_deg", -10.0) * std::numbers::pi / 180.0;
        c.pitch_ready_tolerance = parameter("pitch_ready_tolerance_deg", 1.5)
                                  * std::numbers::pi / 180.0;
        c.min_world_pitch = parameter("min_world_pitch_deg", 0.0)
                            * std::numbers::pi / 180.0;
        c.pitch_min = parameter("pitch_min", c.pitch_min);
        c.pitch_max = parameter("pitch_max", c.pitch_max);
        c.pitch_ramp = parameter("pitch_ramp_rad_s", c.pitch_ramp);
        c.pitch_torque_limit = parameter("pitch_torque_limit", c.pitch_torque_limit);
        c.pitch_target_lead = parameter("pitch_target_lead_limit_deg", 0.8)
                              * std::numbers::pi / 180.0;
        c.pitch_target_soft_tau = parameter("pitch_target_soft_tau_s", 0.0);
        c.pitch_stiction_compensation = parameter("pitch_stiction_compensation_Nm", 0.0);
        c.pitch_precharge_s = parameter("pitch_precharge_s", 0.0);
        c.pitch_precharge_torque_slew = parameter("pitch_precharge_torque_slew_Nm_s", 8.0);
        c.pitch_start_ramp_s = parameter("pitch_start_ramp_s", 0.0);
        c.torque_slew = parameter("pitch_torque_slew_Nm_s", c.torque_slew);
        c.torque_release_slew = parameter("pitch_torque_release_slew_Nm_s",
                                          c.torque_release_slew);
        c.off_slew = parameter("pitch_off_slew_Nm_s", c.off_slew);
        c.yaw_torque_limit = parameter("yaw_torque_limit", c.yaw_torque_limit);
        c.yaw_span = parameter("yaw_span_rad", c.yaw_span);
        c.pitch_velocity_filter_tau = parameter("pitch_velocity_filter_tau_s",
                                                c.pitch_velocity_filter_tau);
        c.yaw_velocity_filter_tau = parameter("yaw_velocity_filter_tau_s",
                                              c.yaw_velocity_filter_tau);
        c.gravity_gain = parameter("pitch_gravity_ff_gain", c.gravity_gain);
        c.gravity_phase = parameter("pitch_gravity_ff_phase", c.gravity_phase);
        if (has_parameter("pitch_gravity_ff_raise_gain")) {
            c.staged_gravity_gain = true;
            c.gravity_raise_gain = get_parameter("pitch_gravity_ff_raise_gain").as_double();
        }
        c.amplitude = parameter("sweep_amplitude", c.amplitude);
        c.start_hz = parameter("start_freq", c.start_hz);
        c.end_hz = parameter("end_freq", c.end_hz);
        c.duration = parameter("duration", c.duration);
        c.tracking_amplitude = parameter("tracking_amplitude_deg", 5.0)
                               * std::numbers::pi / 180.0;
        c.tracking_frequency = parameter("tracking_frequency_hz", c.tracking_frequency);
        if (has_parameter("tracking_frequencies_hz")) {
            const auto values = get_parameter("tracking_frequencies_hz").as_double_array();
            c.tracking_frequencies.assign(values.begin(), values.end());
        }
        c.tracking_ramp = parameter("tracking_ramp_s", c.tracking_ramp);
        c.yaw_ff_velocity = parameter("yaw_velocity_ff_torque_gain", 0.0);
        c.yaw_ff_acceleration = parameter("yaw_acceleration_ff_torque_gain", 0.0);
        c.yaw_ff_bias = parameter("yaw_bias_ff_torque", 0.0);
        c.yaw_torque_slew = parameter("yaw_torque_slew_Nm_s", c.yaw_torque_slew);
        c.yaw_torque_release_slew = parameter("yaw_torque_release_slew_Nm_s",
                                               c.yaw_torque_slew);
        configure("pitch_angle", engine_.pitch_angle);
        configure("pitch_velocity", engine_.pitch_velocity);
        configure("yaw_angle", engine_.yaw_angle);
        configure("yaw_velocity", engine_.yaw_velocity);
        engine_.validate();
        register_input("/gimbal/pitch/angle", pitch_);
        register_input("/tf", tf_);
        register_input("/gimbal/yaw/angle", yaw_);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_);
        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_);
        register_input("/gimbal/pitch/temperature", pitch_temperature_);
        register_input("/gimbal/pitch/torque", pitch_feedback_torque_);
        register_input("/gimbal/pitch/velocity", pitch_motor_velocity_);
        register_input("/gimbal/yaw/temperature", yaw_temperature_);
        register_input("/predefined/update_rate", rate_);
        register_output("/gimbal/pitch/control_torque", pitch_torque_, 0.0);
        register_output("/gimbal/yaw/control_torque", yaw_torque_, 0.0);
        // The C-car hardware includes a supercap command even in a yaw-only test.
        // No chassis power controller is loaded; request zero charging power.
        register_output("/chassis/supercap/charge_power_limit", charge_power_limit_, 0.0);
        register_output("/yaw_experiment/state", state_, 0.0);
        register_output("/yaw_experiment/fault", fault_, 0.0);
        register_output("/yaw_experiment/completed", completed_, 0.0);
        register_output("/yaw_experiment/time_s", time_, 0.0);
        register_output("/yaw_experiment/excitation", excitation_, 0.0);
        register_output("/yaw_experiment/pitch_target", pitch_target_, 0.0);
        register_output("/yaw_experiment/pitch_torque_unlimited", pitch_torque_unlimited_, 0.0);
        register_output("/yaw_experiment/pitch_world", pitch_world_output_, 0.0);
        register_output("/yaw_experiment/yaw_target_offset", yaw_target_offset_, 0.0);
        register_output("/yaw_experiment/yaw_error", yaw_error_output_, 0.0);
        register_output("/yaw_experiment/yaw_feedforward", yaw_feedforward_, 0.0);
        register_output("/yaw_experiment/tracking_frequency_hz", tracking_frequency_, 0.0);
        register_output("/yaw_experiment/excitation_profile", excitation_profile_, 0.0);
        heartbeat_ = create_subscription<std_msgs::msg::Empty>(
            "/yaw_experiment/heartbeat", 1, [this](const std_msgs::msg::Empty&) {
                last_heartbeat_.store(ticks(), std::memory_order_relaxed);
            });
        status_ = create_publisher<std_msgs::msg::String>("/yaw_experiment/status", 1);
        add_service("arm", Command::Arm);
        add_service("sweep", Command::Sweep);
        add_service("hold", Command::Hold);
        add_service("off", Command::Off);
        RCLCPP_WARN(get_logger(), "Experiment OFF. Pitch target %.1f deg (%s), torque limit %.2f Nm. Use console.py or session.py.",
                    (c.world_pitch_control ? c.world_pitch_target : c.pitch_target) * 180 / std::numbers::pi,
                    c.world_pitch_control ? "world" : "encoder", c.pitch_torque_limit);
    }

    void update() override {
        const auto now = Clock::now();
        const double dt = last_update_ == Clock::time_point{} ? 1.0 / *rate_
            : std::chrono::duration<double>(now - last_update_).count();
        last_update_ = now;
        if (start_ == Clock::time_point{}) start_ = now;
        const auto heartbeat_age = ticks() - last_heartbeat_.load(std::memory_order_relaxed);
        const auto pitch_direction = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        const double pitch_world = std::asin(std::clamp(pitch_direction->z(), -1.0, 1.0));
        Input in{*yaw_, *pitch_, pitch_world, *yaw_velocity_, *pitch_velocity_,
                 *yaw_temperature_, *pitch_temperature_, heartbeat_age >= 0 && heartbeat_age < 1500000000LL};
        in.pitch = std::remainder(in.pitch, 2 * std::numbers::pi);
        auto cmd = pending_.exchange(Command::None);
        if (off_requested_.exchange(false)) cmd = Command::Off;
        // Temperature outputs initialize at zero; do not arm before both motors have replied.
        if (cmd == Command::Arm && (in.yaw_temperature <= 0 || in.pitch_temperature <= 0)) {
            RCLCPP_WARN(get_logger(), "Arm refused: waiting for nonzero motor temperature feedback");
            cmd = Command::None;
        }
        const auto out = engine_.update(in, cmd, dt);
        if (cmd == Command::Arm && out.state == State::Off)
            RCLCPP_WARN(get_logger(), "Arm ignored: heartbeat=%d finite=%d pitch_temp=%.1f yaw_temp=%.1f",
                        in.heartbeat, std::isfinite(in.pitch_world),
                        in.pitch_temperature, in.yaw_temperature);
        if (cmd == Command::Sweep && out.state != State::Sweep)
            RCLCPP_WARN(get_logger(), "Sweep did not start: state=%d fault=%d world_pitch=%.1f deg (minimum %.1f deg). Check pitch stability and clearance.",
                        static_cast<int>(out.state), out.fault,
                        in.pitch_world * 180 / std::numbers::pi,
                        engine_.config.min_world_pitch * 180 / std::numbers::pi);
        state_snapshot_.store(out.state);
        *yaw_torque_ = out.yaw_torque;
        *pitch_torque_ = out.pitch_torque;
        *state_ = static_cast<double>(out.state);
        *fault_ = out.fault;
        *completed_ = out.completed ? 1.0 : 0.0;
        *time_ = std::chrono::duration<double>(now - start_).count();
        *excitation_ = out.excitation;
        *pitch_target_ = out.pitch_target;
        *pitch_torque_unlimited_ = out.pitch_torque_unlimited;
        *pitch_world_output_ = in.pitch_world;
        *yaw_target_offset_ = out.yaw_target_offset;
        *yaw_error_output_ = out.yaw_error;
        *yaw_feedforward_ = out.yaw_feedforward;
        *tracking_frequency_ = out.tracking_frequency;
        *excitation_profile_ = out.excitation_profile;
        if (out.state != previous_state_) {
            RCLCPP_WARN(get_logger(), "Experiment state=%d fault=%d pitch=%.2f deg",
                        static_cast<int>(out.state), out.fault, in.pitch * 180 / std::numbers::pi);
            previous_state_ = out.state;
        }
        if (ticks() - last_status_ > 500000000LL) {
            std_msgs::msg::String msg;
            msg.data = "state=" + std::to_string(static_cast<int>(out.state))
                + " fault=" + std::to_string(out.fault)
                + " pitch_deg=" + std::to_string(in.pitch * 180 / std::numbers::pi)
                + " pitch_cmd_Nm=" + std::to_string(out.pitch_torque)
                + " pitch_fb_Nm=" + std::to_string(*pitch_feedback_torque_)
                + " pitch_raw_Nm=" + std::to_string(out.pitch_torque_unlimited)
                + " pitch_world_deg=" + std::to_string(in.pitch_world * 180 / std::numbers::pi)
                + " motor_rad_s=" + std::to_string(*pitch_motor_velocity_)
                + " yaw_deg=" + std::to_string(in.yaw * 180 / std::numbers::pi)
                + " yaw_cmd_Nm=" + std::to_string(out.yaw_torque)
                + " yaw_target_deg=" + std::to_string(out.yaw_target_offset * 180 / std::numbers::pi)
                + " yaw_error_deg=" + std::to_string(out.yaw_error * 180 / std::numbers::pi)
                + " tracking_hz=" + std::to_string(out.tracking_frequency)
                + " profile=" + std::to_string(out.excitation_profile)
                + " pitch_temp=" + std::to_string(in.pitch_temperature)
                + " yaw_temp=" + std::to_string(in.yaw_temperature)
                + " heartbeat_age_s=" + std::to_string(heartbeat_age / 1000000000.0)
                + " sweep_s=" + std::to_string(out.elapsed);
            msg.data += " completed=" + std::to_string(out.completed ? 1 : 0);
            status_->publish(msg);
            last_status_ = ticks();
        }
    }
private:
    static int64_t ticks() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count();
    }
    double parameter(const std::string& name, double value) {
        if (!has_parameter(name)) declare_parameter(name, value);
        return get_parameter(name).as_double();
    }
    void configure(const std::string& name, Pid& pid) {
        pid.kp = parameter(name + "_kp", pid.kp);
        pid.ki = parameter(name + "_ki", pid.ki);
        pid.kd = parameter(name + "_kd", pid.kd);
        pid.integral_min = parameter(name + "_integral_min", -0.5);
        pid.integral_max = parameter(name + "_integral_max", 0.5);
        pid.output_min = parameter(name + "_output_min", -0.5);
        pid.output_max = parameter(name + "_output_max", 0.5);
        if (!std::isfinite(pid.kp) || !std::isfinite(pid.ki) || !std::isfinite(pid.kd)
            || !std::isfinite(pid.integral_min) || !std::isfinite(pid.integral_max)
            || !std::isfinite(pid.output_min) || !std::isfinite(pid.output_max)
            || pid.integral_min > pid.integral_max || pid.output_min >= pid.output_max)
            throw std::runtime_error("Invalid PID parameters for " + name);
    }
    void add_service(const std::string& name, Command cmd) {
        services_.push_back(create_service<Trigger>("/yaw_experiment/" + name,
            [this, cmd](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                auto state = state_snapshot_.load();
                if (cmd == Command::Off) {
                    off_requested_.store(true);
                    response->success = true;
                } else if (cmd == Command::Arm &&
                           (ticks() - last_heartbeat_.load(std::memory_order_relaxed) >= 1500000000LL)) {
                    response->message = "No recent heartbeat; wait for heartbeat_age_s < 1.5";
                    return;
                } else if ((cmd == Command::Arm && state == State::Off && allow_arm_)
                           || (cmd == Command::Sweep && state == State::Ready)
                           || (cmd == Command::Hold && state != State::Off && state != State::Fault)) {
                    Command expected = Command::None;
                    response->success = pending_.compare_exchange_strong(expected, cmd);
                }
                response->message = response->success ? "Queued; watch status for resulting state"
                                                     : "Rejected: state not ready or command pending";
            }));
    }
    Engine engine_;
    bool allow_arm_ = true;
    InputInterface<double> pitch_, yaw_, pitch_velocity_, yaw_velocity_, pitch_temperature_, yaw_temperature_, rate_;
    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<double> pitch_feedback_torque_, pitch_motor_velocity_;
    OutputInterface<double> pitch_torque_, yaw_torque_, state_, fault_, completed_, time_, excitation_, pitch_target_, pitch_torque_unlimited_, pitch_world_output_;
    OutputInterface<double> yaw_target_offset_, yaw_error_output_, yaw_feedforward_;
    OutputInterface<double> tracking_frequency_;
    OutputInterface<double> excitation_profile_;
    OutputInterface<double> charge_power_limit_;
    std::atomic<int64_t> last_heartbeat_{0};
    std::atomic<Command> pending_{Command::None};
    std::atomic<bool> off_requested_{false};
    std::atomic<State> state_snapshot_{State::Off};
    State previous_state_ = State::Off;
    Clock::time_point last_update_{}, start_{};
    int64_t last_status_ = 0;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
};
}
PLUGINLIB_EXPORT_CLASS(yaw_identification::Controller, rmcs_executor::Component)
