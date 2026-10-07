#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <stdexcept>

#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/gantry_homing.hpp"
#include "hardware/gantry_switches.hpp"

namespace rmcs_core::hardware {
class Gantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
    using Clock = std::chrono::steady_clock;

public:
    Gantry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<CommandTransmitter>(
                  get_component_name() + "_command", *this))
        , left_motor_(*this, *command_component_, "/gantry/left_motor")
        , right_motor_(*this, *command_component_, "/gantry/right_motor")
        , yaw_motor_(*this, *command_component_, "/gantry/yaw_motor")
        , remote_control_(std::make_unique<device::RemoteControl>(*this)) {
        pitch_lead_m_ = get_parameter("pitch_lead_m_per_rev").as_double();
        yaw_lead_m_ = get_parameter("yaw_lead_m_per_rev").as_double();
        feedback_timeout_s_ = get_parameter("feedback_timeout_s").as_double();
        remote_timeout_s_ = get_parameter("remote_timeout_s").as_double();
        stall_torque_nm_ = get_parameter("stall_torque_nm").as_double();
        stall_speed_rad_s_ = get_parameter("stall_speed_rad_s").as_double();
        stall_target_speed_rad_s_ = get_parameter("stall_target_speed_rad_s").as_double();
        stall_duration_s_ = get_parameter("stall_duration_s").as_double();
        temperature_limit_c_ = get_parameter("temperature_limit_c").as_double();
        get_parameter_or("auto_home", auto_home_, false);
        if (auto_home_) {
            homing_torque_limit_nm_ = get_parameter("homing_torque_limit_nm").as_double();
            const double contact_torque = get_parameter("homing_contact_torque_nm").as_double();
            const double contact_speed = get_parameter("homing_contact_speed_rad_s").as_double();
            const double contact_hold = get_parameter("homing_contact_hold_s").as_double();
            const double timeout = get_parameter("homing_timeout_s").as_double();
            const double max_travel_m = get_parameter("homing_max_travel_m").as_double();
            if (!std::isfinite(homing_torque_limit_nm_) || homing_torque_limit_nm_ <= 0.0
                || !std::isfinite(contact_torque) || contact_torque <= 0.0
                || contact_torque >= homing_torque_limit_nm_ || !std::isfinite(contact_speed)
                || contact_speed < 0.0 || !std::isfinite(contact_hold) || contact_hold <= 0.0
                || !std::isfinite(timeout) || timeout <= contact_hold
                || !std::isfinite(max_travel_m) || max_travel_m <= 0.0)
                throw std::invalid_argument("Invalid gantry homing parameter");
            homing_ = std::make_unique<GantryHoming>(GantryHoming::Config{
                .contact_torque_nm = contact_torque,
                .contact_speed_rad_s = contact_speed,
                .contact_hold_s = contact_hold,
                .timeout_s = timeout,
                .max_travel_rad = max_travel_m * 2.0 * std::numbers::pi / pitch_lead_m_,
            });
            RCLCPP_WARN(
                get_logger(), "Automatic upper-stop homing enabled: lift motors will rise "
                              "when remote and CAN feedback become valid; joystick ignored");
        }
        if (!std::isfinite(pitch_lead_m_) || pitch_lead_m_ <= 0.0 || !std::isfinite(yaw_lead_m_)
            || yaw_lead_m_ <= 0.0 || !std::isfinite(feedback_timeout_s_)
            || feedback_timeout_s_ <= 0.0 || !std::isfinite(remote_timeout_s_)
            || remote_timeout_s_ <= 0.0 || !std::isfinite(stall_torque_nm_)
            || stall_torque_nm_ <= 0.0 || !std::isfinite(stall_speed_rad_s_)
            || stall_speed_rad_s_ < 0.0 || !std::isfinite(stall_target_speed_rad_s_)
            || stall_target_speed_rad_s_ <= 0.0 || !std::isfinite(stall_duration_s_)
            || stall_duration_s_ <= 0.0 || !std::isfinite(temperature_limit_c_)
            || temperature_limit_c_ <= 0.0)
            throw std::invalid_argument("Invalid gantry transmission or protection parameter");

        // Left/right are viewed facing the launch direction.
        auto left = device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 2};
        auto right = device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3};
        auto yaw = device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 1};
        left.enable_multi_turn_angle();
        right.enable_multi_turn_angle();
        yaw.enable_multi_turn_angle();
        if (get_parameter("left_reversed").as_bool())
            left.set_reversed();
        if (get_parameter("right_reversed").as_bool())
            right.set_reversed();
        if (get_parameter("yaw_reversed").as_bool())
            yaw.set_reversed();
        left_motor_.configure(left);
        right_motor_.configure(right);
        yaw_motor_.configure(yaw);

        remote_control_->register_dr16(&dr16_);
        register_output("/gantry/left_position_m", left_position_m_, 0.0);
        register_output("/gantry/right_position_m", right_position_m_, 0.0);
        register_output("/gantry/yaw_position_m", yaw_position_m_, 0.0);
        register_output("/gantry/feedback_valid", feedback_valid_, false);
        register_output("/gantry/remote_valid", remote_valid_, false);
        register_output("/gantry/fault", fault_output_, false);
        register_output("/gantry/homing_state", homing_state_output_, auto_home_ ? 0.0 : 2.0);
        register_output("/gantry/debug/home_left_reached", home_left_reached_output_, 0.0);
        register_output("/gantry/debug/home_right_reached", home_right_reached_output_, 0.0);
        register_output("/gantry/debug/left_stall", left_stall_output_, 0.0);
        register_output("/gantry/debug/right_stall", right_stall_output_, 0.0);
        register_output("/gantry/debug/yaw_stall", yaw_stall_output_, 0.0);
        register_output("/gantry/debug/dbus_rx_count", dbus_rx_count_, 0.0);
        register_output("/gantry/debug/uart1_rx_count", uart1_rx_count_, 0.0);
        register_output("/gantry/debug/uart2_rx_count", uart2_rx_count_, 0.0);
        register_output("/gantry/debug/dbus_bad_size_count", dbus_bad_size_count_, 0.0);
        register_output("/gantry/debug/dbus_last_size", dbus_last_size_, 0.0);
        register_output("/gantry/debug/remote_age_s", remote_age_s_, -1.0);

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
    }

    void update() override {
        const std::array<device::DjiMotor*, 3> motors{&left_motor_, &right_motor_, &yaw_motor_};
        const auto now = now_ns();
        const auto timeout = static_cast<std::int64_t>(feedback_timeout_s_ * 1e9);
        bool feedback_fresh = true;
        for (std::size_t i = 0; i < motors.size(); ++i) {
            const auto received = feedback_at_ns_[i].load(std::memory_order_relaxed);
            const bool fresh = received != 0 && now - received <= timeout;
            feedback_fresh &= fresh;
            if (received == 0)
                continue;
            motors[i]->update_status();
            if (!origin_initialized_[i]) {
                angle_origin_[i] = motors[i]->angle();
                origin_initialized_[i] = true;
            }
        }

        dr16_.set_timeout_enabled(true);
        dr16_.update_status();
        remote_control_->update();
        // RemoteControl disables this timeout for a single-DR16 setup.
        dr16_.set_timeout_enabled(true);

        constexpr double two_pi = 2.0 * std::numbers::pi;
        *left_position_m_ = (left_motor_.angle() - angle_origin_[0]) * pitch_lead_m_ / two_pi;
        *right_position_m_ = (right_motor_.angle() - angle_origin_[1]) * pitch_lead_m_ / two_pi;
        *yaw_position_m_ = (yaw_motor_.angle() - angle_origin_[2]) * yaw_lead_m_ / two_pi;
        *feedback_valid_ = feedback_fresh;
        const auto remote_stamp = remote_at_ns_.load(std::memory_order_relaxed);
        *remote_valid_ = dr16_.valid() && remote_stamp != 0
                      && now - remote_stamp <= static_cast<std::int64_t>(remote_timeout_s_ * 1e9);
        *dbus_rx_count_ = uart_rx_counts_[0].load(std::memory_order_relaxed);
        *uart1_rx_count_ = uart_rx_counts_[1].load(std::memory_order_relaxed);
        *uart2_rx_count_ = uart_rx_counts_[2].load(std::memory_order_relaxed);
        *dbus_bad_size_count_ = dbus_bad_sizes_.load(std::memory_order_relaxed);
        *dbus_last_size_ = dbus_last_size_bytes_.load(std::memory_order_relaxed);
        *remote_age_s_ = remote_stamp == 0 ? -1.0 : (now - remote_stamp) / 1e9;
        if (!*remote_valid_)
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Remote invalid: DBUS packets=%.0f, bad lengths=%.0f, last bytes=%.0f, "
                "UART1 packets=%.0f, UART2 packets=%.0f, last 18-byte frame age=%.3f s "
                "(-1 means none)",
                *dbus_rx_count_, *dbus_bad_size_count_, *dbus_last_size_,
                *uart1_rx_count_, *uart2_rx_count_, *remote_age_s_);

        if (feedback_fresh
            && (left_motor_.temperature() >= temperature_limit_c_
                || right_motor_.temperature() >= temperature_limit_c_
                || yaw_motor_.temperature() >= temperature_limit_c_))
            fault_latched_.store(true, std::memory_order_relaxed);
        if (*remote_valid_ && dr16_.switch_left() == rmcs_msgs::Switch::DOWN
            && dr16_.switch_right() == rmcs_msgs::Switch::DOWN
            && dr16_.joystick_left().norm() <= 0.05
            && feedback_fresh && left_motor_.temperature() < temperature_limit_c_
            && right_motor_.temperature() < temperature_limit_c_
            && yaw_motor_.temperature() < temperature_limit_c_)
            fault_latched_.store(false, std::memory_order_relaxed);
        *fault_output_ = fault_latched_.load(std::memory_order_relaxed);
        *homing_state_output_ = homing_
            ? static_cast<double>(homing_->state()) : static_cast<double>(GantryHoming::State::Complete);
        const auto reached = homing_ ? homing_->reached() : std::array<bool, 2>{false, false};
        *home_left_reached_output_ = reached[0];
        *home_right_reached_output_ = reached[1];
        *left_stall_output_ = stall_latched_[0].load(std::memory_order_relaxed);
        *right_stall_output_ = stall_latched_[1].load(std::memory_order_relaxed);
        *yaw_stall_output_ = stall_latched_[2].load(std::memory_order_relaxed);
    }

private:
    class CommandTransmitter : public rmcs_executor::Component {
    public:
        explicit CommandTransmitter(Gantry& gantry)
            : gantry_(gantry) {
            register_input("/gantry/left_motor/target_velocity", left_target_);
            register_input("/gantry/right_motor/target_velocity", right_target_);
            register_input("/gantry/yaw_motor/target_velocity", yaw_target_);
        }
        void update() override {
            gantry_.command_update(*left_target_, *right_target_, *yaw_target_);
        }

    private:
        Gantry& gantry_;
        InputInterface<double> left_target_, right_target_, yaw_target_;
    };

    static std::int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
            .count();
    }

    bool stalled(
        std::size_t index, const device::DjiMotor& motor, double target_velocity,
        Clock::time_point now) {
        const bool pushing = std::isfinite(target_velocity)
                          && std::abs(target_velocity) >= stall_target_speed_rad_s_
                          && std::abs(motor.control_torque()) >= stall_torque_nm_
                          && std::abs(motor.velocity()) <= stall_speed_rad_s_;
        if (!pushing) {
            stalled_since_[index] = {};
            return false;
        }
        if (stalled_since_[index] == Clock::time_point{})
            stalled_since_[index] = now;
        return std::chrono::duration<double>(now - stalled_since_[index]).count()
            >= stall_duration_s_;
    }

    void command_update(double left_target, double right_target, double yaw_target) {
        if (homing_ && homing_->state() != GantryHoming::State::Complete) {
            const auto previous = homing_->state();
            const auto previously_reached = homing_->reached();
            const bool valid = *feedback_valid_ && *remote_valid_
                            && !fault_latched_.load(std::memory_order_relaxed);
            const auto cap = [this](double torque) {
                return std::isfinite(torque)
                    ? std::clamp(torque, 0.0, homing_torque_limit_nm_) : 0.0;
            };
            const double left_torque = cap(left_motor_.control_torque());
            const double right_torque = cap(right_motor_.control_torque());
            const auto now = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
            const auto result = homing_->update({
                .valid = valid,
                .time_s = now,
                .angle_rad = {left_motor_.angle(), right_motor_.angle()},
                .speed_rad_s = {left_motor_.velocity(), right_motor_.velocity()},
                .torque_nm = {left_motor_.torque(), right_motor_.torque()},
                .commanded_torque_nm = {left_torque, right_torque},
            });
            if (result.reached[0] && !previously_reached[0])
                RCLCPP_INFO(get_logger(), "Left lift ID 2 reached upper stop; motor stopped");
            if (result.reached[1] && !previously_reached[1])
                RCLCPP_INFO(get_logger(), "Right lift ID 3 reached upper stop; motor stopped");
            if (result.state == GantryHoming::State::Seeking)
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Homing upward: left/right travel %.1f/%.1f mm, "
                    "speed %.2f/%.2f rad/s, reached %d/%d",
                    *left_position_m_ * 1000.0, *right_position_m_ * 1000.0,
                    left_motor_.velocity(), right_motor_.velocity(),
                    result.reached[0], result.reached[1]);
            if (result.state == GantryHoming::State::Complete
                && previous != GantryHoming::State::Complete) {
                angle_origin_[0] = left_motor_.angle();
                angle_origin_[1] = right_motor_.angle();
                origin_initialized_[0] = origin_initialized_[1] = true;
                RCLCPP_INFO(get_logger(), "Both lift upper stops found; lift positions zeroed");
            } else if (result.state == GantryHoming::State::Failed
                       && previous != GantryHoming::State::Failed) {
                RCLCPP_ERROR(
                    get_logger(), "Gantry homing failed; lift and yaw locked. "
                                  "Feedback/remote valid=%d, lift travel=%.4f/%.4f m",
                    valid, *left_position_m_, *right_position_m_);
            }
            transmit_commands(
                device::CanPacket8::Quarter{0},
                valid && result.drive_up[0]
                    ? left_motor_.generate_command(left_torque) : device::CanPacket8::Quarter{0},
                valid && result.drive_up[1]
                    ? right_motor_.generate_command(right_torque) : device::CanPacket8::Quarter{0});
            return;
        }
        // Re-arm only the released axis. The controller may command zero while a
        // stick is still held (for example during skew recovery), so use the raw
        // stick positions rather than target velocity for this decision.
        if (*remote_valid_ && *feedback_valid_) {
            const auto stick = dr16_.joystick_left();
            if (std::isfinite(stick.x()) && std::abs(stick.x()) <= 0.05) {
                stall_latched_[0].store(false, std::memory_order_relaxed);
                stall_latched_[1].store(false, std::memory_order_relaxed);
                stalled_since_[0] = {};
                stalled_since_[1] = {};
            }
            if (std::isfinite(stick.y()) && std::abs(stick.y()) <= 0.05) {
                stall_latched_[2].store(false, std::memory_order_relaxed);
                stalled_since_[2] = {};
            }
        }
        const auto left_switch = dr16_.switch_left();
        const auto right_switch = dr16_.switch_right();
        const bool armed = *feedback_valid_ && *remote_valid_
                        && gantry_switches_enable_motion(left_switch, right_switch);
        if (armed && !fault_latched_.load(std::memory_order_relaxed)) {
            const auto now = Clock::now();
            const std::array<device::DjiMotor*, 3> motors{
                &left_motor_, &right_motor_, &yaw_motor_};
            const std::array<double, 3> targets{left_target, right_target, yaw_target};
            for (std::size_t i = 0; i < motors.size(); ++i) {
                if (!stall_latched_[i].load(std::memory_order_relaxed)
                    && stalled(i, *motors[i], targets[i], now)) {
                    stall_latched_[i].store(true, std::memory_order_relaxed);
                    RCLCPP_ERROR(
                        get_logger(), "Gantry motor ID %u stalled; command zeroed. "
                                      "Release its stick axis to re-arm",
                        motors[i]->id());
                }
            }
        } else {
            stalled_since_.fill({});
        }

        const bool drive = armed && !fault_latched_.load(std::memory_order_relaxed);
        transmit_commands(
            drive && !stall_latched_[2].load(std::memory_order_relaxed)
                ? yaw_motor_.generate_command() : device::CanPacket8::Quarter{0},
            drive && !stall_latched_[0].load(std::memory_order_relaxed)
                ? left_motor_.generate_command() : device::CanPacket8::Quarter{0},
            drive && !stall_latched_[1].load(std::memory_order_relaxed)
                ? right_motor_.generate_command() : device::CanPacket8::Quarter{0});
    }

    void transmit_commands(
        device::CanPacket8::Quarter yaw, device::CanPacket8::Quarter left,
        device::CanPacket8::Quarter right) {
        auto builder = board_->start_transmit();
        builder.can_transmit(
            Spec::kCans.kCan1,
            {.can_id = left_motor_.send_id(),
             .can_data = device::CanPacket8{
                 // Command slots follow CAN motor IDs: 1=yaw, 2=left, 3=right.
                 yaw, left, right, device::CanPacket8::PaddingQuarter{},
             }.as_bytes()});
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != Spec::kCans.kCan1 || data.is_extended_can_id || data.is_remote_transmission
            || data.can_data.size() != 8)
            return;
        const auto stamp = now_ns();
        if (left_motor_.match_then_store_status(data.can_id, data.can_data))
            feedback_at_ns_[0].store(stamp, std::memory_order_relaxed);
        else if (right_motor_.match_then_store_status(data.can_id, data.can_data))
            feedback_at_ns_[1].store(stamp, std::memory_order_relaxed);
        else if (yaw_motor_.match_then_store_status(data.can_id, data.can_data))
            feedback_at_ns_[2].store(stamp, std::memory_order_relaxed);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus) {
            uart_rx_counts_[0].fetch_add(1, std::memory_order_relaxed);
            dbus_last_size_bytes_.store(data.uart_data.size(), std::memory_order_relaxed);
            if (data.uart_data.size() == 18) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                remote_at_ns_.store(now_ns(), std::memory_order_relaxed);
            } else
                dbus_bad_sizes_.fetch_add(1, std::memory_order_relaxed);
        } else if (uart == Spec::kUarts.kUart1)
            uart_rx_counts_[1].fetch_add(1, std::memory_order_relaxed);
        else if (uart == Spec::kUarts.kUart2)
            uart_rx_counts_[2].fetch_add(1, std::memory_order_relaxed);
    }

    std::shared_ptr<CommandTransmitter> command_component_;
    device::DjiMotor left_motor_, right_motor_, yaw_motor_;
    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::unique_ptr<GantryHoming> homing_;
    std::unique_ptr<librmcs::board::CBoard> board_;
    OutputInterface<double> left_position_m_, right_position_m_, yaw_position_m_;
    OutputInterface<bool> feedback_valid_, remote_valid_, fault_output_;
    OutputInterface<double> homing_state_output_;
    OutputInterface<double> home_left_reached_output_, home_right_reached_output_;
    OutputInterface<double> left_stall_output_, right_stall_output_, yaw_stall_output_;
    OutputInterface<double> dbus_rx_count_, uart1_rx_count_, uart2_rx_count_;
    OutputInterface<double> dbus_bad_size_count_, dbus_last_size_, remote_age_s_;
    std::array<std::atomic<std::uint64_t>, 3> uart_rx_counts_{};
    std::atomic<std::uint64_t> dbus_bad_sizes_{0}, dbus_last_size_bytes_{0};
    std::array<std::atomic<std::int64_t>, 3> feedback_at_ns_{};
    std::atomic<std::int64_t> remote_at_ns_{0};
    std::array<double, 3> angle_origin_{};
    std::array<bool, 3> origin_initialized_{};
    std::array<Clock::time_point, 3> stalled_since_{};
    std::array<std::atomic<bool>, 3> stall_latched_{};
    std::atomic<bool> fault_latched_{false};
    double pitch_lead_m_ = 0.0002, yaw_lead_m_ = 0.0002;
    double feedback_timeout_s_ = 0.1;
    double remote_timeout_s_ = 0.1;
    double stall_torque_nm_ = 0.2, stall_speed_rad_s_ = 0.05;
    double stall_target_speed_rad_s_ = 0.3, stall_duration_s_ = 0.2;
    double temperature_limit_c_ = 70.0;
    bool auto_home_ = false;
    double homing_torque_limit_nm_ = 0.4;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
