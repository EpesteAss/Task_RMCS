#include <cmath>
#include <limits>
#include <memory>

#include <eigen3/Eigen/Core>
#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::hardware {

class Gantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

public:
    Gantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)}
        , command_component_{
              create_partner_component<CommandTransmitter>(
                  get_component_name() + "_command",
                  *this)}
        , left_motor_{
              *this,
              *command_component_,
              "/gantry/left_motor"}
        , right_motor_{
              *this,
              *command_component_,
              "/gantry/right_motor"}
        , velocity_pid_calculator_{
              get_parameter("kp").as_double(),
              get_parameter("ki").as_double(),
              get_parameter("kd").as_double()} {

        target_velocity_ =
            get_parameter("target_velocity").as_double();

        sync_coefficient_ =
            get_parameter("sync_coefficient").as_double();

        left_motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM2006,
                1}
                .set_reduction_ratio(1.0)
                .enable_multi_turn_angle());

        right_motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM2006,
                2}
                .set_reduction_ratio(1.0)
                .enable_multi_turn_angle());

        register_output(
            "/gantry/left_motor/velocity",
            left_velocity_,
            0.0);

        register_output(
            "/gantry/right_motor/velocity",
            right_velocity_,
            0.0);

        register_output(
            "/gantry/left_motor/angle",
            left_angle_,
            0.0);

        register_output(
            "/gantry/right_motor/angle",
            right_angle_,
            0.0);

        register_output(
            "/gantry/position_error",
            position_error_,
            0.0);

        register_output(
            "/gantry/left_motor/control_torque",
            left_control_torque_,
            nan_);

        register_output(
            "/gantry/right_motor/control_torque",
            right_control_torque_,
            nan_);

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string());
    }

    Gantry(const Gantry&) = delete;
    Gantry& operator=(const Gantry&) = delete;

    Gantry(Gantry&&) = delete;
    Gantry& operator=(Gantry&&) = delete;

    ~Gantry() override = default;

    void update() override {
        // 更新两个电机的反馈
        left_motor_.update_status();
        right_motor_.update_status();

        const double left_velocity =
            left_motor_.velocity();

        const double right_velocity =
            right_motor_.velocity();

        const double left_angle =
            left_motor_.angle();

        const double right_angle =
            right_motor_.angle();

        // 输出电机速度反馈
        *left_velocity_ = left_velocity;
        *right_velocity_ = right_velocity;

        // 输出电机角度反馈
        *left_angle_ = left_angle;
        *right_angle_ = right_angle;

        // 利用左右电机角度差估计龙门左右位置偏差
        if (std::isfinite(left_angle)
            && std::isfinite(right_angle)) {
            *position_error_ =
                left_angle - right_angle;
        } else {
            *position_error_ = nan_;
        }

        // 反馈无效时停止输出控制量
        if (!std::isfinite(left_velocity)
            || !std::isfinite(right_velocity)
            || !std::isfinite(target_velocity_)) {

            *left_control_torque_ = nan_;
            *right_control_torque_ = nan_;
            return;
        }

        // 左右电机的速度误差
        Eigen::Vector2d setpoint_error{
            target_velocity_ - left_velocity,
            target_velocity_ - right_velocity};

        // 左右电机之间的速度同步误差
        Eigen::Vector2d relative_velocity{
            left_velocity - right_velocity,
            right_velocity - left_velocity};

        // 将同步误差加入速度控制误差
        Eigen::Vector2d control_error =
            setpoint_error
            - sync_coefficient_
                  * relative_velocity;

        // 使用已有的二维 Matrix PID 同时计算两个电机控制量
        const auto control_torques =
            velocity_pid_calculator_.update(
                control_error);

        *left_control_torque_ =
            control_torques[0];

        *right_control_torque_ =
            control_torques[1];
    }

private:
    static constexpr double nan_ =
        std::numeric_limits<double>::quiet_NaN();

    class CommandTransmitter
        : public rmcs_executor::Component {

    public:
        explicit CommandTransmitter(Gantry& gantry)
            : gantry_(gantry) {}

        void update() override {
            gantry_.command_update();
        }

    private:
        Gantry& gantry_;
    };

    std::shared_ptr<CommandTransmitter>
        command_component_;

    device::DjiMotor left_motor_;
    device::DjiMotor right_motor_;

    rmcs_core::controller::pid::MatrixPidCalculator<2>
        velocity_pid_calculator_;

    double target_velocity_{0.0};
    double sync_coefficient_{0.0};

    OutputInterface<double> left_velocity_;
    OutputInterface<double> right_velocity_;

    OutputInterface<double> left_angle_;
    OutputInterface<double> right_angle_;

    OutputInterface<double> position_error_;

    OutputInterface<double> left_control_torque_;
    OutputInterface<double> right_control_torque_;

    std::unique_ptr<librmcs::board::CBoard>
        board_;

    void command_update() {
        auto builder =
            board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        left_motor_.generate_command(),
                        right_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data) override {

        if (data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() < 8) {
            return;
        }

        if (can != Spec::kCans.kCan1) {
            return;
        }

        if (data.can_id == 0x201) {
            left_motor_.store_status(
                data.can_data);
        } else if (data.can_id == 0x202) {
            right_motor_.store_status(
                data.can_data);
        }
    }
};

}  // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Gantry,
    rmcs_executor::Component)