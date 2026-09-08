#include <cmath>
#include <memory>
#include <numbers>

#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::hardware {

class M6020Task3Hardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

public:
    M6020Task3Hardware()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)}
        , command_component_{
              create_partner_component<CommandTransmitter>(
                  get_component_name() + "_command", *this)}
        , motor_{*this, *command_component_, "/m6020"} {

        // GM6020, CAN ID = 1
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kGM6020,
                1}
                .enable_multi_turn_angle());

        target_angle_ = get_parameter("target_angle").as_double();

        major_arc_tolerance_ =
            get_parameter("major_arc_tolerance").as_double();

        register_output(
            "/m6020/target_angle",
            target_angle_output_,
            0.0);

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string());
    }

    void update() override {

        motor_.update_status();

        const double current_angle = motor_.angle();

        if (!target_initialized_) {
            target_angle_output_value_ =
                resolve_major_arc_target(
                    current_angle,
                    target_angle_);

            target_initialized_ = true;
        }

        *target_angle_output_ =
            target_angle_output_value_;
    }

private:

    class CommandTransmitter : public rmcs_executor::Component {
    public:
        explicit CommandTransmitter(
            M6020Task3Hardware& hardware)
            : hardware_(hardware) {}

        void update() override {
            hardware_.command_update();
        }

    private:
        M6020Task3Hardware& hardware_;
    };

    void command_update() {

        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = motor_.send_id(),
                .can_data =
                    device::CanPacket8{
                        motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{}}
                        .as_bytes(),
            });
    }

    static double wrap_angle(double angle) {

        angle =
            std::remainder(
                angle,
                2.0 * std::numbers::pi);

        if (angle <= -std::numbers::pi)
            angle += 2.0 * std::numbers::pi;

        return angle;
    }


    double resolve_major_arc_target(
        double current,
        double target) const {

        const double shortest_error =
            wrap_angle(target - current);

        if (std::abs(shortest_error) <= major_arc_tolerance_)
            return current;

        const double major_error =
            shortest_error > 0.0
                ? shortest_error - 2.0 * std::numbers::pi
                : shortest_error + 2.0 * std::numbers::pi;

        return current + major_error;
    }

    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data) override {

        if (data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() < 8)
            return;

        if (can != Spec::kCans.kCan1)
            return;

        if (data.can_id == 0x205)
            motor_.store_status(data.can_data);
    }

    std::shared_ptr<CommandTransmitter>
        command_component_;

    std::unique_ptr<librmcs::board::CBoard>
        board_;

    device::DjiMotor motor_;

    OutputInterface<double>
        target_angle_output_;

    double target_angle_ = 0.0;
    double major_arc_tolerance_ = 0.05;

    double target_angle_output_value_ = 0.0;

    bool target_initialized_ = false;
};

}  // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::M6020Task3Hardware,
    rmcs_executor::Component)