#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <librmcs/board/c_board.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

#include <numbers>

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
                get_component_name() + "_command", *this)}
        , left_motor_{*this, *command_component_, "/gantry/left_motor"}
        , right_motor_{*this, *command_component_, "/gantry/right_motor"} {

        reduction_ratio_ = get_parameter("reduction_ratio").as_double();
        lead_screw_ = get_parameter("lead_screw").as_double();

        left_motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM2006, 1}
                .set_reduction_ratio(reduction_ratio_)
                .enable_multi_turn_angle());
        right_motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM2006, 2}
                .set_reduction_ratio(reduction_ratio_)
                .enable_multi_turn_angle());

        register_output("/gantry/left_motor/velocity", left_velocity_, 0.0);
        register_output("/gantry/right_motor/velocity", right_velocity_, 0.0);
        register_output("/gantry/left_motor/angle", left_angle_, 0.0);
        register_output("/gantry/right_motor/angle", right_angle_, 0.0);
        register_output("/gantry/left_position_m", left_position_m_, 0.0);
        register_output("/gantry/right_position_m", right_position_m_, 0.0);

        std::string serial = get_parameter("board_serial").as_string();
        if (!serial.empty()) {
            board_ = std::make_unique<librmcs::board::CBoard>(*this, serial);
        }
    }

    void update() override {
        left_motor_.update_status();
        right_motor_.update_status();

        *left_velocity_ = left_motor_.velocity();
        *right_velocity_ = right_motor_.velocity();
        *left_angle_ = left_motor_.angle();
        *right_angle_ = right_motor_.angle();

        constexpr double two_pi = 2.0 * std::numbers::pi;
        *left_position_m_ = left_motor_.angle() / reduction_ratio_ / two_pi * lead_screw_;
        *right_position_m_ = right_motor_.angle() / reduction_ratio_ / two_pi * lead_screw_;
    }

private:
    class CommandTransmitter : public rmcs_executor::Component {
    public:
        explicit CommandTransmitter(Gantry& gantry)
            : gantry_(gantry) {}

        void update() override {
            gantry_.command_update();
        }

    private:
        Gantry& gantry_;
    };

    std::shared_ptr<CommandTransmitter> command_component_;

    device::DjiMotor left_motor_;
    device::DjiMotor right_motor_;

    double reduction_ratio_ = 1.0;
    double lead_screw_ = 0.01;

    OutputInterface<double> left_velocity_;
    OutputInterface<double> right_velocity_;
    OutputInterface<double> left_angle_;
    OutputInterface<double> right_angle_;
    OutputInterface<double> left_position_m_;
    OutputInterface<double> right_position_m_;

    std::unique_ptr<librmcs::board::CBoard> board_;

    void command_update() {
        if (!board_) return;

        auto builder = board_->start_transmit();
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
            || data.can_data.size() < 8) [[unlikely]] {
            return;
        }
        if (can != Spec::kCans.kCan1) {
            return;
        }
        if (data.can_id == 0x201) {
            left_motor_.store_status(data.can_data);
        } else if (data.can_id == 0x202) {
            right_motor_.store_status(data.can_data);
        }
    }
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Gantry,
    rmcs_executor::Component)
