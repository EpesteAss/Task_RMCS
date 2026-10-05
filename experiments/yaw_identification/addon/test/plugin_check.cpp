#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <rmcs_executor/component.hpp>
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        pluginlib::ClassLoader<rmcs_executor::Component> loader(
            "rmcs_executor", "rmcs_executor::Component");
        rmcs_executor::Component::initializing_component_name = "yaw_experiment_plugin_test";
        auto instance = loader.createSharedInstance("yaw_identification::Controller");
        // Constructor only: no hardware instantiated and no control update executed.
    }
    rclcpp::shutdown();
}
