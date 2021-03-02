#include "micro_ros_diagnostic_bridge/micro_ros_diagnostic_bridge.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MicroROSDiagnosticBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
