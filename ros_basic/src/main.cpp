#include "ros_basic/pub.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pub>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "❌ Runtime error: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "✅ MotorController Node has shut down cleanly.");

    if (rclcpp::ok()) 
    {
        rclcpp::shutdown();
    }

    return 0;
  }