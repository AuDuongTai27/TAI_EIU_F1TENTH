#ifndef PUB_HPP
#define PUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Pub : public rclcpp::Node
{
public:
    Pub();
    void timer_callback();


private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
   std::string chatter_name;
};


#endif // MOTOR_CONTROLLER_HPP