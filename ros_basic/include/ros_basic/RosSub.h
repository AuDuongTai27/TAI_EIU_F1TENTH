#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RosSubscriber : public rclcpp::Node {
public:
    RosSubscriber();

private:
    // Callback nhận tin nhắn
    void listen(const std_msgs::msg::String::SharedPtr msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif