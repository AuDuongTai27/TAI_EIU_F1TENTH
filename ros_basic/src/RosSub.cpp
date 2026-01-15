#include "RosSub.h"

using std::placeholders::_1;

RosSubscriber::RosSubscriber() : Node("subscriber_node") {
    RCLCPP_INFO(this->get_logger(), "subscriber_node initialized");

    // Khởi tạo Subscriber nghe topic "chatter"
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&RosSubscriber::listen, this, _1));
}

void RosSubscriber::listen(const std_msgs::msg::String::SharedPtr msg) const {
    // In ra giống lệnh print(msg) trong Python
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
}