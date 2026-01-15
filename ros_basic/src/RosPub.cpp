#include "RosPub.h"

using namespace std::chrono_literals;

PubNode::PubNode(): Node("SimplePub")
{
  RCLCPP_INFO(this->get_logger(), "publisher_node initialized");

  // Khởi tạo Publisher
  publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

  // Khởi tạo Timer (1 giây)
  timer_ = this->create_wall_timer(1s, std::bind(&PubNode::timer_callback, this));
}
void PubNode::timer_callback(){
  chatter_name = "Tai";
  RCLCPP_INFO(this->get_logger(),"Xin chao %s",chatter_name.c_str());
  auto msg = std_msgs::msg::String();
  msg.data = chatter_name;
  publisher_->publish(msg);
}


