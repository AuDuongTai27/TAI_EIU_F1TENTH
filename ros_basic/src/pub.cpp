#include "ros_basic/pub.hpp"
using namespace std::chrono_literals;

Pub::Pub() : Node("pub")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("text", 10);
  timer_ = this->create_wall_timer(1s, std::bind(&Pub::timer_callback, this));

}

void Pub::timer_callback(){
   chatter_name = "Tai";
  RCLCPP_INFO(this->get_logger(),"Xin chao %s",chatter_name.c_str());
  auto msg = std_msgs::msg::String();
  msg.data = chatter_name;
  publisher_->publish(msg);
}