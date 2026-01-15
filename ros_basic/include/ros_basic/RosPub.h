#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
class PubNode : public rclcpp::Node{
	public:
    PubNode();
	private:
		void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string chatter_name;
};
#endif