#include "rclcpp/rclcpp.hpp"
#include "RosPub.h"
#include "RosSub.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // Bạn có thể chọn chạy 1 trong 2 node
    auto node = std::make_shared<PubNode>();
    // auto node = std::make_shared<RosSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
