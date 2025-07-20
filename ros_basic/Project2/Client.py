#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbasic_msgs.srv import BaiTap2

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__("simple_service_client")
        self.client_ = self.create_client(BaiTap2, "BaiTap")

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for service...")

        self.req_ = BaiTap2.Request()
        self.declare_parameter("distance_odom",0.0)
        self.declare_parameter("distance_lidar",0.0)
        
    
        self.req_.distance_odom =self.get_parameter("distance_odom").value
        self.req_.distance_lidar=self.get_parameter("distance_lidar").value

        self.future = self.client_.call_async(self.req_)
        self.future.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"✅ Service responded: {response.message}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed: {e}")

def main():
    rclpy.init()
    client_node = SimpleServiceClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
