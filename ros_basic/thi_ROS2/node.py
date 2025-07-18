#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class MinimalNode(Node):
    def __init__(self):
        super().__init__('my_node')  # tên node là 'my_node'
        self.get_logger().info("Node đã khởi động!")
        self.pub= self.create_publisher(String, 'my_topic', 10)
        self.sub= self.create_subscription(String, 'my_topic', self.listener_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    def listener_callback(self, msg):
        self.get_logger().info(f"Nhận được tin nhắn: {msg.data}")
    def timer_callback(self):
        msg = String()
        msg.data = "Tin nhắn từ timer"
        self.pub.publish(msg)
        self.get_logger().info(f"Đã gửi tin nhắn: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)  # giữ cho node chạy
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()