#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from  sensor_msgs.msg import LaserScan
class Ros_SubscriberScan(Node):
    def __init__(self):
        super().__init__("publisher_node")
        print("publisher_node initialized ")
        self.get_logger().info("publisher_node initialized")
        
        self._sub= self.create_subscription(LaserScan, 'scan',self.listen, 10)

    def listen(self,msg):
        print("msg =     ", msg )

def main():
    rclpy.init()
    node = Ros_SubscriberScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()