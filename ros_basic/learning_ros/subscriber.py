#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Ros_Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        print("subscriber_node initialized ")
        self.get_logger().info("subscriber_node initialized")
        
        self._sub= self.create_subscription(String, '/chatter',self.listen, 10)

    def listen(self,msg):
        print("msg =     ", msg )

def main():
    rclpy.init()
    node = Ros_Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()