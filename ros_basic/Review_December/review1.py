#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from example_interfaces.srv import AddTwoInts
import math
import sympy as sp
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self.group = ReentrantCallbackGroup()
        self.service_ = self.create_service(AddTwoInts, "AddTwoInts", self.serviceCallback,callback_group=self.group)
        self.get_logger().info("Service Car Ready")
        self.sub = self.create_subscription(Odometry, 'odom', self.listten_sub, 10,callback_group=self.group)

    def serviceCallback(self, req, res):
        res.sum = req.a + req.b
        self.get_logger().info(f"Incoming request\na: {req.a} b: {req.b}")
        return res
    
    def listten_sub(self,msg):
        print("x (linear) =     ", msg.twist.twist.linear.x )
        print("z (orientation) =     ", msg.pose.pose.orientation.z )

def main(args=None):
    rclpy.init(args=args)
    try:
        talker = SimpleServiceServer()
        # parallel to the ones in DoubleTalker however.
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(talker)
        try:
            executor.spin()
        finally:
            talker.run = False
            executor.shutdown()
            talker.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

