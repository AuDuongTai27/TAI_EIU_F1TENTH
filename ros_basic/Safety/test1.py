#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool

class test (Node):
    def __init__(self):
        super().__init__('test')
        self.emergency_pub = self.create_publisher(Bool,'/emergency_breaking',10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.counter =0
        self.giuBool = Bool()
    def timer_callback(self):
        print(self.counter)
        self.counter +=1
        if (self.counter%2==0):
            self.giuBool.data = True
            self.emergency_pub.publish(self.giuBool)
        else:
            self.giuBool.data = False
            self.emergency_pub.publish(self.giuBool)

def main(args=None):
    rclpy.init(args=args)
    test1 = test()
    rclpy.spin(test1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()