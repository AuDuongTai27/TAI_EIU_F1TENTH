#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool



class brake(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('chay')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.stop = None
        self.timer = self.create_timer(0.01, self.start_run)
        self.sub_emergency_signals = self.create_subscription(Bool, '/emergency_breaking',self.brake_callback, 10)
        self.check4_pub = self.create_publisher(Bool,'/check4',10)
        self.check2 = True     

    def start_run(self):
        msg =AckermannDriveStamped()
        if (self.check2):
            if (self.stop):
                msg.drive.speed=0.0
                self.check2=False
            else:
                msg.drive.speed=1.0
            self.publisher_.publish(msg)

    def brake_callback(self, msg: Bool):
        self.stop = msg.data

def main(args=None):
    rclpy.init(args=args)
    breaking = brake()
    rclpy.spin(breaking)
    breaking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
