#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool
class stop(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('dung xe')

        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        msg = AckermannDriveStamped()
        msg.drive.speed=0.0
        self.publisher_.publish(msg)
        