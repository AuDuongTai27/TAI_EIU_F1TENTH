#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool



class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """AckermannDriveStamped drive message.

        One publisher should publish to the /drive topic with a 
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.sub1= self.create_subscription(Odometry, '/odom',self.odom_callback, 10)
        self.sub2= self.create_subscription(LaserScan, '/scan',self.scan_callback,10)
        self.emergency_pub = self.create_publisher(Bool,'/emergency_breaking',10)
        self.check4 = self.create_subscription(Bool,'/check4',self.giuCheck4,10)

        self.range = None
        
        
        self.speed = None
        self.t = None
        self.check4 = False

        
    # def emergency_brake(self):
    #     msg =AckermannDriveStamped()
    #     msg.drive.speed = 0.0
    #     self.publisher_.publish(msg)
    #     self.timer.cancel()
        
    # def giuCheck4(self, msg:Bool):
    #     self.check4 = msg.data

    def odom_callback(self, msg:Odometry):
       
        # TODO: update current speed
        self.speed = msg.twist.twist.linear.x
    def scan_callback(self, msg:LaserScan):
        # TODO: calculate TTC
        emergency_breaking=False

        while (not self.check4):
            for i in range (len(msg.ranges)):
                self.t =msg.ranges[i] / max(self.speed * np.cos(msg.angle_min + i * msg.angle_increment), 0.1)
                print(self.t)
                if  self.t< 5:
                    emergency_breaking = True
                    self.check4=True
                    break

            emergency_msg = Bool()
            emergency_msg.data = emergency_breaking
            self.emergency_pub.publish(emergency_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()