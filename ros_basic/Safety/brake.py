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

        self.chaydi = None
        self.timer = self.create_timer(0.01, self.start_run)

        self.sub1= self.create_subscription(Bool, '/emergency_breaking',self.brake_callback, 10)
        self.check4_pub = self.create_publisher(Bool,'/check4',10)
        # self.sub2= self.create_subscription(LaserScan, '/scan',self.checkAround,10)

        self.check2 = True
        
        
    # def checkAround(self, scan:LaserScan):
    #     while (not self.check2):
    #         checkVar = False
    #         for i in range(len(scan.ranges)):
    #             angle_deg = np.degrees(scan.angle_min + i * scan.angle_increment)
    #             if -1< angle_deg < 1:
    #                 print(scan.ranges[i])
    #                 if (scan.ranges[i]<2.0):
    #                     print(" chua an toan")
    #                     checkVar=True
    #                     break
                        
    #         if (not checkVar):
    #             print ("Check 4:   ")
    #             msg = Bool()
    #             msg.data = False
    #             self.check4_pub.publish(msg)
    #             self.check2=True
             

    def start_run(self):
        msg =AckermannDriveStamped()
        if (self.check2):
            if (self.chaydi):
                msg.drive.speed=0.0
                self.check2=False
             
            else:
                msg.drive.speed=1.0
            self.publisher_.publish(msg)

    def brake_callback(self, msg: Bool):
        # self.chaydi = msg.data
        # if self.chaydi != msg.data:
        self.chaydi = msg.data
        #     drive_msg = AckermannDriveStamped()
        #     drive_msg.drive.speed = 0.0 if self.chaydi else 1.0
        #     self.publisher_.publish(drive_msg)






def main(args=None):
    rclpy.init(args=args)
    breaking = brake()
    rclpy.spin(breaking)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    breaking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
