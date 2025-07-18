#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time

class MODE(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class AEB(Node):
    
    def __init__(self):
        super().__init__('AEB')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.sub1= self.create_subscription(Odometry, '/odom',self.odom_callback, 10)
        self.sub2= self.create_subscription(LaserScan, '/scan',self.scan_callback,10)
        self.speed = 0.0
        self.t = None
        self.mode = MODE.FREE
        time.sleep(1)
        self.run()
        
    def odom_callback(self, msg:Odometry):
        self.speed = msg.twist.twist.linear.x
    def scan_callback(self, msg:LaserScan):
        self.mode =MODE.FREE
        for i in range (len(msg.ranges)):
                angle_deg = np.degrees(msg.angle_min + i * msg.angle_increment)
                if -10< angle_deg < 10:
                  self.t =msg.ranges[i] / max(self.speed * np.cos(angle_deg), 0.8)
                  print(self.t)
                  if  self.t< 1.4:
                      self.mode = MODE.DANGER
                      break
                  if  1.4<self.t<2.0:
                      self.mode = MODE.WARNING
                      break
            
                  
        if (self.mode == MODE.DANGER):
            print("Stop")
            stop = AckermannDriveStamped()
            stop.drive.speed=0.0
            self.publisher_.publish(stop)
        elif (self.mode == MODE.WARNING):
            print("Slow")
            stop = AckermannDriveStamped()
            stop.drive.speed=0.5
            self.publisher_.publish(stop)
        else:
            self.run()

    
    
    def run(self):
        msg =AckermannDriveStamped()
        msg.drive.speed = 1.2
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    brake = AEB()
    rclpy.spin(brake)
    brake.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

             
