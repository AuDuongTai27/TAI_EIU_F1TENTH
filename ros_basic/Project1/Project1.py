#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import time
import sympy as sp
import signal
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.uic import loadUi
import time
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtCore import QTimer
from sensor_msgs.msg import LaserScan
import math
class Move_robot(Node):
    def __init__(self, odom_signal,robot_stopped):
        super().__init__("move_robot_node")

        ### Create pub and sub nodes
        self.publisherDrive = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subOdom = self.create_subscription(Odometry, 'odom', self.listenOdom, 10)
        self.subScan = self.create_subscription(LaserScan,'scan',self.listenScan,10)

        ### Expected distance
        self.expected_distance=0.0


        ### 
        self.timer = None
        self.timer2 = None
        self.initialize = None
        self.currentPose = None
        self.odom_signal1 = odom_signal
        self.robot_stopped= robot_stopped


        self.current_x = 0.0
        self.current_y = 0.0
        self.distance = 0.0


        self.iniDistance=None
       # Khởi tạo trong hàm khởi tạo class
        self.obstacle_detected = False

    def start_moving(self):
            if self.timer is None and self.timer2 is None:
                    self.timer = self.create_timer(0.001, self.timer_callback)
                    self.timer2 = self.create_timer(0.001,self.distance_until_stop)

    def stop_moving(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            self.publisherDrive.publish(stop_msg)

    def stop_temporary(self):
            msg=AckermannDriveStamped()
            msg.drive.speed=0.0
            msg.drive.speed=0.0
            self.publisherDrive.publish(msg)

    def timer_callback(self):
            # Gọi hàm này để điều khiển xe
        if self.obstacle_detected:
            # Nếu đang có vật cản, không cho phép xe chạy tiếp
            self.stop_temporary()
        else:
            msg = AckermannDriveStamped()
            msg.drive.speed = 1.0
            msg.drive.steering_angle = 0.0
            self.publisherDrive.publish(msg)
    
    def listenScan(self, scan: LaserScan):
        
        for i in range(len(scan.ranges)):
            angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if -11<= angle_deg <= 11:
                distance1 = scan.ranges[i]
                if 0.01 < distance1 <= 5:
                    if not self.obstacle_detected:
                        print(f"🚫 Vật cản phía trước ({distance1:.2f}m)! Dừng xe.")
                    self.obstacle_detected = True
                else:
                    if self.obstacle_detected:
                        print("✅ Hết vật cản. Cho phép tiếp tục.")
                    self.obstacle_detected = False


    def listenOdom(self, msg:Odometry):
        self.current_x= msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if self.initialize is None:
            self.initialize = [self.current_x, self.current_y]
        self.currentPose = [self.current_x, self.current_y]
        self.distance = float(sp.sqrt((self.current_x - self.initialize[0])**2 + (self.current_y - self.initialize[1])**2))
        
    def distance_until_stop(self):        
        if self.iniDistance ==None:
            self.iniDistance = self.distance
        distance2=self.distance-self.iniDistance
        print(distance2)
        if (distance2>self.expected_distance):
            if self.timer2 is not None:
                self.timer2.cancel()
                self.timer2 = None
                self.timer.cancel()
                self.timer=None
                self.stop_moving()
        
        
      
def main():
    rclpy.init()
    move_robot = Move_robot()
    rclpy.spin(move_robot)
    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()