#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from rosbasic_msgs.srv import BaiTap1
import math
import sympy as sp
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self.group = ReentrantCallbackGroup()
        self.service_ = self.create_service(BaiTap1, "Car", self.serviceCallback,callback_group=self.group)
        self.get_logger().info("Service Car Ready")
        self.publisherDrive = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subOdom = self.create_subscription(Odometry, 'odom', self.listenOdom, 10,callback_group=self.group)
        self.subScan = self.create_subscription(LaserScan,'scan',self.listenScan,10,callback_group=self.group)
        self.j=0.0
        self.timer = None
        self.timer2 = None
        self.initialize = None
        self.currentPose = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance = 0.0
        self.iniDistance=None
        self.tocDo=0.0
       # Khởi tạo trong hàm khởi tạo class
        self.clientStop=False
        
        # self.j=0.0
    def start_moving(self):
        msg=AckermannDriveStamped()
        msg.drive.speed=self.tocDo
        
        msg.drive.steering_angle=0.0
        self.publisherDrive.publish(msg)
    def start_stop(self,test):
        print("📡 start_stop called")
        self.j=test
        self.iniDistance=None
        if self.timer2 is None:
            self.timer2 = self.create_timer(0.001, self.maymet)

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

    # def timer_callback(self):
   
    #         msg = AckermannDriveStamped()
    #         msg.drive.speed = 1.0
    #         msg.drive.steering_angle = 0.0
    #         self.publisherDrive.publish(msg)
    
    def listenScan(self, scan: LaserScan):
        
        for i in range(len(scan.ranges)):
            angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if -11<= angle_deg <= 11:
                distance1 = scan.ranges[i]
                
                if 0.01 < distance1 <= 4:
                    self.stop_temporary()
                else:
                    self.start_moving()



    def listenOdom(self, msg:Odometry):
        self.current_x= msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if self.initialize is None:
            self.initialize = [self.current_x, self.current_y]
        self.currentPose = [self.current_x, self.current_y]
        self.distance = float(sp.sqrt((self.current_x - self.initialize[0])**2 + (self.current_y - self.initialize[1])**2))
       
        
    def maymet(self):        
        if self.iniDistance ==None:
            self.iniDistance = self.distance
        distance2=self.distance-self.iniDistance
        # self.j=float(j)
        if (distance2>self.j):
            if self.timer2 is not None:
                self.timer2.cancel()
                self.timer2 = None
                self.timer.cancel()
                self.timer=None
                stop_msg = AckermannDriveStamped()
                stop_msg.drive.speed = 0.0
                self.publisherDrive.publish(stop_msg)
                
        

    def serviceCallback(self, req, res):
        if req.ok:  # nếu ok=True thì dừng lại
            self.clientStop = True
            self.stop_moving()
            res.msg = "Robot stopped"
        else:       # nếu ok=False thì bắt đầu chạy
            self.tocDo = req.speed
            self.clientStop = False
            self.start_moving()
            res.msg = "Robot started"
        return res

    









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

