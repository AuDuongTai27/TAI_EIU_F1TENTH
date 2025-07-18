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

class MainWindow(QMainWindow):
    update_odom = pyqtSignal(float, float, float)
    robot_stopped = pyqtSignal()
    def __init__(self):

        
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # mỗi 10ms spin 1 lần
        super().__init__()
        loadUi("ui/ui3.ui", self)
        # update_odom = pyqtSignal(float, float, float)
        self.startButton.clicked.connect(self.start_clicked)

        self.stopButton.clicked.connect(self.stop_clicked)

        self.StopIn.clicked.connect(self.dimaymet)

        # self.rosthread = ROS2Thread()

        self.update_odom.connect(self.update_labels)

        self.move_move = Move_robot(self.update_odom,self.robot_stopped)
        # self.node = Move_robot(self.update_odom)
        # rclpy.spin(self.move_move)
        
        self.move_move.robot_stopped.connect(self.reset_button_color)
        


        # self.rosthread.start()
    def spin_ros(self):
        rclpy.spin_once(self.move_move, timeout_sec=0)

        
    def start_clicked(self):
        print("Start button clicked")
        self.startButton.setStyleSheet("background-color: 	#FFEB3B; color: black;")
        self.move_move.start_moving()
        

    def stop_clicked(self):
        print("Stop button clicked")
        self.startButton.setStyleSheet("")
        self.move_move.stop_moving()
    def update_labels(self, x, y, d):
        self.xOdom.setText(f"{x:.2f}")
        self.yOdom.setText(f"{y:.2f}")
        self.dOdom.setText(f"{d:.2f}")


    def dimaymet(self):
        self.StopIn.setStyleSheet("background-color: #FFEB3B;")

        j_value = self.LineEdit.value()  # Lấy giá trị float từ QDoubleSpinBox
        self.move_move.start_stop(j_value)
    
    def reset_button_color(self):
        self.StopIn.setStyleSheet("")  # 👈 Đổi về màu mặc định
        self.startButton.setStyleSheet("")
        

class Move_robot(Node):
    def __init__(self, odom_signal,robot_stopped):
        super().__init__("move_robot_node")
        self.publisherDrive = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subOdom = self.create_subscription(Odometry, 'odom', self.listenOdom, 10)
        self.subScan = self.create_subscription(LaserScan,'scan',self.listenScan,10)
        self.j=0.0
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

        
        # self.j=0.0
    def start_moving(self):
            if self.timer is None:
                    self.timer = self.create_timer(0.001, self.timer_callback)
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
                print(distance1)
                if 0.01 < distance1 <= 5:
                    if not self.obstacle_detected:
                        print(f"🚫 Vật cản phía trước ({distance1:.2f}m)! Dừng xe.")
                    self.obstacle_detected = True
                else:
                    if self.obstacle_detected:
                        print("✅ Hết vật cản. Cho phép tiếp tục.")
                    self.obstacle_detected = False


        # index_0 = int((0.0 - scan.angle_min) / scan.angle_increment) ##tìm tia mà tại đó tượng trưng cho góc 0 độ
        # if 0 <= index_0 < len(scan.ranges):
        #     distance1 = scan.ranges[index_0]
        #     if 0.01 < distance1 <= 2.0:
        #         if not self.obstacle_detected:
        #             print(f"🚫 Vật cản phía trước ({distance1:.2f}m)! Dừng xe.")
        #         self.obstacle_detected = True
        #     else:
        #         if self.obstacle_detected:
        #             print("✅ Hết vật cản. Cho phép tiếp tục.")
        #         self.obstacle_detected = False
                    

    def listenOdom(self, msg:Odometry):
        self.current_x= msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if self.initialize is None:
            self.initialize = [self.current_x, self.current_y]
        self.currentPose = [self.current_x, self.current_y]
        self.distance = float(sp.sqrt((self.current_x - self.initialize[0])**2 + (self.current_y - self.initialize[1])**2))
        self.odom_signal1.emit(self.current_x, self.current_y, self.distance)
        
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
                self.robot_stopped.emit() 
        
        
        

# def signal_handler(sig, frame):
#     print("Ctrl+C detected. Stopping robot.")
#     stop_msg = AckermannDriveStamped()
#     stop_msg.drive.speed = 0.0
#     if node is not None:
#         node.publisher_.publish(stop_msg)
#         time.sleep(0.1)
#         node.destroy_node()
#     rclpy.shutdown()
#     sys.exit(0)

##signal.signal(signal.SIGINT, signal_handler)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
    rclpy.shutdown()

    
if __name__ == "__main__":
    main()
