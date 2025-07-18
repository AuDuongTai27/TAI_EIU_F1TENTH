#!/usr/bin/env python3
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.uic import loadUi
from PyQt6.QtCore import pyqtSignal, QThread

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.subscription
        self.callback_fn = None
        self.initialize = None

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.initialize is None:
            self.initialize = (x, y)

        distance = math.sqrt((x - self.initialize[0])**2 + (y - self.initialize[1])**2)

        if self.callback_fn:
            self.callback_fn(x, y, distance)


class ROS2Thread(QThread):
    update_odom = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.node = None

    def handle_odom_update(self, x, y, a):
        self.update_odom.emit(x, y, a)

    def run(self):
        rclpy.init()
        self.node = OdomNode()
        self.node.callback_fn = self.handle_odom_update
        rclpy.spin(self.node)
        rclpy.shutdown()

    def stop(self):
        if self.node:
            self.node.destroy_node()



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("ui/ui3.ui", self)

        self.ros_thread = ROS2Thread()
        self.ros_thread.update_odom.connect(self.update_gui_from_odom)
        self.ros_thread.start()

    def update_gui_from_odom(self, x, y, a):
        self.lineEdit_x.setText(f"{x:.2f}")
        self.lineEdit_y.setText(f"{y:.2f}")
        self.lineEdit_z.setText(f"{a:.2f}")

    def closeEvent(self, event):
        self.ros_thread.quit()
        self.ros_thread.wait()
        return super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
