import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.uic import loadUi
import time
from PyQt6.QtCore import pyqtSignal, QObject
from move_robot import Move_robot
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__
        loadUi("ui/ui3.ui",self)
        self.startOdom.clicked.connect(self.start_odom)

        
            

