#!/usr/bin/env python3
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool 
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math

class State(Enum):
    FREE = 0
    DANGER = 1

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")
        self.declare_parameter("danger_distance", 0.5)
        # self.declare_parameter("warning_distance", 2.0)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")

        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        # self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.state = State.FREE
        self.prev_state = State.FREE
        self.is_first_msg = True

        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.zones_pub = self.create_publisher(MarkerArray, "zones_Tai", 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)

        # Tạo hình cánh quạt thay vì hình tròn
        self.zones = MarkerArray()
        
        # # Vùng cảnh báo (warning) - hình cánh quạt
        # warning_zone = self.create_fan_marker(
        #     id=0,
        #     distance=self.warning_distance,
        #     color_r=1.0, color_g=0.984, color_b=0.0, color_a=0.5
        # )
        
        # Vùng nguy hiểm (danger) - hình cánh quạt
        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.action = Marker.ADD
        danger_zone.type = Marker.CYLINDER
        danger_zone.scale.z = 0.2
        danger_zone.scale.x = self.danger_distance * 2
        danger_zone.scale.y = self.danger_distance * 2
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 1.0  # Vẫn cần set dù bạn nói không quan tâm
        danger_zone.pose.position.x = 0.0
        danger_zone.pose.position.y = 0.0
        danger_zone.pose.position.z = 0.5  # Đặt cao hơn để dễ nhìn



        text = Marker()
        text.id = 2
        text.action = Marker.ADD
        text.type = Marker.TEXT_VIEW_FACING
        text.scale.z = 0.5
        text.scale.x = 0.5
        text.scale.y = 0.5
        text.color.r = 1.0
        text.color.g = 0.9
        text.color.b = 0.5
        text.color.a = 1.0  # Vẫn cần set dù bạn nói không quan tâm
        text.pose.position.x = 0.0
        text.pose.position.y = 0.8
        text.pose.position.z = 2.0  # Đặt cao hơn để dễ nhìn
        text.text = "Eastern International University"
        self.zones.markers = [danger_zone,text]
        


    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE

        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.danger_distance:
                self.state = State.DANGER
                break
            

        if self.state != self.prev_state:
            is_safety_stop = Bool()
            if self.state == State.DANGER:
                is_safety_stop.data = True
                self.zones.markers[0].color.a = 1.0  # Làm đậm cả hai vùng
            elif self.state == State.FREE:
                self.zones.markers[0].color.a = 0.5  # Trở về trạng thái ban đầu
                is_safety_stop.data = False

            self.prev_state = self.state
            self.safety_stop_pub.publish(is_safety_stop)
        
        if self.is_first_msg:
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id
            self.is_first_msg = False
        
        self.zones_pub.publish(self.zones)

def main():
    rclpy.init()
    safety_stop_node = SafetyStop()
    rclpy.spin(safety_stop_node)
    safety_stop_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()