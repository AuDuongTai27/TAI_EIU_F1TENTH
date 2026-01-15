#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PointStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

class SimpleSimulator(Node):
    def __init__(self):
        super().__init__('simple_simulator')
        
        # --- Config ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.steering_angle = 0.0
        self.wheelbase = 0.33
        
        # Vật cản (Mặc định ở vô cực)
        self.obstacle_pos = None 
        self.obstacle_radius = 0.5 

        # --- Pub/Sub ---
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, 10)
        # Lắng nghe nút "Publish Point" trên RViz để tạo vật cản
        self.create_subscription(PointStamped, '/clicked_point', self.obstacle_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/ego_racecar/odom', 10)
        self.marker_pub = self.create_publisher(Marker, '/sim_obstacle', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.create_timer(0.02, self.update_physics)
        self.get_logger().info("Simulator Ready. Click 'Publish Point' in RViz to spawn Obstacle.")

    def drive_callback(self, msg):
        self.velocity = msg.drive.speed
        self.steering_angle = msg.drive.steering_angle

    def obstacle_callback(self, msg):
        # Nếu click gần vật cản cũ (< 1m) -> Xóa vật cản
        if self.obstacle_pos is not None:
            dist = math.dist([self.obstacle_pos[0], self.obstacle_pos[1]], [msg.point.x, msg.point.y])
            if dist < 1.0:
                self.obstacle_pos = None
                self.get_logger().info("Obstacle Removed!")
                self.publish_obstacle()
                return

        # Tạo vật cản mới
        self.obstacle_pos = [msg.point.x, msg.point.y]
        self.get_logger().info(f"Obstacle Spawned at: {self.obstacle_pos}")
        self.publish_obstacle()

    def publish_obstacle(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        if self.obstacle_pos:
            marker.pose.position.x = self.obstacle_pos[0]
            marker.pose.position.y = self.obstacle_pos[1]
            marker.scale.x = self.obstacle_radius * 2
            marker.scale.y = self.obstacle_radius * 2
            marker.scale.z = 1.0
            marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0 # Đỏ
        else:
            marker.action = Marker.DELETE # Xóa marker nếu không có vật cản

        self.marker_pub.publish(marker)

    def update_physics(self):
        # Kinematic Bicycle Model
        self.x += self.velocity * math.cos(self.theta) * 0.02
        self.y += self.velocity * math.sin(self.theta) * 0.02
        self.theta += (self.velocity / self.wheelbase) * math.tan(self.steering_angle) * 0.02

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]; odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x; t.transform.translation.y = self.y
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSimulator()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()