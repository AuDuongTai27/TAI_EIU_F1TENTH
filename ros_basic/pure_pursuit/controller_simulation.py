#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # --- 1. Parameters ---
        self.declare_parameter("waypoint_path", "/home/adt/ros2_ws/install/waypoint/share/waypoint/f1tenth_waypoint_generator/racelines/f1tenth_waypoint.csv")
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("max_speed", 4.0)
        
        self.csv_path = self.get_parameter("waypoint_path").value
        self.L = self.get_parameter("lookahead_dist").value
        self.max_speed = self.get_parameter("max_speed").value
        
        # --- 2. Load Data ---
        self.waypoints = self.load_waypoints(self.csv_path)
        self.last_idx = 0
        
        # --- 3. Pub/Sub ---
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/markers', 10)
        self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 10)
        
        # Vẽ đường Path tĩnh 1 lần
        self.publish_static_path()
        self.get_logger().info(f"Pure Pursuit Ready. Points: {len(self.waypoints)}")

    def load_waypoints(self, file_path):
        """ Load waypoint từ CSV vào Numpy Array """
        points = []
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                next(reader, None) 
                for row in reader:
                    points.append([float(row[0]), float(row[1])])
        return np.array(points)

    def odom_callback(self, msg):
        if len(self.waypoints) == 0: return

        # 1. Lấy vị trí xe từ Odom
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        
        # Chuyển Quaternion -> Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        car_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 2. Tìm điểm Lookahead (Mục tiêu)
        target_point = self.get_target_point(car_x, car_y)

        # 3. Tính toán và điều khiển
        steering_angle = self.calculate_steering(target_point, car_x, car_y, car_yaw)
        self.publish_drive(steering_angle)
        
        # 4. Hiển thị (Visualization)
        self.publish_dynamic_markers(target_point, car_x, car_y)

    def get_target_point(self, car_x, car_y):
        """ Tìm điểm Lookahead với thuật toán VÒNG LẶP (Looping) """
        num_pts = len(self.waypoints)
        # Chỉ tìm trong 50 điểm gần nhất (có wrap-around %)
        search_len = 50 
        
        # --- Bước 1: Tìm điểm gần nhất (Nearest Neighbor) ---
        # Thay vì cắt mảng [start:end], ta tạo danh sách index xoay vòng
        # Ví dụ: Mảng có 100 điểm, last_idx=98 -> indices = [98, 99, 0, 1, 2...]
        indices = [(self.last_idx + i) % num_pts for i in range(search_len)]
        
        # Lấy tọa độ các điểm tương ứng với indices
        search_points = np.array([self.waypoints[i] for i in indices])
        
        # Tính khoảng cách hàng loạt
        dists = np.linalg.norm(search_points - np.array([car_x, car_y]), axis=1)
        
        # Lấy index của điểm gần nhất trong nhóm search_points
        min_local_idx = np.argmin(dists)
        
        # Map ngược lại index gốc của toàn bộ waypoints
        nearest_idx = indices[min_local_idx]
        self.last_idx = nearest_idx # Lưu lại để lần sau tìm tiếp từ đây
        
        # --- Bước 2: Tìm điểm Lookahead (Cách xe khoảng L) ---
        lookahead_idx = nearest_idx
        while True:
            # Tăng index lên 1, nếu quá độ dài mảng thì quay về 0 (%)
            lookahead_idx = (lookahead_idx + 1) % num_pts
            
            dist = math.dist([car_x, car_y], self.waypoints[lookahead_idx])
            
            # Nếu tìm thấy điểm xa hơn Lookahead distance -> CHỐT
            if dist >= self.L:
                return self.waypoints[lookahead_idx]
            
            # (Phòng hờ) Nếu dò cả vòng mà ko thấy (tránh treo vòng lặp vô tận)
            if lookahead_idx == nearest_idx:
                return self.waypoints[lookahead_idx]

    def calculate_steering(self, target, car_x, car_y, car_yaw):
        """ Chuyển đổi tọa độ và tính góc lái Pure Pursuit """
        tx, ty = target
        dx = tx - car_x
        dy = ty - car_y
        
        # Chuyển điểm mục tiêu về hệ trục tọa độ của xe (Car Frame)
        target_y_local = dx * math.sin(-car_yaw) + dy * math.cos(-car_yaw)
        
        lookahead_dist = math.dist([car_x, car_y], target)
        wheelbase = 0.33 # Chiều dài cơ sở xe f1tenth
        
        # Công thức: delta = atan(2 * L * y / ld^2)
        return math.atan((2 * wheelbase * target_y_local) / (lookahead_dist**2))

    def publish_drive(self, angle):
        speed = self.max_speed
        # Giảm tốc khi cua gấp (> 20 độ)
        if abs(angle) > 0.35: 
            speed *= 0.5 
            
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(msg)

    def stop_vehicle(self):
        """ Gửi lệnh dừng xe (được gọi khi tắt node) """
        self.get_logger().warn("Stopping Vehicle...")
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    # ================= VISUALIZATION =================
    
    def create_marker(self, id, x, y, r, g, b, scale=0.3, type=Marker.SPHERE):
        m = Marker()
        m.header.frame_id = "map"
        m.id = id; m.type = type; m.action = Marker.ADD
        m.pose.position.x = float(x); m.pose.position.y = float(y)
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b
        return m

    def publish_static_path(self):
        if len(self.waypoints) == 0: return
        marker_array = MarkerArray()
        # Dùng SPHERE_LIST vẽ hàng nghìn điểm cực nhẹ
        path_marker = self.create_marker(999, 0, 0, 1.0, 0.0, 0.0, 0.1, Marker.SPHERE_LIST)
        for pt in self.waypoints:
            p = Point(); p.x = pt[0]; p.y = pt[1]
            path_marker.points.append(p)
        marker_array.markers.append(path_marker)
        self.marker_pub.publish(marker_array)

    def publish_dynamic_markers(self, target, car_x, car_y):
        marker_array = MarkerArray()
        # 1. Điểm Lookahead (Xanh lá)
        marker_array.markers.append(self.create_marker(0, target[0], target[1], 0.0, 1.0, 0.0))
        # 2. Vị trí Xe (Xanh dương)
        marker_array.markers.append(self.create_marker(1, car_x, car_y, 0.0, 0.0, 1.0))
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_vehicle()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()