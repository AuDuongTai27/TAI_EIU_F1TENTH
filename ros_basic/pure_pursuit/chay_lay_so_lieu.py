#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import math
import csv
import os
import time  # <--- MỚI: Dùng để tính giờ
from datetime import datetime # <--- MỚI: Để đặt tên file log

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformException

class PurePursuitReal(Node):
    def __init__(self):
        super().__init__('pure_pursuit_real_node')
        
        # ==========================================
        # 1. PARAMETERS
        # ==========================================
        self.declare_parameter("waypoint_path", "/home/adt/ros2_ws/install/waypoint/share/waypoint/f1tenth_waypoint_generator/racelines/f1tenth_waypoint.csv")
        self.declare_parameter("lookahead_dist", 0.8)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("wheelbase", 0.39)
        self.declare_parameter("steering_limit", 0.35)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.csv_path = self.get_parameter("waypoint_path").value
        self.L = self.get_parameter("lookahead_dist").value
        self.max_speed = self.get_parameter("max_speed").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.steering_limit = self.get_parameter("steering_limit").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # ==========================================
        # 2. INIT
        # ==========================================
        self.waypoints = self.load_waypoints(self.csv_path)
        self.last_idx = 0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/markers', 10)
        self.create_subscription(Odometry, '/odom', self.control_loop, 10) 
        
        self.publish_static_path()

        # --- [MỚI] SETUP LOGGER & HZ COUNTER ---
        # 1. Setup đo Hz
        self.loop_count = 0
        self.last_print_time = time.time()
        
        # 2. Setup ghi file CSV (Lưu vào thư mục Home)
        home_dir = os.path.expanduser('~')
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_filename = os.path.join(home_dir, f"f1tenth_log_{timestamp}.csv")
        
        self.log_file = open(self.log_filename, 'w', newline='')
        self.writer = csv.writer(self.log_file)
        # Ghi header cho file CSV
        self.writer.writerow(["time", "cte", "steering_angle", "speed", "x", "y"])
        self.start_time = time.time()
        
        self.get_logger().info(f"REAL CAR READY. Logging to: {self.log_filename}")

    def load_waypoints(self, file_path):
        points = []
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                next(reader, None) 
                for row in reader:
                    points.append([float(row[0]), float(row[1])])
        else:
            self.get_logger().error(f"File not found: {file_path}")
        return self.remove_overlapped_waypoints(points, threshold=0.5)
    
    def remove_overlapped_waypoints(self, waypoints_list, threshold=0.1):
        if not waypoints_list: return []
        cleaned_waypoints_list = [waypoints_list[0]]
        for i in range(1, len(waypoints_list)):
            x1, y1 = cleaned_waypoints_list[-1]
            x2, y2 = waypoints_list[i]        
            if self.distance_between_2points(x1,y1, x2, y2) >= threshold:
                cleaned_waypoints_list.append(waypoints_list[i])
        return cleaned_waypoints_list

    def distance_between_2points(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # ================= CORE LOOP =================
    def control_loop(self, msg):
        if len(self.waypoints) == 0: return

        # --- [MỚI] TÍNH HZ ---
        self.loop_count += 1
        now = time.time()
        if now - self.last_print_time >= 1.0: # Mỗi 1 giây in 1 lần
            hz = self.loop_count / (now - self.last_print_time)
            self.get_logger().info(f"Loop Rate: {hz:.2f} Hz") # <--- Lấy số này điền vào báo cáo
            self.loop_count = 0
            self.last_print_time = now

        # --- BƯỚC 1: TF LOOKUP ---
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_frame, 
                rclpy.time.Time()
            )
            car_x = transform.transform.translation.x
            car_y = transform.transform.translation.y
            
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            car_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except TransformException as e:
            return

        # --- BƯỚC 2: PURE PURSUIT ---
        target_point = self.get_target_point(car_x, car_y)
        steering_angle = self.calculate_steering(target_point, car_x, car_y, car_yaw)
        
        # --- BƯỚC 3: ACTUATION & LOGGING ---
        # Tính tốc độ thực tế gửi đi (để log cho đúng)
        final_speed = self.max_speed
        safe_angle = max(min(steering_angle, self.steering_limit), -self.steering_limit)
        if abs(safe_angle) > 0.17: 
            final_speed *= 1.0

        self.publish_drive(safe_angle, final_speed)
        self.publish_dynamic_markers(target_point, car_x, car_y)

        # --- [MỚI] TÍNH TOÁN DỮ LIỆU ĐỂ LOG ---
        # 1. Tính CTE (Cross Track Error): Khoảng cách từ xe đến waypoint gần nhất
        # self.last_idx đã được cập nhật trong hàm get_target_point
        nearest_wp = self.waypoints[self.last_idx]
        current_cte = math.dist([car_x, car_y], nearest_wp)
        
        # 2. Ghi vào file
        log_time = time.time() - self.start_time
        self.writer.writerow([f"{log_time:.3f}", f"{current_cte:.4f}", f"{safe_angle:.3f}", f"{final_speed:.2f}", f"{car_x:.3f}", f"{car_y:.3f}"])

    def get_target_point(self, car_x, car_y):
        num_pts = len(self.waypoints)
        search_len = 50 
        indices = [(self.last_idx + i) % num_pts for i in range(search_len)]
        search_points = np.array([self.waypoints[i] for i in indices])
        dists = np.linalg.norm(search_points - np.array([car_x, car_y]), axis=1)
        nearest_idx = indices[np.argmin(dists)]
        self.last_idx = nearest_idx
        
        lookahead_idx = nearest_idx
        while True:
            lookahead_idx = (lookahead_idx + 1) % num_pts
            dist = math.dist([car_x, car_y], self.waypoints[lookahead_idx])
            if dist >= self.L or lookahead_idx == nearest_idx:
                return self.waypoints[lookahead_idx]

    def calculate_steering(self, target, car_x, car_y, car_yaw):
        tx, ty = target
        dx = tx - car_x
        dy = ty - car_y
        target_y_local = dx * math.sin(-car_yaw) + dy * math.cos(-car_yaw)
        lookahead_dist = math.dist([car_x, car_y], target)
        return math.atan((2 * self.wheelbase * target_y_local) / (lookahead_dist**2))

    # Đã sửa hàm này để nhận tham số speed và angle đã xử lý
    def publish_drive(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)  
        msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(msg)

    # ... (Các hàm create_marker, publish_static_path... giữ nguyên) ...
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
        path_marker = self.create_marker(999, 0, 0, 1.0, 0.0, 0.0, 0.1, Marker.SPHERE_LIST)
        for pt in self.waypoints:
            p = Point(); p.x = pt[0]; p.y = pt[1]
            path_marker.points.append(p)
        marker_array.markers.append(path_marker)
        self.marker_pub.publish(marker_array)

    def publish_dynamic_markers(self, target, car_x, car_y):
        marker_array = MarkerArray()
        marker_array.markers.append(self.create_marker(0, target[0], target[1], 0.0, 1.0, 0.0))
        marker_array.markers.append(self.create_marker(1, car_x, car_y, 0.0, 0.0, 1.0))
        self.marker_pub.publish(marker_array)
    
    # [MỚI] Hàm dọn dẹp khi tắt node
    def destroy_node(self):
        self.log_file.close()
        self.get_logger().info("Log file closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()