#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration # <--- Cần thêm thư viện này cho Duration
import numpy as np
import math
import csv
import os

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
        self.base_frame = self.get_parameter("base_frame").value # Lưu ý biến này

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
        self.get_logger().info(f"REAL CAR READY. Wheelbase: {self.wheelbase}, Speed: {self.max_speed}")

    # ... (Các hàm load_waypoints, remove_overlapped_waypoints giữ nguyên) ...
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

    # =========================================================
    # ĐÂY LÀ CHỖ BẠN CẦN THAY THẾ (HÀM control_loop)
    # =========================================================
    def control_loop(self, msg):
        if len(self.waypoints) == 0: return

        # --- BƯỚC 1: LẤY VỊ TRÍ (Đã thay đoạn code của bạn vào đây) ---
        try:
            # Hỏi TF: "Base_link đang ở đâu trên Map?"
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_frame,  # Dùng self.base_frame cho đồng bộ với __init__
                rclpy.time.Time(), # Lấy thời gian mới nhất
                # Duration(seconds=0.1) # Có thể thêm timeout nếu muốn
            )
            
            # 1. Lấy tọa độ X, Y
            car_x = transform.transform.translation.x
            car_y = transform.transform.translation.y

            # 2. QUAN TRỌNG: Phải tính Yaw (Góc lái) thì Pure Pursuit mới chạy được
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            car_yaw = math.atan2(siny_cosp, cosy_cosp)

            # 3. Gọi hàm hiển thị riêng của bạn (nếu có)
            self.publish_vi_tri_hien_tai(car_x, car_y)
            self.publish_ten_xe(car_x, car_y)
            
        except TransformException as e:
            # Nếu chưa có TF map->base_link thì return luôn
            return

        # --- BƯỚC 2: THUẬT TOÁN PURE PURSUIT ---
        target_point = self.get_target_point(car_x, car_y)
        steering_angle = self.calculate_steering(target_point, car_x, car_y, car_yaw)
        
        # --- BƯỚC 3: GỬI LỆNH ---
        self.publish_drive(steering_angle)
        
        # --- BƯỚC 4: VISUALIZATION ---
        self.publish_dynamic_markers(target_point, car_x, car_y)

    # --- CÁC HÀM BỔ SUNG (Placeholder để không lỗi) ---
    def publish_vi_tri_hien_tai(self, x, y):
        # Bạn viết code publish text marker hoặc log ở đây
        pass 

    def publish_ten_xe(self, x, y):
        # Bạn viết code publish tên xe ở đây
        pass

    # ... (Các hàm get_target_point, calculate_steering, publish_drive, visualization giữ nguyên) ...
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

    def publish_drive(self, angle):
        speed = self.max_speed
        angle = max(min(angle, self.steering_limit), -self.steering_limit)
        if abs(angle) > 0.17: 
            speed *= 1.0  
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(msg)

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