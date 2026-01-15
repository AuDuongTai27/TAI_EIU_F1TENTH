#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv
import os
import time
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

# Định nghĩa các trạng thái
STATE_GLOBAL_TRACKING = 0
STATE_WAITING_PLAN = 1
STATE_LOCAL_TRACKING = 2

class SmartController(Node):
    def __init__(self):
        super().__init__('smart_controller')
        
        # --- Config ---
        # 1. Đường dẫn Global (Đường vẽ sẵn)
        self.global_csv = "/home/adt/ros2_ws/install/waypoint/share/waypoint/f1tenth_waypoint_generator/racelines/f1tenth_waypoint.csv" 
        # 2. Đường dẫn Local (Do RRT sinh ra tạm thời)
        self.local_csv = os.path.expanduser('~/rrt_temp_path.csv')
        
        self.waypoints = self.load_waypoints(self.global_csv)
        self.state = STATE_GLOBAL_TRACKING
        self.current_path_type = "GLOBAL"
        self.last_idx = 0
        
        # Vật cản
        self.obstacle_pos = None
        self.detection_dist = 2.0  # Khoảng cách phát hiện vật cản (m)
        self.rejoin_lookahead = 4.0 # Khoảng cách điểm mục tiêu sau vật cản (m)

        # --- Pub/Sub ---
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.replan_pub = self.create_publisher(Path, '/trigger_replan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/markers', 10)
        
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.create_subscription(Marker, '/sim_obstacle', self.obstacle_callback, 10)
        self.create_subscription(Bool, '/replan_success', self.replan_success_callback, 10)
        
        self.get_logger().info(f"Smart Controller Started. Mode: {self.current_path_type}")

    def load_waypoints(self, file_path):
        points = []
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    # Bỏ qua dòng trống hoặc dòng thiếu dữ liệu
                    if not row or len(row) < 2:
                        continue
                    
                    try:
                        # Cố gắng chuyển đổi sang số float
                        x = float(row[0])
                        y = float(row[1])
                        points.append([x, y])
                    except ValueError:
                        # Nếu lỗi (do gặp chữ 'x', 'y' ở header), bỏ qua dòng này
                        continue
                        
        if len(points) == 0:
            self.get_logger().error(f"Cannot load any valid waypoints from {file_path}")
            return np.array([])
            
        return np.array(points)
    
    def obstacle_callback(self, msg):
        # Cập nhật vị trí vật cản từ Simulator
        if msg.action == Marker.ADD:
            self.obstacle_pos = [msg.pose.position.x, msg.pose.position.y]
        else:
            self.obstacle_pos = None

    def replan_success_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received New Path! Switching to LOCAL TRACKING.")
            # Load path mới từ file temp
            self.waypoints = self.load_waypoints(self.local_csv)
            self.last_idx = 0
            self.state = STATE_LOCAL_TRACKING
            self.current_path_type = "LOCAL"

    def odom_callback(self, msg):
        if len(self.waypoints) == 0: return
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        
        # --- LOGIC XỬ LÝ THEO TRẠNG THÁI ---
        
        # 1. Nếu đang chạy Global -> Kiểm tra vật cản
        if self.state == STATE_GLOBAL_TRACKING:
            if self.check_collision_ahead(car_x, car_y):
                self.trigger_replan(car_x, car_y)
                return # Dừng xử lý frame này

        # 2. Nếu đang đợi RRT -> Dừng xe
        elif self.state == STATE_WAITING_PLAN:
            self.publish_drive(0.0, 0.0) # Phanh gấp
            return

        # 3. Nếu đang chạy Local -> Kiểm tra xem đã hết đường chưa
        elif self.state == STATE_LOCAL_TRACKING:
            dist_to_end = math.dist([car_x, car_y], self.waypoints[-1])
            if dist_to_end < 0.5: # Đã đến cuối đường Local
                self.get_logger().info("Avoidance Complete. Switching back to GLOBAL.")
                self.waypoints = self.load_waypoints(self.global_csv)
                
                # Tìm lại index gần nhất trên đường Global để tiếp tục
                dists = np.linalg.norm(self.waypoints - np.array([car_x, car_y]), axis=1)
                self.last_idx = np.argmin(dists)
                
                self.state = STATE_GLOBAL_TRACKING
                self.current_path_type = "GLOBAL"
                
                # Xóa file temp cho sạch sẽ
                if os.path.exists(self.local_csv): os.remove(self.local_csv)

        # --- PURE PURSUIT (CHẠY XE) ---
        target = self.get_target_point(car_x, car_y)
        
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        
        steering = self.calculate_steering(target, car_x, car_y, yaw)
        self.publish_drive(2.0 if self.state == STATE_GLOBAL_TRACKING else 1.0, steering)
        self.publish_markers(target, car_x, car_y)

    def check_collision_ahead(self, car_x, car_y):
        """ Kiểm tra xem vật cản có nằm chắn đường phía trước không """
        if self.obstacle_pos is None: return False
        
        # Khoảng cách tới vật cản
        dist = math.dist([car_x, car_y], self.obstacle_pos)
        
        # Nếu vật cản ở xa > detection_dist -> Kệ nó
        if dist > self.detection_dist: return False
        
        # Nếu vật cản ở gần, kiểm tra xem nó có nằm TRÊN đường path không
        # (Đơn giản hóa: Kiểm tra lookahead point hiện tại có gần vật cản không)
        target = self.get_target_point(car_x, car_y)
        dist_obs_path = math.dist(target, self.obstacle_pos)
        
        if dist_obs_path < 1.0: # Nếu điểm sắp tới nằm gần vật cản
            self.get_logger().warn(f"Obstacle Detected ahead! Dist: {dist:.2f}")
            return True
        return False

    def trigger_replan(self, car_x, car_y):
        self.state = STATE_WAITING_PLAN
        self.get_logger().info("Requesting RRT* Replan...")
        self.publish_drive(0.0, 0.0) # Dừng xe
        
        # 1. Điểm Start: Vị trí hiện tại
        start_pose = PoseStamped()
        start_pose.pose.position.x = car_x
        start_pose.pose.position.y = car_y
        
        # 2. Điểm Goal: Một điểm trên đường Global, nằm SAU vật cản một đoạn
        # Ta lấy điểm hiện tại + lookahead xa (rejoin_lookahead)
        future_idx = (self.last_idx + int(self.rejoin_lookahead * 10)) % len(self.waypoints)
        goal_point = self.waypoints[future_idx]
        
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = goal_point[0]
        goal_pose.pose.position.y = goal_point[1]
        
        # Gửi request
        msg = Path()
        msg.poses = [start_pose, goal_pose]
        self.replan_pub.publish(msg)

    # --- Pure Pursuit Helpers (Giữ nguyên) ---
    def get_target_point(self, car_x, car_y):
        indices = [(self.last_idx + i) % len(self.waypoints) for i in range(50)]
        search_pts = self.waypoints[indices]
        dists = np.linalg.norm(search_pts - np.array([car_x, car_y]), axis=1)
        self.last_idx = indices[np.argmin(dists)]
        
        lookahead_idx = self.last_idx
        for _ in range(len(self.waypoints)):
            lookahead_idx = (lookahead_idx + 1) % len(self.waypoints)
            if math.dist([car_x, car_y], self.waypoints[lookahead_idx]) >= 1.0:
                return self.waypoints[lookahead_idx]
        return self.waypoints[self.last_idx]

    def calculate_steering(self, target, car_x, car_y, car_yaw):
        dx = target[0] - car_x; dy = target[1] - car_y
        yt = dx * math.sin(-car_yaw) + dy * math.cos(-car_yaw)
        ld = math.dist([car_x, car_y], target)
        return math.atan((2 * 0.33 * yt) / (ld**2))

    def publish_drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(msg)

    def publish_markers(self, target, car_x, car_y):
        ma = MarkerArray()
        # Marker Xanh Lá: Local Path, Đỏ: Global Path
        color_g = 1.0 if self.current_path_type == "GLOBAL" else 0.0
        color_r = 1.0 if self.current_path_type == "LOCAL" else 0.0
        
        # Target Point
        m = Marker()
        m.header.frame_id = "map"; m.id = 0; m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = target[0]; m.pose.position.y = target[1]
        m.scale.x=0.3; m.scale.y=0.3; m.scale.z=0.3
        m.color.a=1.0; m.color.r=color_r; m.color.g=color_g; m.color.b=0.0
        ma.markers.append(m)
        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = SmartController()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()