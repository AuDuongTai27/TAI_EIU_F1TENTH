#!/usr/bin/env python3
"""
dagger_inference.py
───────────────────
ROS 2 Node chạy Suy luận mô hình Imitation Learning tích hợp cơ chế DAgger (Cứu nét).

Logic:
  1. Đọc và giải mã dữ liệu LiDAR từ `/scan`, tiền xử lý về 60 beams.
  2. Lấy tọa độ X,Y của xe từ `/odom` để tính toán khoảng cách lệch tâm đường (Cross-track Error - CTE).
  3. Tính toán trước lệnh lái tối ưu của Chuyên gia (Pure Pursuit) theo quỹ đạo chuẩn (waypoints).
  4. Nếu CTE < 0.15m:
     - Dùng AI Model (PyTorch) điều khiển xe chạy tự động (publish tới `/cmd_vel`).
  5. Nếu CTE >= 0.15m:
     - Kích hoạt chế độ cứu nét: Đè lệnh lái của Pure Pursuit lên `/cmd_vel` để cứu xe quay lại quỹ đạo.
     - Thu thập đồng thời dữ liệu [LiDAR hiện tại + Lệnh lái của Chuyên gia] lưu lại vào bộ đệm.
     - Khi bộ đệm gom đủ 50 mẫu, lưu tiếp vào `dagger_dataset.csv` để tái huấn luyện ở vòng DAgger sau.
"""

import os
import csv
import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import PyTorch cho phần suy luận mô hình
try:
    import torch
    import torch.nn as nn
    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False

# --- Định nghĩa kiến trúc mô hình (Cần đồng bộ với train.py) ---
class DAggerMLP(nn.Module):
    def __init__(self, input_dim=60, output_dim=2):
        super(DAggerMLP, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, output_dim)
        )
        
    def forward(self, x):
        return self.network(x)


class DaggerInferenceNode(Node):
    def __init__(self):
        super().__init__('dagger_inference_node')

        # --- 1. Parameters ---
        self.declare_parameter('model_path', os.path.expanduser('~/ros2_ws/src/ros_basic/learning_ros/dagger_model.pth'))
        self.declare_parameter('waypoint_path', os.path.expanduser('~/ros2_ws/install/waypoint/share/waypoint/f1tenth_waypoint_generator/racelines/f1tenth_waypoint.csv'))
        self.declare_parameter('dataset_path', os.path.expanduser('~/ros2_ws/src/ros_basic/learning_ros/dagger_dataset.csv'))
        
        self.declare_parameter('target_beams', 60)
        self.declare_parameter('lookahead_dist', 1.0)
        self.declare_parameter('expert_speed', 0.8)       # Vận tốc của chuyên gia (m/s)
        self.declare_parameter('ai_speed', 0.8)           # Vận tốc tối đa của AI (m/s)
        self.declare_parameter('cte_threshold', 0.15)     # Ngưỡng cứu nét (m)
        self.declare_parameter('buffer_size', 50)
        self.declare_parameter('max_range', 10.0)

        self.model_path = self.get_parameter('model_path').value
        self.waypoint_path = self.get_parameter('waypoint_path').value
        self.dataset_path = self.get_parameter('dataset_path').value
        
        self.target_beams = self.get_parameter('target_beams').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.expert_speed = self.get_parameter('expert_speed').value
        self.ai_speed = self.get_parameter('ai_speed').value
        self.cte_threshold = self.get_parameter('cte_threshold').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.max_range = self.get_parameter('max_range').value

        # --- 2. Load PyTorch Model ---
        self.model = None
        if _HAS_TORCH:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.model = DAggerMLP(input_dim=self.target_beams, output_dim=2).to(self.device)
            if os.path.exists(self.model_path):
                try:
                    self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
                    self.model.eval()
                    self.get_logger().info(f"Successfully loaded PyTorch model from {self.model_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to load model weights: {e}")
            else:
                self.get_logger().warn(f"Model file not found at {self.model_path}. Will ONLY run on Expert mode.")
        else:
            self.get_logger().error("PyTorch is not installed in this environment. Running in EXPERT-ONLY mode.")

        # --- 3. Waypoint & Pure Pursuit States ---
        self.waypoints = self.load_waypoints(self.waypoint_path)
        self.last_idx = 0
        
        # --- 4. State & Buffer Variables ---
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.odom_received = False
        
        self.buffer = []
        self.lock = threading.Lock()
        self.total_saved_samples = 0

        # --- 5. Pub/Sub ---
        # QoS cho odom (hỗ trợ BEST_EFFORT)
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, odom_qos)

        self.get_logger().info("=========================================")
        self.get_logger().info(" DAGGER INFERENCE NODE STARTED")
        self.get_logger().info(f" CTE Threshold: {self.cte_threshold}m")
        self.get_logger().info("=========================================")

    def load_waypoints(self, file_path):
        """Tải các điểm waypoint từ file CSV, nếu không có sẽ tự tạo đường tròn ảo để chạy thử"""
        if os.path.exists(file_path):
            try:
                points = []
                with open(file_path, 'r') as f:
                    reader = csv.reader(f)
                    first_row = next(reader, None)
                    if first_row:
                        try:
                            points.append([float(first_row[0]), float(first_row[1])])
                        except ValueError:
                            pass # Bỏ qua header
                    for row in reader:
                        if len(row) >= 2:
                            points.append([float(row[0]), float(row[1])])
                self.get_logger().info(f"Loaded {len(points)} waypoints from CSV.")
                return np.array(points)
            except Exception as e:
                self.get_logger().error(f"Error loading waypoints: {e}")

        # Fallback: Tạo quỹ đạo hình tròn bán kính 4m tâm tại (0, 3) để test
        self.get_logger().warn("Waypoint file not found! Generating circular fallback waypoints.")
        theta = np.linspace(0, 2*np.pi, 200)
        r = 4.0
        points = np.stack([r * np.cos(theta), r * np.sin(theta) + 3.0], axis=1)
        return points

    def odom_callback(self, msg: Odometry):
        """Cập nhật vị trí hiện tại của xe và góc quay (yaw)"""
        with self.lock:
            self.car_x = msg.pose.pose.position.x
            self.car_y = msg.pose.pose.position.y
            
            # Quaternion -> Yaw
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.car_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        """Xử lý LiDAR, tính sai lệch CTE, quyết định điều khiển qua AI hoặc Expert"""
        with self.lock:
            if not self.odom_received:
                return
            curr_x = self.car_x
            curr_y = self.car_y
            curr_yaw = self.car_yaw

        # 1. Tính toán sai lệch khoảng cách (Cross-track Error - CTE) đến quỹ đạo
        cte, nearest_idx = self.calculate_cross_track_error(curr_x, curr_y)

        # 2. Tính toán lệnh lái dự phòng từ Chuyên gia (Pure Pursuit)
        expert_linear, expert_angular = self.calculate_pure_pursuit(curr_x, curr_y, curr_yaw, nearest_idx)

        # 3. Tiền xử lý dữ liệu scan
        preprocessed_scan = self.preprocess_scan(msg)

        # 4. Trạng thái cứu nét hoặc AI
        if self.model is None or cte >= self.cte_threshold:
            # --- CHẾ ĐỘ CHUYÊN GIA CỨU NÉT (OVERRIDE) ---
            self.publish_cmd(expert_linear, expert_angular)
            self.get_logger().warn(f"[EXPERT OVERRIDE] CTE: {cte:.3f}m >= {self.cte_threshold}m. Expert driving.", throttle_duration_sec=1.0)
            
            # Lưu lại dữ liệu lúc cứu nét (DAgger data aggregation)
            with self.lock:
                row = list(preprocessed_scan) + [expert_linear, expert_angular]
                self.buffer.append(row)
                
                if len(self.buffer) >= self.buffer_size:
                    buffer_to_save = list(self.buffer)
                    self.buffer.clear()
                    threading.Thread(target=self._flush_buffer, args=(buffer_to_save,), daemon=True).start()
        else:
            # --- CHẾ ĐỘ AI SUY LUẬN TỰ ĐỘNG ---
            ai_linear, ai_angular = self.run_model_inference(preprocessed_scan)
            self.publish_cmd(ai_linear, ai_angular)
            self.get_logger().info(f"[AI DRIVING] CTE: {cte:.3f}m < {self.cte_threshold}m.", throttle_duration_sec=2.0)

    def calculate_cross_track_error(self, car_x, car_y):
        """
        Tính khoảng cách ngắn nhất (vuông góc) từ xe tới đường đi.
        Tìm đoạn thẳng (segment) nối giữa 2 waypoint gần nhất và tính khoảng cách từ xe tới nó.
        """
        num_pts = len(self.waypoints)
        search_len = min(50, num_pts)
        
        # Chỉ quét tìm kiếm trong phạm vi 50 điểm lân cận vị trí trước đó
        indices = [(self.last_idx + i) % num_pts for i in range(search_len)]
        search_points = self.waypoints[indices]
        
        dists = np.linalg.norm(search_points - np.array([car_x, car_y]), axis=1)
        min_local_idx = np.argmin(dists)
        nearest_idx = indices[min_local_idx]
        self.last_idx = nearest_idx

        # Lấy 2 điểm tạo thành đoạn thẳng quỹ đạo chuẩn
        A = self.waypoints[nearest_idx]
        B = self.waypoints[(nearest_idx + 1) % num_pts]
        P = np.array([car_x, car_y])

        AB = B - A
        AP = P - A
        ab_len_sq = np.sum(AB**2)

        if ab_len_sq < 1e-6:
            cte = np.linalg.norm(AP)
        else:
            # Tính hình chiếu của P lên đoạn thẳng AB
            t = np.clip(np.dot(AP, AB) / ab_len_sq, 0.0, 1.0)
            closest_pt = A + t * AB
            cte = np.linalg.norm(P - closest_pt)

        return cte, nearest_idx

    def calculate_pure_pursuit(self, car_x, car_y, car_yaw, nearest_idx):
        """Thuật toán Pure Pursuit để tính góc lái cứu nguy"""
        num_pts = len(self.waypoints)
        lookahead_idx = nearest_idx
        
        # Tìm điểm đích cách xe một khoảng lookahead_dist
        while True:
            lookahead_idx = (lookahead_idx + 1) % num_pts
            dist = math.hypot(car_x - self.waypoints[lookahead_idx][0], car_y - self.waypoints[lookahead_idx][1])
            if dist >= self.lookahead_dist:
                target_pt = self.waypoints[lookahead_idx]
                break
            if lookahead_idx == nearest_idx:
                target_pt = self.waypoints[lookahead_idx]
                break

        # Chuyển điểm đích sang hệ tọa độ của xe (Car Frame)
        dx = target_pt[0] - car_x
        dy = target_pt[1] - car_y
        local_x = dx * math.cos(car_yaw) + dy * math.sin(car_yaw)
        local_y = -dx * math.sin(car_yaw) + dy * math.cos(car_yaw)

        # Tính độ cong quỹ đạo kappa = 2 * local_y / ld^2
        kappa = (2.0 * local_y) / (self.lookahead_dist**2)

        # Vận tốc của chuyên gia và tốc độ góc tương ứng
        linear_v = self.expert_speed
        angular_z = linear_v * kappa

        return linear_v, angular_z

    def preprocess_scan(self, msg: LaserScan):
        """Crop góc quét về [-60, 60] độ và downsample về 60 beams"""
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        crop_limit = math.radians(60.0)

        angles = np.arange(len(ranges)) * angle_increment + angle_min
        mask = (angles >= -crop_limit) & (angles <= crop_limit)

        if not np.any(mask):
            return np.ones(self.target_beams, dtype=np.float32) * self.max_range

        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        valid_ranges = np.where(np.isnan(valid_ranges) | np.isinf(valid_ranges), self.max_range, valid_ranges)
        valid_ranges = np.clip(valid_ranges, 0.0, self.max_range)

        target_angles = np.linspace(-crop_limit, crop_limit, self.target_beams)
        return np.interp(target_angles, valid_angles, valid_ranges)

    def run_model_inference(self, preprocessed_scan):
        """Dự đoán lệnh lái qua PyTorch Model"""
        if not _HAS_TORCH or self.model is None:
            return 0.0, 0.0
            
        with torch.no_grad():
            tensor_input = torch.tensor(preprocessed_scan, dtype=torch.float32).unsqueeze(0).to(self.device)
            output = self.model(tensor_input).cpu().squeeze(0).numpy()
            
        # Output: [linear_v, angular_z]
        linear_v = float(np.clip(output[0], 0.0, self.ai_speed))
        angular_z = float(output[1])
        return linear_v, angular_z

    def publish_cmd(self, linear_v, angular_z):
        """Publish lệnh tới topic /cmd_vel"""
        msg = Twist()
        msg.linear.x = float(linear_v)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _flush_buffer(self, data_list):
        """Ghi dữ liệu Expert cứu nét vào dataset CSV"""
        # Nếu file chưa tồn tại thì ghi header trước
        if not os.path.exists(self.dataset_path):
            with open(self.dataset_path, 'w', newline='') as f:
                writer = csv.writer(f)
                header = [f'lidar_{i}' for i in range(self.target_beams)] + ['linear_v', 'angular_z']
                writer.writerow(header)

        try:
            with open(self.dataset_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(data_list)
            
            self.total_saved_samples += len(data_list)
            self.get_logger().info(f"[DAgger Data Collector] Logged {len(data_list)} expert recovery samples. Total: {self.total_saved_samples}")
        except Exception as e:
            self.get_logger().error(f"Error saving DAgger dataset: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DaggerInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Shutting down DAgger inference node.")
        node.publish_cmd(0.0, 0.0)
        if len(node.buffer) > 0:
            node._flush_buffer(node.buffer)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
