#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv
import os
import copy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ReactivePlanner(Node):
    def __init__(self):
        super().__init__('reactive_planner')

        # --- CONFIG ---
        self.declare_parameter("waypoint_path", "/home/adt/ros2_ws/install/waypoint/share/waypoint/f1tenth_waypoint_generator/racelines/f1tenth_waypoint.csv")
        self.csv_path = self.get_parameter("waypoint_path").value
        
        # Các thông số Plannerth
        self.lookahead_dist = 1.5       # Khoảng cách nhìn xa để tạo đường cong
        self.nudge_width = 0.5          # Khoảng cách giữa các đường sinh ra (m)
        self.num_paths = 7              # Số lượng đường sinh ra (Lẻ: 1 giữa, còn lại chia đều 2 bên)
        self.obstacle_radius = 0.6      # Bán kính an toàn vật cản
        self.max_speed = 2.0
        
        # --- VARIABLES ---
        self.global_waypoints = self.load_waypoints(self.csv_path)
        self.current_local_path = []    # Đường xe đang chạy theo
        self.obstacle_pos = None        # Vị trí vật cản
        self.last_selected_offset_idx = (self.num_paths // 2) # Mặc định chọn đường giữa (index 3 nếu có 7 đường)
        self.lock_counter = 0           # Biến đếm để chống nhiễu (giữ đường né một lúc)

        # --- PUB/SUB ---
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.vis_pub = self.create_publisher(MarkerArray, '/visualization/markers', 10)
        
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.create_subscription(Marker, '/sim_obstacle', self.obstacle_callback, 10)
        
        self.get_logger().info("Reactive Planner Started. Ready to swerve!")

    def load_waypoints(self, file_path):
        points = []
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row or len(row) < 2: continue
                    try: points.append([float(row[0]), float(row[1])])
                    except: continue
        return np.array(points)

    def obstacle_callback(self, msg):
        if msg.action == Marker.ADD:
            self.obstacle_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        else:
            self.obstacle_pos = None

    def odom_callback(self, msg):
        if len(self.global_waypoints) == 0: return

        # 1. Lấy trạng thái xe
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        car_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

        # 2. Tìm điểm mục tiêu trên Global Path (Reference Point)
        # Điểm này sẽ là tâm để ta sinh ra các đường cong
        ref_idx = self.get_closest_index(car_x, car_y)
        lookahead_idx = self.get_lookahead_index(ref_idx, self.lookahead_dist)
        ref_point = self.global_waypoints[lookahead_idx]
        
        # Tính hướng tiếp tuyến của đường Global tại điểm đó (để sinh đường song song)
        next_idx = (lookahead_idx + 1) % len(self.global_waypoints)
        prev_idx = (lookahead_idx - 1) % len(self.global_waypoints)
        dx = self.global_waypoints[next_idx][0] - self.global_waypoints[prev_idx][0]
        dy = self.global_waypoints[next_idx][1] - self.global_waypoints[prev_idx][1]
        path_yaw = math.atan2(dy, dx)

        # 3. SINH ĐƯỜNG (CANDIDATE GENERATION)
        candidates = [] # List các đường dẫn [[(x,y), (x,y)...], ...]
        offsets = []    # List các độ lệch [-1.5, -1.0, -0.5, 0, 0.5 ...]
        
        # Vector pháp tuyến (vuông góc) để dịch chuyển điểm mục tiêu sang trái/phải
        norm_x = -math.sin(path_yaw)
        norm_y = math.cos(path_yaw)

        # Tạo chùm tia
        center_idx = self.num_paths // 2
        for i in range(self.num_paths):
            offset = (i - center_idx) * self.nudge_width # Ví dụ: -1.0, -0.5, 0.0, 0.5, 1.0
            
            # Điểm đích tạm thời của tia này (Target Offset)
            target_x = ref_point[0] + norm_x * offset
            target_y = ref_point[1] + norm_y * offset
            
            # Sinh đường cong Cubic Bezier hoặc Spline đơn giản từ Xe -> Target Offset
            path = self.generate_cubic_path(car_x, car_y, car_yaw, target_x, target_y, path_yaw)
            candidates.append(path)
            offsets.append(i)

        # 4. CHỌN ĐƯỜNG TỐT NHẤT (COST FUNCTION)
        best_idx = self.select_best_path(candidates, offsets)
        
        # Logic khóa đường (Chống nhiễu):
        # Nếu đang né (best_idx != center), giữ nó ít nhất 10 frame trừ khi bị chặn
        if best_idx != center_idx:
            self.lock_counter = 10
        elif self.lock_counter > 0:
            self.lock_counter -= 1
            # Nếu đường cũ vẫn an toàn, ưu tiên giữ lại đường cũ (tránh nhảy về center quá sớm)
            if not self.check_collision(candidates[self.last_selected_offset_idx]):
                 best_idx = self.last_selected_offset_idx

        self.last_selected_offset_idx = best_idx
        self.current_local_path = candidates[best_idx]

        # 5. ĐIỀU KHIỂN (PURE PURSUIT TRÊN LOCAL PATH)
        # Lấy điểm khoảng giữa đường Local để lái theo
        follow_point = self.current_local_path[len(self.current_local_path)//2]
        steering = self.calculate_steering(follow_point, car_x, car_y, car_yaw)
        
        # Giảm tốc nếu cua gắt hoặc đang né vật cản
        speed = self.max_speed
        if best_idx != center_idx: speed *= 0.6 
        
        self.publish_drive(speed, steering)
        self.visualize(candidates, best_idx, ref_point, car_x, car_y)

    def generate_cubic_path(self, x0, y0, yaw0, x1, y1, yaw1):
        """ Sinh đường cong mượt nối 2 điểm (dạng tham số) """
        dist = math.hypot(x1-x0, y1-y0)
        steps = 10
        path = []
        for i in range(steps + 1):
            s = i / steps
            # Công thức nội suy đơn giản để tạo đường cong chữ S
            # (Thực tế có thể dùng Bezier bậc 3 chuẩn hơn, nhưng cái này đủ nhanh)
            # Nội suy tuyến tính tọa độ nhưng thêm trọng số hướng
            
            # Cách đơn giản nhất cho mô phỏng: Chia đoạn thẳng và làm mượt
            # Ở đây dùng nội suy tuyến tính + offset theo hình sin để tạo độ cong
            # P(t) = P0 + (P1-P0)*t
            # Nhưng để xe đi đúng hướng, ta dùng mô hình đường cong Dubins giả lập hoặc Spline
            
            # Dùng Bezier bậc 3:
            # P0: Xe, P3: Đích
            # P1: Xe + hướng xe * dist/2
            # P2: Đích - hướng đích * dist/2
            p0x, p0y = x0, y0
            p3x, p3y = x1, y1
            p1x, p1y = x0 + math.cos(yaw0) * dist/2, y0 + math.sin(yaw0) * dist/2
            p2x, p2y = x1 - math.cos(yaw1) * dist/2, y1 - math.sin(yaw1) * dist/2
            
            bx = (1-s)**3*p0x + 3*(1-s)**2*s*p1x + 3*(1-s)*s**2*p2x + s**3*p3x
            by = (1-s)**3*p0y + 3*(1-s)**2*s*p1y + 3*(1-s)*s**2*p2y + s**3*p3y
            path.append([bx, by])
            
        return np.array(path)

    def select_best_path(self, candidates, offsets):
        """ Chọn đường: Không va chạm + Gần đường giữa nhất """
        center_idx = self.num_paths // 2
        valid_indices = []
        
        # 1. Lọc va chạm
        for i, path in enumerate(candidates):
            if not self.check_collision(path):
                valid_indices.append(i)
        
        if not valid_indices:
            self.get_logger().warn("ALL PATHS BLOCKED! STOPPING.")
            return center_idx # Vẫn trả về đường giữa nhưng xe sẽ dừng ở logic ngoài (nếu muốn)
            
        # 2. Chọn đường có chi phí thấp nhất
        # Cost = |offset_index - center_index| (Ưu tiên đường thẳng)
        # Cost += Hysteresis (Ưu tiên đường đang chọn cũ)
        best_cost = float('inf')
        best_idx = valid_indices[0]
        
        for idx in valid_indices:
            cost = abs(idx - center_idx) * 1.0 # Trọng số lệch tâm
            
            # Trọng số quán tính (giúp giữ đường né, chống nhiễu)
            if idx == self.last_selected_offset_idx:
                cost -= 0.5 
                
            if cost < best_cost:
                best_cost = cost
                best_idx = idx
                
        return best_idx

    def check_collision(self, path):
        if self.obstacle_pos is None: return False
        # Kiểm tra từng điểm trên path có quá gần vật cản không
        for pt in path:
            dist = math.hypot(pt[0] - self.obstacle_pos[0], pt[1] - self.obstacle_pos[1])
            if dist < self.obstacle_radius:
                return True
        return False

    def get_closest_index(self, x, y):
        dists = np.linalg.norm(self.global_waypoints - np.array([x, y]), axis=1)
        return np.argmin(dists)

    def get_lookahead_index(self, start_idx, dist):
        idx = start_idx
        accumulated_dist = 0.0
        while accumulated_dist < dist:
            curr = self.global_waypoints[idx]
            idx = (idx + 1) % len(self.global_waypoints)
            next_pt = self.global_waypoints[idx]
            accumulated_dist += math.hypot(next_pt[0]-curr[0], next_pt[1]-curr[1])
            if idx == start_idx: break # Tránh vòng lặp vô tận
        return idx

    def calculate_steering(self, target, car_x, car_y, car_yaw):
        dx = target[0] - car_x
        dy = target[1] - car_y
        target_y_local = dx * math.sin(-car_yaw) + dy * math.cos(-car_yaw)
        lookahead_dist = math.hypot(dx, dy)
        wheelbase = 0.33
        return math.atan((2 * wheelbase * target_y_local) / (lookahead_dist**2))

    def publish_drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(msg)

    def visualize(self, candidates, best_idx, ref_point, cx, cy):
        ma = MarkerArray()
        
        # 1. Global Path (Đỏ - Tham khảo) - Vẽ ít điểm thôi cho nhẹ
        # (Ở đây ta không vẽ lại toàn bộ global path mỗi frame vì nặng, chỉ vẽ local thôi)

        # 2. Candidate Paths (Xám)
        for i, path in enumerate(candidates):
            m = Marker()
            m.header.frame_id = "map"; m.id = i; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.02 # Mảnh
            if i == best_idx:
                m.color.r=0.0; m.color.g=1.0; m.color.b=0.0; m.color.a=1.0 # Xanh lá (Đường chọn)
                m.scale.x = 0.08 # Dày hơn
            else:
                m.color.r=0.7; m.color.g=0.7; m.color.b=0.7; m.color.a=0.5 # Xám (Ứng viên)
            
            for pt in path:
                p = Point(); p.x=pt[0]; p.y=pt[1]
                m.points.append(p)
            ma.markers.append(m)

        # 3. Lookahead/Target Point (Xanh dương)
        mt = Marker()
        mt.header.frame_id = "map"; mt.id = 100; mt.type = Marker.SPHERE; mt.action = Marker.ADD
        mt.pose.position.x = ref_point[0]; mt.pose.position.y = ref_point[1]
        mt.scale.x=0.3; mt.scale.y=0.3; mt.scale.z=0.3
        mt.color.b=1.0; mt.color.a=1.0
        ma.markers.append(mt)
        
        self.vis_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = ReactivePlanner()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()