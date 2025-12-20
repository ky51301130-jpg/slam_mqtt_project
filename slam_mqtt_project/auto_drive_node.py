#!/usr/bin/env python3
"""SLAM 탐색 자율주행 노드 (PORT 도킹 버전)

기능:
- LiDAR + 초음파 기반 장애물 회피
- 탐색 중 PORT_A 마커 발견 시 도킹
- 측정된 X,Y,Z 값으로 정확한 도킹 위치 진입
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
import math
import json
import os
from collections import deque

try:
    from pinkylib import Ultrasonic
    ULTRASONIC_AVAILABLE = True
except ImportError:
    ULTRASONIC_AVAILABLE = False

from slam_mqtt_project.topics import ROS, ARUCO, DRIVE

# ====== topics.py에서 가져온 상수 (편의용 alias) ======
# 거리 임계값
FRONT_STOP = DRIVE.FRONT_STOP
FRONT_SLOW = DRIVE.FRONT_SLOW
DIAGONAL_STOP = DRIVE.DIAGONAL_STOP
DIAGONAL_SLOW = DRIVE.DIAGONAL_SLOW
SIDE_TOUCH = DRIVE.SIDE_TOUCH
REAR_STOP = DRIVE.REAR_STOP
US_EMERGENCY = DRIVE.US_EMERGENCY
US_CRITICAL = DRIVE.US_CRITICAL

# 속도
SPEED_FAST = DRIVE.SPEED_FAST
SPEED_EXPLORE = DRIVE.SPEED_EXPLORE
SPEED_SLOW = DRIVE.SPEED_SLOW
SPEED_CORRIDOR = DRIVE.SPEED_CORRIDOR
BACKUP_SPEED = DRIVE.BACKUP_SPEED
TURN_SPEED = DRIVE.TURN_SPEED
STEER_SPEED = DRIVE.STEER_SPEED

# 타이밍
STUCK_TIME = DRIVE.STUCK_TIME
STUCK_DIST = DRIVE.STUCK_DIST
CORRIDOR_WIDTH = DRIVE.CORRIDOR_WIDTH
LOG_INTERVAL = DRIVE.LOG_INTERVAL
CONTROL_PERIOD = DRIVE.CONTROL_PERIOD

# 도킹 - ARUCO에서 가져옴
DOCK_SIZE_MIN = ARUCO.DOCK_SIZE_MIN
DOCK_SIZE_MAX = ARUCO.DOCK_SIZE_MAX
DOCK_SIZE_TARGET = ARUCO.DOCK_SIZE_TARGET
DOCK_CENTER_TOLERANCE = ARUCO.DOCK_CENTER_TOLERANCE
DOCK_SPEED_FORWARD = ARUCO.DOCK_SPEED_FORWARD
DOCK_SPEED_BACK = ARUCO.DOCK_SPEED_BACK
DOCK_ANGULAR_P = ARUCO.DOCK_ANGULAR_P
DOCK_STABLE_COUNT = ARUCO.DOCK_STABLE_COUNT


class AutoDriveNode(Node):
    def __init__(self):
        super().__init__("auto_drive_node")
        
        # HOME 설정
        self.declare_parameter('home_mode', 'pose')
        self.declare_parameter('home_x', float('nan'))
        self.declare_parameter('home_y', float('nan'))
        self.declare_parameter('home_yaw', float('nan'))
        self.declare_parameter('home_xy_tolerance', 0.25)
        self.declare_parameter('home_yaw_tolerance_deg', 20.0)
        
        self.home_mode = str(self.get_parameter('home_mode').value).strip().lower()
        self.home_xy_tolerance = float(self.get_parameter('home_xy_tolerance').value)
        self.home_yaw_tolerance = math.radians(float(self.get_parameter('home_yaw_tolerance_deg').value))
        
        # HOME pose
        self.home_pose_file = os.path.expanduser('~/.ros_home_config.json')
        self.home_pose_valid = False
        self.home_pose_x = self.home_pose_y = self.home_pose_yaw = 0.0
        
        # ArUco 마커 상태
        self.marker_detected = False
        self.marker_center_x = self.marker_center_y = 0.0
        self.marker_size = 0
        self.marker_id = -1  # 현재 감지된 마커 ID
        
        # 도킹 상태
        self.dock_stable_count = 0
        
        # 연속 회전 방지 (무한 회전 탈출)
        self.consecutive_turns = 0
        self.max_consecutive_turns = 3  # 3번 연속 회전하면 후진
        
        # 탐색 중 PORT 위치 기록
        self.discovered_ports = {}  # {marker_id: (x, y, yaw)}
        
        # 초음파
        self.ultrasonic = 1.0
        self.us_buffer = deque(maxlen=5)
        self.us_sensor = None
        if ULTRASONIC_AVAILABLE:
            try:
                self.us_sensor = Ultrasonic()
                self.get_logger().info("✅ Ultrasonic initialized")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Ultrasonic init failed: {e}")
        
        # LiDAR 거리
        self.front_dist = self.front_left = self.front_right = 1.0
        self.left_dist = self.right_dist = self.rear_dist = 1.0
        self.left_open = self.right_open = 1.0
        
        # 상태
        self.is_enabled = True
        self.state = "EXPLORE"
        self.turn_direction = 1
        self.action_start = None
        self.in_corridor = False
        
        # 위치 추적
        self.position_history = deque(maxlen=50)
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.turn_started = False
        
        # 복귀
        self.returning_home = False
        self.return_target = None  # 'PORT_B' 또는 'PORT_A'
        
        # HOME 로드
        self._load_home_config()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, ROS.CMD_VEL, 1)
        self.camera_pub = self.create_publisher(Bool, ROS.CAMERA_TRIGGER, 1)
        self.driving_pub = self.create_publisher(Bool, ROS.AUTO_DRIVE_ACTIVE, 1)
        self.at_home_pub = self.create_publisher(Bool, ROS.ROBOT_AT_HOME, 1)
        self.us_pub = self.create_publisher(Float32, ROS.ULTRASONIC, 1)
        self.port_odom_pub = self.create_publisher(String, ARUCO.PORT_ODOM, 1)  # PORT 좌표 발행
        
        # Subscribers
        self.create_subscription(LaserScan, ROS.SCAN, self.scan_cb, 1)
        self.create_subscription(Bool, ROS.AUTO_DRIVE_ENABLE, self.enable_cb, 1)
        self.create_subscription(Odometry, ROS.ODOM, self.odom_cb, 1)
        self.create_subscription(Bool, ROS.ROBOT_RETURN_HOME, self.return_home_cb, 1)
        self.create_subscription(String, ROS.MAP_SAVER_CYCLE, self.cycle_complete_cb, 1)
        self.create_subscription(String, ROS.ARUCO_HOME_DETECTED, self.aruco_cb, 10)
        
        # Timers
        self.create_timer(2.0, self.publish_driving_state)
        self.create_timer(CONTROL_PERIOD, self.control)
        self.create_timer(0.33, self.read_ultrasonic)
        self._last_log = 0
        
        self.get_logger().info(f"🚗 Auto Drive Started (home_mode={self.home_mode})")

    def _load_home_config(self):
        """HOME 설정 로드"""
        # 파라미터 우선
        px = float(self.get_parameter('home_x').value)
        py = float(self.get_parameter('home_y').value)
        pyaw = float(self.get_parameter('home_yaw').value)
        
        if not (math.isnan(px) or math.isnan(py) or math.isnan(pyaw)):
            self.home_pose_x, self.home_pose_y, self.home_pose_yaw = px, py, pyaw
            self.home_pose_valid = True
            self.get_logger().info(f"🏠 HOME from params: ({px:.2f}, {py:.2f})")
            return
        
        # 파일에서 로드
        try:
            if os.path.exists(self.home_pose_file):
                with open(self.home_pose_file, 'r') as f:
                    data = json.load(f)
                robot_pose = data.get('robot_pose_at_home', {})
                self.home_pose_x = float(robot_pose.get('x', 0.0))
                self.home_pose_y = float(robot_pose.get('y', 0.0))
                self.home_pose_yaw = float(robot_pose.get('yaw', 0.0))
                self.home_pose_valid = True
                self.get_logger().info(f"🏠 HOME loaded: ({self.home_pose_x:.2f}, {self.home_pose_y:.2f})")
        except Exception as e:
            self.get_logger().warn(f"⚠️ HOME config load failed: {e}")

    def _is_at_home(self) -> bool:
        """HOME 도달 여부"""
        if not self.home_pose_valid:
            return False
        dx = self.home_pose_x - self.current_x
        dy = self.home_pose_y - self.current_y
        dist = math.sqrt(dx*dx + dy*dy)
        yaw_err = self._normalize_angle(self.home_pose_yaw - self.current_yaw)
        return dist <= self.home_xy_tolerance and abs(yaw_err) <= self.home_yaw_tolerance

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    @staticmethod
    def _quat_to_yaw(q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    # ====== Callbacks ======
    def publish_driving_state(self):
        self.driving_pub.publish(Bool(data=self.is_enabled))

    def read_ultrasonic(self):
        if not self.us_sensor:
            return
        try:
            dist = self.us_sensor.get_dist()
            if dist and 0.02 < dist < 4.0:
                self.us_buffer.append(round(dist, 3))
                # 5개 이상 모이면 중간값 사용 (노이즈 제거)
                if len(self.us_buffer) >= 5:
                    self.ultrasonic = sorted(self.us_buffer)[len(self.us_buffer)//2]
                    self.us_pub.publish(Float32(data=self.ultrasonic))
        except:
            pass

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        self.position_history.append((self.current_x, self.current_y, time.time()))

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)
        n = len(ranges)
        if n == 0:
            return
        valid = np.where((ranges > 0.05) & (ranges < 10.0), ranges, 10.0)
        
        def get_min(s, e):
            i1, i2 = int(s*n/360)%n, int(e*n/360)%n
            return float(np.min(valid[i1:i2+1])) if i1<=i2 else float(min(np.min(valid[i1:]), np.min(valid[:i2+1])))
        
        def get_mean(s, e):
            i1, i2 = int(s*n/360)%n, int(e*n/360)%n
            return float(np.mean(valid[i1:i2+1])) if i1<=i2 else float(np.mean(np.concatenate([valid[i1:], valid[:i2+1]])))
        
        self.front_dist = get_min(337, 23)
        self.front_left = get_min(23, 67)
        self.front_right = get_min(293, 337)
        self.left_dist = get_min(67, 112)
        self.right_dist = get_min(248, 293)
        self.rear_dist = get_min(135, 225)
        self.left_open = get_mean(60, 120)
        self.right_open = get_mean(240, 300)

    def enable_cb(self, msg):
        self.is_enabled = msg.data
        if not self.is_enabled:
            self.stop()
        self.get_logger().info(f"AutoDrive: {'ON' if self.is_enabled else 'OFF'}")

    def return_home_cb(self, msg):
        if msg.data:
            self.returning_home = True
            # ★ 무조건 PORT_B 먼저 시도! (360도 회전으로 찾음) ★
            self.return_target = 'PORT_B'
            self.state = "RETURN_PORT_B"
            self.search_start_yaw = None  # 360도 회전 초기화
            self.get_logger().info("🚀 Return requested! PORT_B 먼저 찾기 (360도 회전) → 그 다음 PORT_A(HOME)")
            self.dock_stable_count = 0

    def cycle_complete_cb(self, msg):
        """맵 저장 이벤트 처리"""
        try:
            data = json.loads(msg.data)
            event = data.get('event', '')
            
            if event == 'map_saved':
                # 개별 맵 저장 → 휴식 후 재시작
                cycle_num = data.get('cycle_number', 0)
                total = data.get('total_cycles', 8)
                self.get_logger().info("=" * 50)
                self.get_logger().info(f"🗺️ Map {cycle_num + 1}/{total} saved → 10초 휴식 후 재시작")
                self.get_logger().info("=" * 50)
                self.stop()
                self.returning_home = False
                self.state = "HOME_REST"
                self.home_rest_start = self.get_clock().now()
                self.position_history.clear()
                self.discovered_ports.clear()
                self.dock_stable_count = 0
                
            elif event == 'cycle_complete':
                # 8개 맵 완료 → 완전 정지
                self.get_logger().info("=" * 50)
                self.get_logger().info("🎉 8-Cycle COMPLETE! 모든 맵 저장 완료 → 정지")
                self.get_logger().info("=" * 50)
                self.stop()
                self.returning_home = False
                self.is_enabled = False  # 완전 정지
                self.state = "STOPPED"
                self.position_history.clear()
                self.discovered_ports.clear()
                self.dock_stable_count = 0
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid cycle message: {msg.data}")

    def aruco_cb(self, msg):
        try:
            data = json.loads(msg.data)
            detected = data.get('detected', False)
            
            if detected:
                marker_id = data.get('marker_id', -1)
                
                # PORT_A, PORT_B만 인식, 나머지는 노이즈로 무시
                if marker_id not in ARUCO.DOCK_MARKER_IDS:
                    self.marker_detected = False
                    return
                
                self.marker_detected = True
                self.marker_id = marker_id
                self.marker_center_x = data.get('center_x', 0)
                self.marker_center_y = data.get('center_y', 0)
                self.marker_size = data.get('size', 0)
                port_name = data.get('port_name', ARUCO.PORT_MAP.get(marker_id, f'ID:{marker_id}'))
                
                # 탐색 중 마커 발견 시 → 임시 좌표 저장 (도킹 완료 시 정확한 좌표로 덮어씀)
                if not self.returning_home and marker_id not in self.discovered_ports:
                    # ★ 임시 좌표 저장 (복귀 시 찾아갈 수 있게) ★
                    self.discovered_ports[marker_id] = (self.current_x, self.current_y, self.current_yaw)
                    self.get_logger().info(
                        f"👀 {port_name} 발견! 임시 좌표 저장: ({self.current_x:.2f}, {self.current_y:.2f})",
                        throttle_duration_sec=3.0
                    )
                
                # RETURN_HOME 상태에서 마커 감지 시 로그
                if self.returning_home:
                    self.get_logger().info(
                        f"🎯 {port_name} detected! x={self.marker_center_x:.2f}, size={self.marker_size:.0f}px",
                        throttle_duration_sec=1.0
                    )
            else:
                self.marker_detected = False
        except:
            self.marker_detected = False

    # ====== Control ======
    def control(self):
        if not self.is_enabled or self.state == "STOPPED":
            self.stop()
            return
        
        # 초음파 긴급 정지 - LiDAR 정면도 가까울 때만 (값 튐 방지)
        # 도킹 중에는 완전 무시
        is_docking = self.returning_home and self.marker_detected
        us_close = 0 < self.ultrasonic < US_EMERGENCY
        lidar_close = self.front_dist < 0.20  # LiDAR 정면도 20cm 이내
        if us_close and lidar_close and self.state not in ["BACKUP", "TURN"] and not is_docking:
            self.get_logger().error(f"🛑 EMERGENCY! US:{self.ultrasonic:.2f}m LiDAR:{self.front_dist:.2f}m")
            self.turn_direction = 1 if self.left_open >= self.right_open else -1
            self.state = "BACKUP"
            self.action_start = self.get_clock().now()
            self.cmd_vel_pub.publish(Twist(linear=Twist().linear, angular=Twist().angular))
            twist = Twist()
            twist.linear.x = BACKUP_SPEED
            self.cmd_vel_pub.publish(twist)
            return
        
        self.in_corridor = self.left_dist < CORRIDOR_WIDTH and self.right_dist < CORRIDOR_WIDTH
        
        # returning_home 플래그가 True면 복귀 상태 유지 (단, BACKUP/TURN/EXPLORE_FOR_PORT_B는 허용)
        if self.returning_home and self.state not in ["RETURN_HOME", "RETURN_PORT_B", "BACKUP", "TURN", "EXPLORE_FOR_PORT_B"]:
            # return_target에 따라 상태 결정
            if self.return_target == 'PORT_B':
                self.state = "RETURN_PORT_B"
            else:
                self.state = "RETURN_HOME"
        
        # 로그
        now = time.time()
        if now - self._last_log > LOG_INTERVAL:
            self.get_logger().info(f"[{self.state}] US:{self.ultrasonic:.2f} F:{self.front_dist:.2f}")
            self._last_log = now
        
        # 상태 머신
        if self.state == "RETURN_PORT_B":
            self.do_return_port_b()
        elif self.state == "RETURN_HOME":
            self.do_return_home()
        elif self.state == "HOME_REST":
            self.do_home_rest()
        elif self.state == "TURN":
            self.do_turn()
        elif self.state == "BACKUP":
            self.do_backup()
        elif self.state == "EXPLORE_FOR_PORT_B":
            self.do_explore_for_port_b()
        else:
            self.do_explore()

    def do_explore(self):
        twist = Twist()
        front_clear = min(self.front_dist, self.ultrasonic if self.ultrasonic > 0 else 10)
        
        # ★ 탐색 중 PORT_B 발견 시 → 자동 도킹하여 좌표 저장 ★
        if self.marker_detected and self.marker_id == ARUCO.PORT_B_ID:
            # 아직 좌표 저장 안 됐으면 도킹
            if ARUCO.PORT_B_ID not in self.discovered_ports:
                center_x_error = self.marker_center_x
                current_size = self.marker_size
                center_ok = abs(center_x_error) < DOCK_CENTER_TOLERANCE
                size_in_zone = DOCK_SIZE_MIN <= current_size <= DOCK_SIZE_MAX
                
                if size_in_zone and center_ok:
                    self.dock_stable_count += 1
                    if self.dock_stable_count >= DOCK_STABLE_COUNT:
                        # ★ 도킹 완료! 좌표 저장 후 탐색 계속 ★
                        self.discovered_ports[ARUCO.PORT_B_ID] = (self.current_x, self.current_y, self.current_yaw)
                        self.get_logger().info("=" * 50)
                        self.get_logger().info(f"📍 PORT_B 도킹 완료! 좌표 저장: ({self.current_x:.2f}, {self.current_y:.2f})")
                        self.get_logger().info("=" * 50)
                        self.publish_port_odom("PORT_B", self.current_x, self.current_y, self.current_yaw)
                        self.dock_stable_count = 0
                        # 후진 후 탐색 재개
                        self.state = "BACKUP"
                        self.action_start = self.get_clock().now()
                        self.turn_direction = 1
                        return
                else:
                    self.dock_stable_count = 0
                
                # 도킹 제어
                twist.angular.z = -center_x_error * DOCK_ANGULAR_P
                if current_size < DOCK_SIZE_MIN:
                    twist.linear.x = DOCK_SPEED_FORWARD
                elif current_size > DOCK_SIZE_MAX:
                    twist.linear.x = -DOCK_SPEED_BACK
                
                self.get_logger().info(
                    f"🎯 [탐색 중] PORT_B 도킹: size={current_size:.0f}, x={center_x_error:.2f}",
                    throttle_duration_sec=0.5
                )
                self.cmd_vel_pub.publish(twist)
                return
        
        # 초음파 크리티컬
        if 0 < self.ultrasonic < US_CRITICAL:
            self.turn_direction = 1 if self.left_open >= self.right_open else -1
            self.start_turn()
            return
        
        # 정면 막힘
        if front_clear < FRONT_STOP:
            self.camera_pub.publish(Bool(data=True))
            self.turn_direction = 1 if self.left_open >= self.right_open else -1
            self.start_turn()
            return
        
        # 통로 모드
        if self.in_corridor and front_clear > FRONT_STOP:
            center_error = self.left_dist - self.right_dist
            twist.angular.z = max(-STEER_SPEED*0.8, min(STEER_SPEED*0.8, center_error*1.2))
            twist.linear.x = SPEED_EXPLORE if front_clear > FRONT_SLOW else SPEED_CORRIDOR
            self.cmd_vel_pub.publish(twist)
            return
        
        # 일반 탐색 - 열린 공간(긴 거리)으로 유도
        steer = 0.0
        open_diff = self.left_open - self.right_open
        
        if front_clear > FRONT_SLOW:
            twist.linear.x = SPEED_FAST
            # 열린 공간 쪽으로 더 강하게 유도 (차이가 클수록 강하게)
            steer = open_diff * 0.15  # 0.05 → 0.15로 증가
        else:
            # 사선 회피
            if self.front_left < DIAGONAL_STOP:
                steer = -STEER_SPEED
            elif self.front_right < DIAGONAL_STOP:
                steer = STEER_SPEED
            else:
                # 사선 회피 아닐 때도 열린 쪽으로 유도
                steer = open_diff * 0.2
            twist.linear.x = SPEED_EXPLORE if front_clear > FRONT_STOP + 0.05 else SPEED_SLOW
        
        # 측면 회피 (열린 공간 유도보다 우선)
        if self.left_dist < SIDE_TOUCH:
            steer = -STEER_SPEED * 0.5
        elif self.right_dist < SIDE_TOUCH:
            steer = STEER_SPEED * 0.5
        
        # 스티어링 제한
        steer = max(-STEER_SPEED, min(STEER_SPEED, steer))
        
        twist.angular.z = steer
        self.cmd_vel_pub.publish(twist)

    def start_turn(self):
        self.state = "TURN"
        self.action_start = self.get_clock().now()
        self.turn_started = False
        self.consecutive_turns += 1  # 연속 회전 카운트
        
        # ★ do_turn()에서 360도 회전하며 열린 곳 찾으므로, 여기선 체크 안 함 ★
        # 연속 회전 카운터는 갇힘 감지용으로만 사용
        self.get_logger().info(f"🔄 회전 시작 (연속 {self.consecutive_turns}번)")

    def do_turn(self):
        elapsed = (self.get_clock().now() - self.action_start).nanoseconds / 1e9
        
        # ★★★ 회전 중 PORT 발견하면 바로 도킹! ★★★
        if self.returning_home and self.marker_detected:
            if self.return_target == 'PORT_B' and self.marker_id == ARUCO.PORT_B_ID:
                self.get_logger().info(f"🎯 회전 중 PORT_B 발견! → 도킹 시작")
                self.state = "RETURN_PORT_B"
                self.action_start = self.get_clock().now()
                self.consecutive_turns = 0
                return
            elif self.return_target == 'PORT_A' and self.marker_id == ARUCO.PORT_A_ID:
                self.get_logger().info(f"🎯 회전 중 PORT_A 발견! → 도킹 시작")
                self.state = "RETURN_HOME"
                self.action_start = self.get_clock().now()
                self.consecutive_turns = 0
                return
        
        if not self.turn_started:
            self.target_yaw = self._normalize_angle(self.current_yaw + math.pi/4 * self.turn_direction)
            self.turn_started = True
            self.turn_total_rotation = 0.0  # 총 회전량 추적
            self.turn_last_yaw = self.current_yaw
        
        # 회전량 누적
        yaw_delta = abs(self._normalize_angle(self.current_yaw - self.turn_last_yaw))
        self.turn_total_rotation += yaw_delta
        self.turn_last_yaw = self.current_yaw
        
        angle_diff = self._normalize_angle(self.target_yaw - self.current_yaw)
        
        if abs(angle_diff) < math.radians(5):
            # ★★★ 정면이 열렸으면 바로 진입! ★★★
            if self.front_dist > FRONT_STOP + 0.05:
                self.get_logger().info(f"✅ 열린 통로 발견! (front={self.front_dist:.2f}m) → 진입!")
                # 회전 완료 - 이전 상태에 따라 복귀
                if self.returning_home:
                    if self.return_target == 'PORT_B':
                        self.state = "EXPLORE_FOR_PORT_B"
                    else:
                        self.state = "RETURN_HOME"
                else:
                    self.state = "EXPLORE"
                self.consecutive_turns = 0
                self.position_history.clear()
                return
            # 아직 막혀있으면 45도 더 회전
            self.target_yaw = self._normalize_angle(self.target_yaw + math.pi/4 * self.turn_direction)
            self.get_logger().info(
                f"🔄 {math.degrees(self.turn_total_rotation):.0f}° 회전, 아직 막힘 (front={self.front_dist:.2f}m) → 45° 더",
                throttle_duration_sec=0.5
            )
        
        # ★★★ 360도 다 돌았는데도 열린 곳 없으면 → 후진 후 반대 방향 ★★★
        if self.turn_total_rotation > 2 * math.pi:
            self.get_logger().warn(f"⚠️ 360도 회전 완료! 열린 곳 없음 → 후진 후 반대 방향")
            self.state = "BACKUP"
            self.action_start = self.get_clock().now()
            return
        
        # 20초 타임아웃 (안전장치)
        if elapsed > 20.0:
            self.get_logger().warn(f"⚠️ 회전 타임아웃! → 후진")
            self.state = "BACKUP"
            self.action_start = self.get_clock().now()
            return
        
        twist = Twist()
        turn_speed = max(0.2, min(TURN_SPEED, abs(angle_diff) * 1.5))
        twist.angular.z = turn_speed if angle_diff > 0 else -turn_speed
        self.cmd_vel_pub.publish(twist)

    def do_backup(self):
        elapsed = (self.get_clock().now() - self.action_start).nanoseconds / 1e9
        
        if self.rear_dist < REAR_STOP or elapsed > 1.0:
            self.turn_direction = -self.turn_direction  # 반대 방향으로 회전 시도
            self.state = "TURN"
            self.action_start = self.get_clock().now()
            self.turn_started = False
            self.consecutive_turns = 0  # 후진 후 카운터 리셋
            self.get_logger().info(f"🔄 Backup done, turning {'left' if self.turn_direction > 0 else 'right'}")
            return
        
        twist = Twist()
        twist.linear.x = BACKUP_SPEED
        self.cmd_vel_pub.publish(twist)

    def do_home_rest(self):
        """HOME 도착 후 10초 휴식 (리셋 대기)"""
        elapsed = (self.get_clock().now() - self.home_rest_start).nanoseconds / 1e9
        remaining = 10.0 - elapsed
        
        if remaining <= 0:
            self.get_logger().info("=" * 50)
            self.get_logger().info("💤 휴식 완료! 후진 후 탐색 재시작")
            self.get_logger().info("=" * 50)
            # 후진 후 회전해서 PORT_A에서 벗어나기
            self.state = "BACKUP"
            self.action_start = self.get_clock().now()
            self.turn_direction = 1  # 왼쪽으로 회전
            self.position_history.clear()
            self.discovered_ports.clear()
            self.dock_stable_count = 0
            return
        
        # 남은 시간 로깅 (1초마다)
        self.get_logger().info(
            f"💤 휴식 중... {remaining:.0f}초 남음",
            throttle_duration_sec=1.0
        )
        self.stop()

    def do_explore_for_port_b(self):
        """PORT_B를 찾기 위한 자율주행 탐색 - PORT_A는 완전 무시!"""
        twist = Twist()
        front_clear = min(self.front_dist, self.ultrasonic if self.ultrasonic > 0 else 10)
        
        # ★★★ PORT_B 발견! → 즉시 도킹 모드로 전환 ★★★
        if self.marker_detected and self.marker_id == ARUCO.PORT_B_ID:
            self.get_logger().info("🎯 자율주행 중 PORT_B 발견! → 도킹 모드로 전환!")
            self.state = "RETURN_PORT_B"
            return
        
        # PORT_A가 보여도 완전 무시 (로그만 출력)
        if self.marker_detected and self.marker_id == ARUCO.PORT_A_ID:
            self.get_logger().info(
                f"🔍 PORT_B 탐색 자율주행 중... (PORT_A 보임, 무시!)",
                throttle_duration_sec=3.0
            )
        
        # 장애물 회피 (일반 EXPLORE와 동일)
        if 0 < self.ultrasonic < US_EMERGENCY:
            self.get_logger().warn(f"⚠️ [PORT_B 탐색] 장애물! 후진")
            self.turn_direction = 1 if self.left_open > self.right_open else -1
            self.state = "BACKUP"
            self.action_start = self.get_clock().now()
            return
        
        if front_clear < FRONT_STOP:
            self.turn_direction = 1 if self.left_open > self.right_open else -1
            self.start_turn()
            return
        
        # 일반 탐색 주행
        self.get_logger().info(
            f"🚗 [PORT_B 탐색] 자율주행 중... F:{front_clear:.2f}",
            throttle_duration_sec=2.0
        )
        
        twist.linear.x = SPEED_EXPLORE
        # 열린 공간으로 유도 (do_explore와 동일)
        open_diff = self.left_open - self.right_open
        steer = open_diff * 0.15
        steer = max(-STEER_SPEED, min(STEER_SPEED, steer))
        twist.angular.z = steer
        self.cmd_vel_pub.publish(twist)

    def do_return_port_b(self):
        """PORT_B 도킹 → 완료 후 PORT_A로 (do_return_home과 동일 방식)"""
        twist = Twist()
        front_clear = min(self.front_dist, self.ultrasonic if self.ultrasonic > 0 else 10)
        
        # 360도 회전 탐색 중인지 확인
        is_searching = hasattr(self, 'search_start_yaw') and self.search_start_yaw is not None
        
        # 장애물 회피 (360도 회전 중엔 무시)
        if 0 < self.ultrasonic < US_EMERGENCY and not is_searching:
            self.get_logger().warn(f"⚠️ RETURN_PORT_B 중 장애물! 후진")
            self.turn_direction = 1 if self.left_open > self.right_open else -1
            self.state = "BACKUP"
            self.action_start = self.get_clock().now()
            return
        
        if front_clear < FRONT_STOP and not is_searching:
            self.turn_direction = 1 if self.left_open > self.right_open else -1
            self.start_turn()
            return
        
        # ===== PORT_B 보이면 → 도킹 =====
        # ★★★ PORT_A가 보여도 무시! PORT_B만 도킹! ★★★
        if self.marker_detected and self.marker_id == ARUCO.PORT_B_ID:
            center_x_error = self.marker_center_x
            current_size = self.marker_size
            center_ok = abs(center_x_error) < DOCK_CENTER_TOLERANCE
            
            # 360도 회전 탐색 중이면 취소
            if is_searching:
                self.get_logger().info(f"🎯 PORT_B 발견! 360도 회전 취소 → 도킹 시작")
                self.search_start_yaw = None
            
            size_in_zone = DOCK_SIZE_MIN <= current_size <= DOCK_SIZE_MAX
            
            if size_in_zone and center_ok:
                self.dock_stable_count += 1
                self.get_logger().info(
                    f"🎯 [PORT_B ZONE] STABLE {self.dock_stable_count}/{DOCK_STABLE_COUNT} "
                    f"(size={current_size:.0f}, x={center_x_error:.2f})"
                )
                if self.dock_stable_count >= DOCK_STABLE_COUNT:
                    self.stop()
                    self.get_logger().info("=" * 50)
                    self.get_logger().info(f"✅ PORT_B DOCK COMPLETE! 이제 PORT_A(HOME)로 이동")
                    self.get_logger().info("=" * 50)
                    
                    # ★ 도킹 완료 시점에 정확한 좌표 저장 ★
                    self.discovered_ports[ARUCO.PORT_B_ID] = (self.current_x, self.current_y, self.current_yaw)
                    self.get_logger().info(f"📍 PORT_B 좌표 저장: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f})")
                    self.publish_port_odom("PORT_B", self.current_x, self.current_y, self.current_yaw)
                    
                    # PORT_B 완료 → PORT_A로 전환
                    self.return_target = 'PORT_A'
                    self.state = "RETURN_HOME"
                    self.dock_stable_count = 0
                    self.search_start_yaw = None
                    return
                self.cmd_vel_pub.publish(twist)
                return
            else:
                self.dock_stable_count = 0
            
            # 도킹 제어: x 정렬 + 전진/후진
            twist.angular.z = -center_x_error * DOCK_ANGULAR_P
            
            if current_size < DOCK_SIZE_MIN:
                twist.linear.x = DOCK_SPEED_FORWARD
                self.get_logger().info(
                    f"➡️ [PORT_B] FORWARD: size={current_size:.0f} < {DOCK_SIZE_MIN}",
                    throttle_duration_sec=0.5
                )
            elif current_size > DOCK_SIZE_MAX:
                twist.linear.x = -DOCK_SPEED_BACK
                self.get_logger().info(
                    f"⬅️ [PORT_B] BACK: size={current_size:.0f} > {DOCK_SIZE_MAX}",
                    throttle_duration_sec=0.5
                )
            else:
                self.get_logger().info(
                    f"🔄 [PORT_B] ALIGN: size={current_size:.0f} OK, x={center_x_error:.2f}",
                    throttle_duration_sec=0.5
                )
            
            self.cmd_vel_pub.publish(twist)
            return
        
        # ===== PORT_B 안 보이면 → 360도 회전하며 찾기 =====
        self.dock_stable_count = 0
        
        # ★★★ PORT_A가 보여도 무시하고 계속 회전! PORT_B만 찾음 ★★★
        if self.marker_detected and self.marker_id == ARUCO.PORT_A_ID:
            self.get_logger().info(
                f"🔍 PORT_B 찾는 중... (PORT_A 보임, 무시하고 계속 회전!)",
                throttle_duration_sec=2.0
            )
            # PORT_A가 보여도 아래 360도 회전 로직 계속 실행
        
        # 360도 회전 시작
        if not is_searching:
            self.search_start_yaw = self.current_yaw
            self.search_last_yaw = self.current_yaw
            self.search_total_rotation = 0.0
            self.search_port_a_seen = False  # PORT_A 봤는지 기록
            self.get_logger().info(f"🔄 PORT_B 탐색: 360도 회전 시작!")
        
        # PORT_A를 봤으면 기록 (PORT_B가 반대편에 있을 수 있음)
        if self.marker_detected and self.marker_id == ARUCO.PORT_A_ID:
            if hasattr(self, 'search_port_a_seen'):
                self.search_port_a_seen = True
        
        # 회전량 누적
        yaw_delta = self._normalize_angle(self.current_yaw - self.search_last_yaw)
        self.search_total_rotation += abs(yaw_delta)
        self.search_last_yaw = self.current_yaw
        
        self.get_logger().info(
            f"🔄 [PORT_B 탐색] 회전 중: {math.degrees(self.search_total_rotation):.0f}°/360°",
            throttle_duration_sec=1.0
        )
        
        # 360도 돌았는데도 PORT_B 못 찾으면 → 자율주행하면서 찾기!
        if self.search_total_rotation > 2 * math.pi:
            self.get_logger().warn(f"⚠️ 360도 회전 완료! PORT_B 안 보임 → 자율주행하며 PORT_B 찾기!")
            self.search_start_yaw = None
            # ★★★ PORT_A로 가지 않음! PORT_B 찾을 때까지 EXPLORE! ★★★
            # return_target은 'PORT_B' 유지!
            self.state = "EXPLORE_FOR_PORT_B"  # 특수 탐색 모드
            return
        
        # 천천히 회전 (PORT_B 찾을 때까지!)
        twist.angular.z = 0.4
        self.cmd_vel_pub.publish(twist)

    def do_return_home(self):
        twist = Twist()
        front_clear = min(self.front_dist, self.ultrasonic if self.ultrasonic > 0 else 10)
        
        # 360도 회전 탐색 중인지 확인
        is_searching = hasattr(self, 'search_start_yaw') and self.search_start_yaw is not None
        
        # 장애물 회피 - 360도 회전 중에는 무시
        if 0 < self.ultrasonic < US_EMERGENCY and not is_searching:
            self.get_logger().warn(f"⚠️ RETURN_HOME 중 장애물! 후진")
            self.turn_direction = 1 if self.left_open > self.right_open else -1
            self.state = "BACKUP"
            self.action_start = self.get_clock().now()
            return
        
        # 360도 회전 중에는 정면 장애물도 무시
        if front_clear < FRONT_STOP and not is_searching:
            self.turn_direction = 1 if self.left_open > self.right_open else -1
            self.start_turn()
            return
        
        # ===== ZONE 방식 도킹: PORT_A 보이고 size가 범위 안에 들어오면 완료 =====
        
        # PORT_A가 보이면 → size 기준으로 도킹
        if self.marker_detected and self.marker_id == ARUCO.PORT_A_ID:
            center_x_error = self.marker_center_x
            current_size = self.marker_size
            center_ok = abs(center_x_error) < DOCK_CENTER_TOLERANCE
            
            # 360도 회전 탐색 중이면 취소 (PORT_A 발견!)
            if is_searching:
                self.get_logger().info(f"🎯 PORT_A 발견! 360도 회전 취소 → 도킹 시작")
                self.search_start_yaw = None
                self.search_last_yaw = None
                self.search_total_rotation = 0.0
            
            # === ZONE 체크: size가 범위 안이고 x 정렬되면 완료 ===
            size_in_zone = DOCK_SIZE_MIN <= current_size <= DOCK_SIZE_MAX
            
            if size_in_zone and center_ok:
                self.dock_stable_count += 1
                self.get_logger().info(
                    f"🎯 [ZONE] STABLE {self.dock_stable_count}/{DOCK_STABLE_COUNT} "
                    f"(size={current_size:.0f}, x={center_x_error:.2f})"
                )
                if self.dock_stable_count >= DOCK_STABLE_COUNT:
                    self.stop()
                    self.returning_home = False
                    self.state = "HOME_REST"  # 10초 휴식 상태
                    self.home_rest_start = self.get_clock().now()
                    self.get_logger().info("=" * 50)
                    self.get_logger().info(f"✅ HOME DOCK COMPLETE! size={current_size:.0f}px (ZONE: {DOCK_SIZE_MIN}~{DOCK_SIZE_MAX})")
                    self.get_logger().info("💤 10초간 휴식 시작 (리셋 대기)...")
                    self.get_logger().info("=" * 50)
                    
                    # ★ 도킹 완료 시점에 정확한 좌표 저장 ★
                    self.discovered_ports[ARUCO.PORT_A_ID] = (self.current_x, self.current_y, self.current_yaw)
                    self.get_logger().info(f"📍 PORT_A 좌표 저장: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f})")
                    self.publish_port_odom("PORT_A", self.current_x, self.current_y, self.current_yaw)
                    
                    self.at_home_pub.publish(Bool(data=True))
                    return
                self.cmd_vel_pub.publish(twist)  # 정지 유지
                return
            else:
                self.dock_stable_count = 0
            
            # === 도킹 제어 ===
            # 1. x 정렬 안되면 회전
            twist.angular.z = -center_x_error * DOCK_ANGULAR_P
            
            # 2. size 기준 전진/후진
            if current_size < DOCK_SIZE_MIN:
                # 너무 멀면 전진
                twist.linear.x = DOCK_SPEED_FORWARD
                self.get_logger().info(
                    f"➡️ [PORT_A] FORWARD: size={current_size:.0f} < {DOCK_SIZE_MIN}",
                    throttle_duration_sec=0.5
                )
            elif current_size > DOCK_SIZE_MAX:
                # 너무 가까우면 후진
                twist.linear.x = -DOCK_SPEED_BACK
                self.get_logger().info(
                    f"⬅️ [PORT_A] BACK: size={current_size:.0f} > {DOCK_SIZE_MAX}",
                    throttle_duration_sec=0.5
                )
            else:
                # ZONE 안인데 x만 안 맞음 → 회전만
                self.get_logger().info(
                    f"🔄 [PORT_A] ALIGN: size={current_size:.0f} OK, x={center_x_error:.2f}",
                    throttle_duration_sec=0.5
                )
            
            self.cmd_vel_pub.publish(twist)
            return
        
        # PORT_B가 보이면 무시 (PORT_A 찾기)
        if self.marker_detected and self.marker_id == ARUCO.PORT_B_ID:
            if is_searching:
                self.get_logger().info(
                    f"🔄 360도 회전 중 - PORT_B 무시, 계속 회전",
                    throttle_duration_sec=2.0
                )
                # 아래 360도 회전 로직으로 넘어감
            else:
                self.get_logger().info(
                    f"🔍 PORT_A 찾는 중... (PORT_B 보임, 무시)",
                    throttle_duration_sec=2.0
                )
        
        # ===== PORT_A가 안 보일 때: 탐색 =====
        self.dock_stable_count = 0
        
        # 발견한 PORT 위치가 있으면 → 그 위치로 먼저 이동
        if self.discovered_ports:
            # PORT_A 우선, 없으면 PORT_B
            target_port_id = ARUCO.PORT_A_ID if ARUCO.PORT_A_ID in self.discovered_ports else (
                ARUCO.PORT_B_ID if ARUCO.PORT_B_ID in self.discovered_ports else None
            )
            
            if target_port_id is not None:
                port_x, port_y, port_yaw = self.discovered_ports[target_port_id]
                dx = port_x - self.current_x
                dy = port_y - self.current_y
                dist_to_port = math.sqrt(dx*dx + dy*dy)
                angle_to_port = math.atan2(dy, dx)
                angle_error = self._normalize_angle(angle_to_port - self.current_yaw)
                
                port_name = ARUCO.PORT_MAP.get(target_port_id, f"ID:{target_port_id}")
                self.get_logger().info(
                    f"🚀 Going to {port_name} ({port_x:.2f}, {port_y:.2f}), dist={dist_to_port:.2f}m",
                    throttle_duration_sec=3.0
                )
                
                # PORT 근처에 도착하면 360도 회전하며 마커 찾기
                if dist_to_port < 0.6:
                    # 360도 회전하며 마커 탐색
                    if not hasattr(self, 'search_start_yaw') or self.search_start_yaw is None:
                        self.search_start_yaw = self.current_yaw
                        self.search_last_yaw = self.current_yaw
                        self.search_total_rotation = 0.0
                        self.get_logger().info(f"🔄 [{port_name}] 근처 도착! 360도 회전하며 마커 탐색 시작")
                    
                    # 회전량 누적 (이전 yaw와의 차이를 누적)
                    yaw_delta = self._normalize_angle(self.current_yaw - self.search_last_yaw)
                    self.search_total_rotation += abs(yaw_delta)
                    self.search_last_yaw = self.current_yaw
                    
                    self.get_logger().info(
                        f"🔄 [{port_name}] 회전 중: {math.degrees(self.search_total_rotation):.0f}°/360°",
                        throttle_duration_sec=1.0
                    )
                    
                    # 360도 이상 돌았는데 마커 못 찾으면 → 다시 접근
                    if self.search_total_rotation > 2 * math.pi:  # 360도
                        self.get_logger().warn(f"⚠️ [{port_name}] 360도 회전 완료! 마커 못 찾음, 더 접근")
                        self.search_start_yaw = None
                        self.search_last_yaw = None
                        self.search_total_rotation = 0.0
                        # 조금 더 가까이 접근
                        twist.linear.x = 0.08
                        self.cmd_vel_pub.publish(twist)
                        return
                    
                    # 천천히 회전
                    twist.angular.z = 0.4
                    self.cmd_vel_pub.publish(twist)
                    return
                
                # 회전 탐색 변수 초기화 (멀어지면)
                if hasattr(self, 'search_start_yaw') and self.search_start_yaw is not None:
                    self.search_start_yaw = None
                    self.search_last_yaw = None
                    self.search_total_rotation = 0.0
                
                # PORT 방향으로 이동 (장애물 회피하면서)
                # 정면 막히면 열린 쪽으로 우회
                if front_clear < FRONT_SLOW:
                    # 열린 쪽으로 회전하면서 천천히 전진
                    open_diff = self.left_open - self.right_open
                    twist.angular.z = open_diff * 0.3
                    twist.linear.x = SPEED_SLOW if front_clear > FRONT_STOP else 0.0
                    self.get_logger().info(
                        f"⚠️ [{port_name}] 장애물 우회 중... L:{self.left_open:.2f} R:{self.right_open:.2f}",
                        throttle_duration_sec=1.0
                    )
                elif abs(angle_error) > 0.3:
                    # PORT 방향으로 회전
                    twist.angular.z = 0.3 * (1 if angle_error > 0 else -1)
                else:
                    # PORT 방향으로 전진
                    twist.linear.x = SPEED_EXPLORE
                    twist.angular.z = angle_error * 0.5
                
                self.cmd_vel_pub.publish(twist)
                return
        
        # HOME pose 없고 PORT도 없으면 → 탐색
        self.get_logger().info("🔍 Searching for ArUco marker... (exploring)", throttle_duration_sec=3.0)
        
        # 탐색 이동
        steer = 0.0
        if self.front_dist > FRONT_SLOW:
            twist.linear.x = SPEED_EXPLORE
            steer = (self.left_open - self.right_open) * 0.05
        else:
            twist.linear.x = SPEED_SLOW
            if self.front_left < DIAGONAL_STOP:
                steer = -STEER_SPEED
            elif self.front_right < DIAGONAL_STOP:
                steer = STEER_SPEED
        twist.angular.z = steer
        self.cmd_vel_pub.publish(twist)

    def publish_port_odom(self, port_name, x, y, yaw):
        """PORT 좌표(odom 기준) 발행 → map_saver가 YAML에 저장"""
        msg = String()
        msg.data = json.dumps({
            "port_name": port_name,
            "x": round(x, 3),
            "y": round(y, 3),
            "yaw": round(yaw, 3)
        })
        self.port_odom_pub.publish(msg)
        self.get_logger().info(f"📡 PORT_ODOM 발행: {port_name} ({x:.2f}, {y:.2f}, {yaw:.2f})")

    def stop(self):
        try:
            if rclpy.ok():
                self.cmd_vel_pub.publish(Twist())
        except:
            pass

    def destroy_node(self):
        self.stop()
        if self.us_sensor:
            try:
                self.us_sensor.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
