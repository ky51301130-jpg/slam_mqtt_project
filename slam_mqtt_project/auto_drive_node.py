#!/usr/bin/env python3
"""SLAM 탐색 자율주행 노드 - LiDAR + 초음파 장애물 회피 + ArUco 도킹"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math, json, os, time
from collections import deque

try:
    from pinkylib import Ultrasonic
    US_OK = True
except ImportError:
    US_OK = False

from slam_mqtt_project.topics import ROS, ARUCO, DRIVE

# 상수
D = DRIVE  # 짧은 alias
A = ARUCO


class AutoDriveNode(Node):
    def __init__(self):
        super().__init__("auto_drive_node")
        
        # 파라미터
        for p, v in [('home_mode','pose'),('home_x',float('nan')),('home_y',float('nan')),
                     ('home_yaw',float('nan')),('home_xy_tolerance',0.25),('home_yaw_tolerance_deg',20.0)]:
            self.declare_parameter(p, v)
        
        self.home_xy_tol = self.get_parameter('home_xy_tolerance').value
        self.home_yaw_tol = math.radians(self.get_parameter('home_yaw_tolerance_deg').value)
        
        # HOME 설정
        self.home_valid = False
        self.home_x = self.home_y = self.home_yaw = 0.0
        self._load_home()
        
        # 상태 변수
        self.marker_detected = False
        self.marker_id = -1
        self.marker_x = self.marker_size = 0
        self.dock_stable = 0
        self.discovered_ports = {}
        
        # 초음파 (버퍼 크기 증가로 노이즈 필터링 강화)
        self.us = 1.0
        self.us_buf = deque(maxlen=10)
        self.us_sensor = Ultrasonic() if US_OK else None
        
        # LiDAR 거리
        self.front = self.fl = self.fr = 1.0
        self.left = self.right = self.rear = 1.0
        self.left_open = self.right_open = 1.0
        
        # 상태
        self.enabled = True
        self.state = "EXPLORE"
        self.turn_dir = 1
        self.act_start = None
        self.returning = False
        self.target = None
        
        # 회피 상태 (PORT 이동 중 장애물 회피용)
        self.avoiding = False
        self.avoid_dir = 0  # -1: 좌회피, 1: 우회피
        self.avoid_start = None
        self.pre_avoid_yaw = 0.0  # 회피 전 목표 방향
        
        # 위치
        self.pos_hist = deque(maxlen=50)
        self.x = self.y = self.yaw = 0.0
        self.target_yaw = 0.0
        self.turn_started = False
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, ROS.CMD_VEL, 1)
        self.driving_pub = self.create_publisher(Bool, ROS.AUTO_DRIVE_ACTIVE, 1)
        self.home_pub = self.create_publisher(Bool, ROS.ROBOT_AT_HOME, 1)
        self.us_pub = self.create_publisher(Float32, ROS.ULTRASONIC, 1)
        self.port_pub = self.create_publisher(String, ARUCO.PORT_ODOM, 1)
        
        # Subscribers
        self.create_subscription(LaserScan, ROS.SCAN, self.scan_cb, 1)
        self.create_subscription(Bool, ROS.AUTO_DRIVE_ENABLE, self.enable_cb, 1)
        self.create_subscription(Odometry, ROS.ODOM, self.odom_cb, 1)
        self.create_subscription(Bool, ROS.ROBOT_RETURN_HOME, self.return_cb, 1)
        self.create_subscription(String, ROS.MAP_SAVER_CYCLE, self.cycle_cb, 1)
        self.create_subscription(String, ROS.ARUCO_HOME_DETECTED, self.aruco_cb, 10)
        
        # Timers
        self.create_timer(2.0, lambda: self.driving_pub.publish(Bool(data=self.enabled)))
        self.create_timer(D.CONTROL_PERIOD, self.control)
        self.create_timer(0.33, self.read_us)
        self._last_log = 0
        
        self.get_logger().info("🚗 Auto Drive Started")

    def _load_home(self):
        px, py, pyaw = [self.get_parameter(p).value for p in ['home_x','home_y','home_yaw']]
        if not any(math.isnan(v) for v in [px,py,pyaw]):
            self.home_x, self.home_y, self.home_yaw, self.home_valid = px, py, pyaw, True
            return
        try:
            with open(os.path.expanduser('~/.ros_home_config.json')) as f:
                d = json.load(f).get('robot_pose_at_home', {})
                self.home_x, self.home_y, self.home_yaw = d.get('x',0), d.get('y',0), d.get('yaw',0)
                self.home_valid = True
        except: pass

    @staticmethod
    def norm(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def read_us(self):
        if not self.us_sensor: return
        try:
            d = self.us_sensor.get_dist()
            if d and 0.02 < d < 4.0:
                self.us_buf.append(d)
                if len(self.us_buf) >= 5:
                    self.us = sorted(self.us_buf)[2]
                    self.us_pub.publish(Float32(data=self.us))
        except: pass

    def odom_cb(self, m):
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def scan_cb(self, m):
        r = np.array(m.ranges)
        n = len(r)
        if n == 0: return
        v = np.where((r > 0.05) & (r < 10), r, 10.0)
        
        def get(s, e, fn=np.min):
            i1, i2 = int(s*n/360)%n, int(e*n/360)%n
            return float(fn(v[i1:i2+1]) if i1<=i2 else fn(np.concatenate([v[i1:], v[:i2+1]])))
        
        self.front = get(337, 23)
        self.fl, self.fr = get(23, 67), get(293, 337)
        self.left, self.right = get(67, 112), get(248, 293)
        self.rear = get(135, 225)
        self.left_open, self.right_open = get(60, 120, np.mean), get(240, 300, np.mean)

    def enable_cb(self, m):
        self.enabled = m.data
        if not self.enabled: self.stop()

    def return_cb(self, m):
        if m.data:
            self.returning = True
            self.search_yaw = None
            self.dock_stable = 0
            
            # PORT_B 이미 발견했으면 바로 PORT_A로
            if A.PORT_B_ID in self.discovered_ports:
                self.get_logger().info("✅ PORT_B 이미 저장됨 → 바로 PORT_A로 이동")
                self.target, self.state = 'PORT_A', "RETURN_HOME"
            else:
                self.target, self.state = 'PORT_B', "RETURN_PORT_B"

    def cycle_cb(self, m):
        try:
            d = json.loads(m.data)
            if d.get('event') == 'map_saved':
                self.stop()
                self.returning, self.state = False, "HOME_REST"
                self.rest_start = self.get_clock().now()
                self.discovered_ports.clear()
            elif d.get('event') == 'cycle_complete':
                self.stop()
                self.enabled, self.state = False, "STOPPED"
        except: pass

    def aruco_cb(self, m):
        try:
            d = json.loads(m.data)
            if d.get('detected') and d.get('marker_id') in A.DOCK_MARKER_IDS:
                self.marker_detected = True
                self.marker_id = d['marker_id']
                self.marker_x = d.get('center_x', 0)
                self.marker_size = d.get('size', 0)
            else:
                self.marker_detected = False
        except:
            self.marker_detected = False

    def control(self):
        if not self.enabled or self.state == "STOPPED":
            self.stop()
            return
        
        # 상태 로그 (3초마다)
        now = self.get_clock().now().nanoseconds // 1_000_000_000
        if now - self._last_log >= 3:
            self._last_log = now
            ret_info = f"→{self.target}" if self.returning else ""
            marker_info = f"📍M{self.marker_id} sz:{self.marker_size:.0f}" if self.marker_detected else ""
            self.get_logger().info(f"🚗 [{self.state}]{ret_info} {marker_info} F:{self.front:.2f} US:{self.us:.2f}")
        
        # 긴급 정지 (도킹 중 제외)
        is_dock = self.returning and self.marker_detected
        if 0 < self.us < D.US_EMERGENCY and self.front < 0.20 and self.state not in ["BACKUP","TURN"] and not is_dock:
            self.turn_dir = 1 if self.left_open >= self.right_open else -1
            self.state, self.act_start = "BACKUP", self.get_clock().now()
            return
        
        # 상태 머신
        {"RETURN_PORT_B": self.do_dock_b, "RETURN_HOME": self.do_dock_a, "HOME_REST": self.do_rest,
         "TURN": self.do_turn, "BACKUP": self.do_backup, "TURN_180": self.do_turn_180,
         "BACKUP_180": self.do_backup_180,
         "EXPLORE_FOR_PORT_B": self.do_explore_b, "EXPLORE_FOR_PORT_A": self.do_explore_a,
         "DOCK_A": self.do_dock_a, "DOCKED_WAIT": self.do_docked_wait
        }.get(self.state, self.do_explore)()

    def do_explore(self):
        t = Twist()
        fc = min(self.front, self.us if self.us > 0 else 10)
        
        # 탐색 중 PORT_B 마커 발견 → 도킹 시도
        if self.marker_detected and self.marker_id == A.PORT_B_ID and A.PORT_B_ID not in self.discovered_ports:
            if self._dock_approach(A.PORT_B_ID, "PORT_B"):
                self.state, self.act_start, self.turn_dir = "BACKUP", self.get_clock().now(), 1
            return
        
        if 0 < self.us < D.US_CRITICAL or fc < D.FRONT_STOP:
            self.turn_dir = 1 if self.left_open >= self.right_open else -1
            self._start_turn()
            return
        
        # 일반 탐색
        steer = (self.left_open - self.right_open) * 0.15
        t.linear.x = D.SPEED_FAST if fc > D.FRONT_SLOW else D.SPEED_EXPLORE
        t.angular.z = max(-D.STEER_SPEED, min(D.STEER_SPEED, steer))
        self.cmd_pub.publish(t)

    def do_explore_b(self):
        """PORT_B 탐색 자율주행 - 기존 탐험모드 + 마커 감지"""
        if self.marker_detected and self.marker_id == A.PORT_B_ID:
            self.get_logger().info("🎯 PORT_B 마커 발견! → 도킹 시도")
            self.state = "RETURN_PORT_B"
            return
        
        # 기존 탐험 로직 그대로 사용 (TURN/BACKUP 회피 포함)
        t = Twist()
        fc = min(self.front, self.us if self.us > 0 else 10)
        
        if 0 < self.us < D.US_CRITICAL or fc < D.FRONT_STOP:
            self.turn_dir = 1 if self.left_open >= self.right_open else -1
            self._start_turn()
            return
        
        steer = (self.left_open - self.right_open) * 0.15
        t.linear.x = D.SPEED_FAST if fc > D.FRONT_SLOW else D.SPEED_EXPLORE
        t.angular.z = max(-D.STEER_SPEED, min(D.STEER_SPEED, steer))
        self.cmd_pub.publish(t)

    def do_explore_a(self):
        """PORT_A 탐색 자율주행 (HOME 복귀) - 기존 탐험모드 + 마커 감지"""
        if self.marker_detected and self.marker_id == A.PORT_A_ID:
            self.get_logger().info("🎯 PORT_A 마커 발견! → 도킹 시도")
            self.state = "DOCK_A"
            return
        
        # 기존 탐험 로직 그대로 사용 (TURN/BACKUP 회피 포함)
        t = Twist()
        fc = min(self.front, self.us if self.us > 0 else 10)
        
        if 0 < self.us < D.US_CRITICAL or fc < D.FRONT_STOP:
            self.turn_dir = 1 if self.left_open >= self.right_open else -1
            self._start_turn()
            return
        
        steer = (self.left_open - self.right_open) * 0.15
        t.linear.x = D.SPEED_FAST if fc > D.FRONT_SLOW else D.SPEED_EXPLORE
        t.angular.z = max(-D.STEER_SPEED, min(D.STEER_SPEED, steer))
        self.cmd_pub.publish(t)

    def do_dock_b(self):
        """PORT_B 도킹 → 완료 시 후퇴 + 180도 회전 + PORT_A로 이동"""
        # 마커 보이면 도킹 시도
        if self.marker_detected and self.marker_id == A.PORT_B_ID:
            if self._dock_approach(A.PORT_B_ID, "PORT_B"):
                self.get_logger().info("✅ PORT_B 도킹 완료! → 후퇴 + 180도 회전 후 PORT_A로")
                self.dock_stable = 0
                # PORT_B 완료 → 후퇴+회전 후 PORT_A로 이동
                self.state = "BACKUP_180"
                self.act_start = self.get_clock().now()
            return
        
        # 마커 안 보이면 기존 탐색 (do_explore와 동일)
        t = Twist()
        fc = min(self.front, self.us if self.us > 0 else 10)
        if 0 < self.us < D.US_CRITICAL or fc < D.FRONT_STOP:
            self.turn_dir = 1 if self.left_open >= self.right_open else -1
            self._start_turn()
            return
        steer = (self.left_open - self.right_open) * 0.15
        t.linear.x = D.SPEED_FAST if fc > D.FRONT_SLOW else D.SPEED_EXPLORE
        t.angular.z = max(-D.STEER_SPEED, min(D.STEER_SPEED, steer))
        self.cmd_pub.publish(t)

    def do_dock_a(self):
        """PORT_A(HOME) 도킹"""
        # 마커 보이면 도킹 시도
        if self.marker_detected and self.marker_id == A.PORT_A_ID:
            if self._dock_approach(A.PORT_A_ID, "PORT_A"):
                self.get_logger().info("✅ PORT_A(HOME) 도킹 완료! → 휴식")
                self.returning, self.state = False, "HOME_REST"
                self.rest_start = self.get_clock().now()
                self.home_pub.publish(Bool(data=True))
            return
        
        # 마커 안 보이면 기존 탐색
        t = Twist()
        fc = min(self.front, self.us if self.us > 0 else 10)
        if 0 < self.us < D.US_CRITICAL or fc < D.FRONT_STOP:
            self.turn_dir = 1 if self.left_open >= self.right_open else -1
            self._start_turn()
            return
        steer = (self.left_open - self.right_open) * 0.15
        t.linear.x = D.SPEED_FAST if fc > D.FRONT_SLOW else D.SPEED_EXPLORE
        t.angular.z = max(-D.STEER_SPEED, min(D.STEER_SPEED, steer))
        self.cmd_pub.publish(t)

    def _dock_approach(self, mid, name):
        """
        도킹 접근 로직
        1. 마커 못 찾으면 → False 반환 (호출자가 탐색 계속)
        2. 마커 발견 → 정면 정렬 후 직진
        3. 직진 중 장애물 → 회피 후 복귀
        """
        t = Twist()
        
        # 도킹 상태 초기화
        if not hasattr(self, 'dock_phase'):
            self.dock_phase = "FORWARD"  # ALIGN, FORWARD, AVOID
            self.dock_avoid_dir = 0
            self.dock_avoid_start = None
            self.dock_pre_yaw = 0.0
        
        # ========== 마커 못 찾음 → 호출자에게 반환 (기존 탐색 유지) ==========
        if not self.marker_detected or self.marker_id != mid:
            self.dock_phase = "FORWARD"
            self.dock_stable = 0
            return False  # 호출자가 기존 탐색 로직 실행
        
        # ========== 마커 발견됨 ==========
        cx, sz = self.marker_x, self.marker_size
        
        # 도킹 완료 체크 (마커 사이즈 OR 초음파 센서 기반)
        # 조건1: 마커 중앙 정렬 + 적절한 거리
        # 조건2: 초음파 센서가 아주 가까움 (< 0.15m)
        marker_aligned = abs(cx) < A.DOCK_CENTER_TOLERANCE
        marker_close_enough = A.DOCK_SIZE_MIN <= sz <= A.DOCK_SIZE_MAX
        us_very_close = 0 < self.us < 0.15
        
        dock_complete = marker_aligned and (marker_close_enough or us_very_close)
        
        if dock_complete:
            self.dock_stable += 1
            if self.dock_stable >= A.DOCK_STABLE_COUNT:
                self.stop()
                self.discovered_ports[mid] = (self.x, self.y, self.yaw)
                self._pub_port(name)
                self.get_logger().info(f"📤 PORT 발행: {name} x:{self.x:.2f} y:{self.y:.2f}")
                self.dock_stable = 0
                self.dock_phase = "FORWARD"
                return True
        else:
            self.dock_stable = 0
        
        # ========== 2. 정면 정렬 ==========
        if abs(cx) > 0.15:  # 마커가 중앙에서 많이 벗어남
            self.dock_phase = "ALIGN"
            t.linear.x = 0.0
            t.angular.z = -cx * 0.8  # 마커 방향으로 회전
            self.cmd_pub.publish(t)
            return False
        
        # ========== 3. 직진 (장애물 회피 포함) ==========
        
        # ★★★ 회피 중이면 회피 로직만 실행 (장애물 재감지 안함) ★★★
        if self.dock_phase == "AVOID":
            elapsed = (self.get_clock().now() - self.dock_avoid_start).nanoseconds / 1e9
            
            if elapsed < 1.0:
                # 1초간 회피 방향으로 회전
                t.linear.x = 0.0
                t.angular.z = self.dock_avoid_dir * 0.6
            elif elapsed < 2.0:
                # 1초간 전진
                t.linear.x = D.SPEED_EXPLORE
                t.angular.z = 0.0
            elif elapsed < 3.0:
                # 1초간 반대로 회전 (원래 방향 복귀)
                t.linear.x = 0.0
                t.angular.z = -self.dock_avoid_dir * 0.6
            else:
                # 회피 완료 → FORWARD로 복귀
                self.dock_phase = "FORWARD"
                self.get_logger().info("✅ 회피 완료, 도킹 재시도")
            self.cmd_pub.publish(t)
            return False
        
        # ★★★ 도킹 중 장애물 감지 ★★★
        # 마커가 보이면 US 센서 무시 (PORT 스테이션 자체가 US에 감지됨)
        # LiDAR 정면만 체크: 아주 가까울 때만 (0.12m 이하) 회피
        real_obstacle = self.front < 0.12 and sz < A.DOCK_SIZE_MIN * 0.5
        
        # 장애물 감지 → 회피 시작 (마커가 작을 때만 = 아직 멀리 있을 때만)
        if real_obstacle:
            self.dock_phase = "AVOID"
            self.dock_avoid_dir = 1 if self.left_open > self.right_open else -1
            self.dock_avoid_start = self.get_clock().now()
            self.get_logger().info(f"⚠️ 도킹 중 진짜 장애물! {'좌' if self.dock_avoid_dir > 0 else '우'}회피 (F:{self.front:.2f} sz:{sz:.0f})")
            return False
        
        # ========== 정상 직진 ==========
        self.dock_phase = "FORWARD"
        t.angular.z = -cx * A.DOCK_ANGULAR_P  # 미세 조향
        
        if sz < A.DOCK_SIZE_MIN:
            t.linear.x = A.DOCK_SPEED_FORWARD
        elif sz > A.DOCK_SIZE_MAX:
            t.linear.x = -A.DOCK_SPEED_BACK
        else:
            t.linear.x = A.DOCK_SPEED_FORWARD * 0.5
        
        self.cmd_pub.publish(t)
        return False

    def _search_rotate(self, t, name):
        """360도 회전 탐색 - 완료 시 True"""
        if not hasattr(self, 'search_yaw') or self.search_yaw is None:
            self.search_yaw = self.yaw
            self.search_rot = 0.0
            self.search_last = self.yaw
        
        self.search_rot += abs(self.norm(self.yaw - self.search_last))
        self.search_last = self.yaw
        
        if self.search_rot > 2*math.pi:
            self.search_yaw = None
            return True
        
        t.angular.z = 0.4
        self.cmd_pub.publish(t)
        return False

    def _go_to_port(self, t, mid):
        """PORT 위치로 이동 - 장애물 회피 포함"""
        px, py, _ = self.discovered_ports[mid]
        dx, dy = px - self.x, py - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        # PORT 근처 도착 → 전진하며 마커 찾기 (장애물 회피 포함)
        if dist < 0.6:
            self.avoiding = False
            fc = min(self.front, self.us if self.us > 0 else 10)
            if 0 < self.us < D.US_CRITICAL or fc < D.FRONT_STOP:
                # 장애물: 회피 조향하면서 천천히 전진
                t.linear.x = D.SPEED_EXPLORE * 0.3
                t.angular.z = 0.6 if self.left_open > self.right_open else -0.6
            else:
                # 전방 클리어: 전진하면서 마커 찾기
                t.linear.x = D.SPEED_EXPLORE
                t.angular.z = (self.left_open - self.right_open) * 0.1
            self.cmd_pub.publish(t)
            return
        
        # 목표 방향 계산
        target_angle = math.atan2(dy, dx)
        angle_to_target = self.norm(target_angle - self.yaw)
        
        # 전방 거리 (LiDAR + 초음파)
        fc = min(self.front, self.us if self.us > 0 else 10)
        
        # ===== 장애물 감지 시 최소 회피 =====
        obstacle_close = fc < D.FRONT_STOP or (0 < self.us < D.US_CRITICAL)
        
        if obstacle_close and not self.avoiding:
            # 회피 시작: 열린 쪽으로 회피
            self.avoiding = True
            self.avoid_start = self.get_clock().now()
            self.pre_avoid_yaw = target_angle  # 원래 목표 방향 저장
            # 대각선 거리로 회피 방향 결정
            self.avoid_dir = 1 if self.fl > self.fr else -1
            self.get_logger().info(f"⚠️ PORT 이동 중 장애물! {'좌' if self.avoid_dir > 0 else '우'}회피")
        
        if self.avoiding:
            elapsed = (self.get_clock().now() - self.avoid_start).nanoseconds / 1e9
            
            # 회피 시간 초과 (3초) 또는 전방 열림 → 원래 방향으로 복귀
            if elapsed > 3.0 or (fc > D.FRONT_SLOW and self.fl > 0.3 and self.fr > 0.3):
                self.avoiding = False
                self.get_logger().info("✅ 회피 완료, 원래 방향 복귀")
            else:
                # 회피 중: 약간 회전하면서 전진 (45도 정도 틀기)
                t.angular.z = self.avoid_dir * 0.5  # 회피 방향으로 회전
                if fc > 0.25:  # 최소 거리 확보 시 전진
                    t.linear.x = D.SPEED_SLOW
                self.cmd_pub.publish(t)
                return
        
        # ===== 정상 이동: 목표 방향으로 =====
        if abs(angle_to_target) > 0.3:
            t.angular.z = 0.3 * (1 if angle_to_target > 0 else -1)
        else:
            t.linear.x = D.SPEED_EXPLORE if fc > D.FRONT_SLOW else D.SPEED_SLOW
            t.angular.z = angle_to_target * 0.5
        self.cmd_pub.publish(t)

    def _start_turn(self):
        self.state, self.act_start = "TURN", self.get_clock().now()
        self.turn_started = False
        self.turn_rot = 0.0
        self.turn_last = self.yaw

    def do_turn(self):
        if not self.turn_started:
            self.target_yaw = self.norm(self.yaw + math.pi/4 * self.turn_dir)
            self.turn_started = True
        
        # 도킹 중 마커 발견
        if self.returning and self.marker_detected:
            if (self.target == 'PORT_B' and self.marker_id == A.PORT_B_ID) or \
               (self.target == 'PORT_A' and self.marker_id == A.PORT_A_ID):
                self.state = "RETURN_PORT_B" if self.target == 'PORT_B' else "RETURN_HOME"
                return
        
        self.turn_rot += abs(self.norm(self.yaw - self.turn_last))
        self.turn_last = self.yaw
        
        diff = self.norm(self.target_yaw - self.yaw)
        if abs(diff) < math.radians(5):
            if self.front > D.FRONT_STOP + 0.05:
                self.state = "EXPLORE_FOR_PORT_B" if self.returning and self.target == 'PORT_B' else \
                            ("RETURN_HOME" if self.returning else "EXPLORE")
                return
            self.target_yaw = self.norm(self.target_yaw + math.pi/4 * self.turn_dir)
        
        if self.turn_rot > 2*math.pi:
            self.state, self.act_start = "BACKUP", self.get_clock().now()
            return
        
        t = Twist()
        t.angular.z = D.TURN_SPEED * (1 if diff > 0 else -1)
        self.cmd_pub.publish(t)

    def do_backup(self):
        elapsed = (self.get_clock().now() - self.act_start).nanoseconds / 1e9
        if self.rear < D.REAR_STOP or elapsed > 1.0:
            self.turn_dir = -self.turn_dir
            self._start_turn()
            return
        t = Twist()
        t.linear.x = D.BACKUP_SPEED
        self.cmd_pub.publish(t)

    def do_backup_180(self):
        """도킹 후 후퇴 → 180도 회전"""
        elapsed = (self.get_clock().now() - self.act_start).nanoseconds / 1e9
        if self.rear < D.REAR_STOP or elapsed > 1.5:
            # 후퇴 완료 → 180도 회전 시작
            self.state = "TURN_180"
            self.act_start = self.get_clock().now()
            self.target_yaw = self.norm(self.yaw + math.pi)  # 180도
            return
        t = Twist()
        t.linear.x = D.BACKUP_SPEED
        self.cmd_pub.publish(t)

    def do_turn_180(self):
        """180도 회전 후 PORT_A로 이동"""
        diff = self.norm(self.target_yaw - self.yaw)
        if abs(diff) < math.radians(10):
            self.get_logger().info("✅ 180도 회전 완료 → PORT_A로 이동")
            # PORT_A로 이동 (맵 저장을 위해)
            self.target = "PORT_A"
            self.state = "RETURN_HOME"
            self.search_yaw = None
            self.dock_stable = 0
            return
        t = Twist()
        t.angular.z = D.TURN_SPEED if diff > 0 else -D.TURN_SPEED
        self.cmd_pub.publish(t)

    def do_rest(self):
        elapsed = (self.get_clock().now() - self.rest_start).nanoseconds / 1e9
        if elapsed >= 10.0:
            self.state, self.act_start, self.turn_dir = "BACKUP", self.get_clock().now(), 1
            self.discovered_ports.clear()
            return
        self.stop()

    def _pub_port(self, name):
        m = String()
        m.data = json.dumps({"port_name": name, "x": round(self.x,3), "y": round(self.y,3), "yaw": round(self.yaw,3)})
        self.port_pub.publish(m)

    def stop(self):
        try:
            if rclpy.ok(): self.cmd_pub.publish(Twist())
        except: pass

    def do_docked_wait(self):
        """맵 저장/외부 명령 대기 중 정지 상태 유지"""
        self.stop()

    def destroy_node(self):
        self.stop()
        if self.us_sensor:
            try: self.us_sensor.close()
            except: pass
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
        rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
