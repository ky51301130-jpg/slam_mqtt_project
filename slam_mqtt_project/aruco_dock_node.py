#!/usr/bin/env python3
"""
=============================================================================
                        ArUco 도킹 노드
=============================================================================
서버에서 YOLO로 QR/마커 감지 → MQTT로 PORT_A/PORT_B 수신
로봇에서 ArUco로 정밀 위치(x,y,yaw) 계산 → 도킹 제어

역할:
  1. MQTT에서 PORT 감지 신호 수신 (AI 서버 YOLO)
  2. ArUco 마커로 정밀 거리/각도 계산
  3. 정밀 접근 제어 (cmd_vel)
  4. 도착 시 위치 저장 + 도착 신호 발행
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2
import cv2.aruco as aruco
import numpy as np
import json
import math
import threading
import os
from typing import Optional, Tuple, Dict

from slam_mqtt_project.topics import ROS, ARUCO

# 파일 경로
PORT_GOALS_FILE = "/home/pinky/saved_maps/port_goals.json"
CALIBRATION_FILE = "/home/pinky/saved_maps/aruco_calibration.json"


class ArucoDockNode(Node):
    """ArUco 기반 정밀 도킹 노드"""
    
    # ArUco 마커 ID → 포트 매핑 (topics.py에서 가져옴)
    MARKER_PORT_MAP = ARUCO.PORT_MAP
    
    def __init__(self):
        super().__init__('aruco_dock_node')
        
        # ======================== 캘리브레이션 로드 ========================
        self._load_calibration()
        
        # ======================== 상태 변수 ========================
        self.docking_enabled = False  # 도킹 모드 활성화
        self.target_port: Optional[str] = None  # 목표 포트
        self.target_marker_id: Optional[int] = None  # 목표 마커 ID
        self.current_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self.port_goals: Dict[str, Dict] = {}  # 저장된 포트 Goal 좌표
        self.learning_mode = False  # 학습 모드 (새 포트 위치 저장)
        
        # 저장된 포트 위치 로드
        self._load_port_goals()
        
        # ======================== ArUco 설정 ========================
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # ======================== 이미지 토픽 수신용 ========================
        self.current_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        
        # ======================== ROS2 퍼블리셔 ========================
        self.cmd_vel_pub = self.create_publisher(Twist, ROS.CMD_VEL, 10)
        self.arrival_pub = self.create_publisher(String, ARUCO.PORT_ARRIVAL, 10)
        self.dock_status_pub = self.create_publisher(String, ARUCO.DOCK_STATUS, 10)
        self.port_goals_pub = self.create_publisher(String, ARUCO.PORT_GOALS, 10)
        
        # ======================== ROS2 구독자 ========================
        # 카메라 이미지 토픽 구독 (camera_stream_node에서 발행)
        self.create_subscription(
            CompressedImage, ROS.CAMERA_IMAGE + '/compressed',
            self.image_callback, 1
        )
        
        # Nav2에서 목표 포트 수신
        self.create_subscription(
            String, ARUCO.TARGET_PORT,
            self.target_port_callback, 10
        )
        
        # 오도메트리 (현재 위치)
        self.create_subscription(
            Odometry, ROS.ODOM,
            self.odom_callback, 10
        )
        
        # 도킹 활성화/비활성화 (Nav2 도착 후 활성화)
        self.create_subscription(
            Bool, ARUCO.DOCK_ENABLE,
            self.dock_enable_callback, 10
        )
        
        # 학습 모드 (새 포트 위치 저장)
        self.create_subscription(
            Bool, ARUCO.LEARNING_MODE,
            self.learning_mode_callback, 10
        )
        
        # ======================== 타이머 ========================
        # 10Hz로 ArUco 감지 및 도킹 제어 (Nav2 모드에서만 작동)
        self.create_timer(0.1, self.dock_control_loop)
        
        self.get_logger().info('🎯 ArUco Dock Node Started (Nav2 mode only)')
        self.get_logger().info(f'   Marker Size: {self.MARKER_SIZE*100:.0f}cm')
        self.get_logger().info(f'   Dock Distance: {self.DOCK_DISTANCE*100:.0f}cm')
        self.get_logger().info(f'   Saved Ports: {list(self.port_goals.keys())}')
    
    def _load_calibration(self):
        """캘리브레이션 파일에서 설정 로드"""
        # 기본값 - topics.py에서 가져옴
        self.MARKER_SIZE = 0.10  # 10cm
        self.DOCK_DISTANCE = 0.30  # 30cm
        self.DOCK_TOLERANCE = 0.05  # 5cm
        self.CENTER_TOLERANCE = ARUCO.DOCK_CENTER_TOLERANCE
        self.ANGLE_TOLERANCE = 5.0  # 5도
        self.LINEAR_SPEED = ARUCO.DOCK_SPEED_FORWARD
        self.ANGULAR_SPEED = ARUCO.DOCK_ANGULAR_P
        
        # 기본 카메라 캘리브레이션
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5,), dtype=np.float32)
        
        # 파일에서 로드
        try:
            if os.path.exists(CALIBRATION_FILE):
                with open(CALIBRATION_FILE, 'r') as f:
                    config = json.load(f)
                
                # 카메라 캘리브레이션
                if 'camera_matrix' in config and config['camera_matrix']:
                    self.camera_matrix = np.array(config['camera_matrix'], dtype=np.float32)
                if 'dist_coeffs' in config and config['dist_coeffs']:
                    self.dist_coeffs = np.array(config['dist_coeffs'], dtype=np.float32)
                
                # 도킹 파라미터
                self.MARKER_SIZE = config.get('marker_size', self.MARKER_SIZE)
                self.DOCK_DISTANCE = config.get('dock_distance', self.DOCK_DISTANCE)
                self.DOCK_TOLERANCE = config.get('dock_tolerance', self.DOCK_TOLERANCE)
                self.CENTER_TOLERANCE = config.get('center_tolerance', self.CENTER_TOLERANCE)
                self.ANGLE_TOLERANCE = config.get('angle_tolerance', self.ANGLE_TOLERANCE)
                
                self.get_logger().info(f'📂 Loaded calibration from {CALIBRATION_FILE}')
            else:
                self.get_logger().info('📐 Using default calibration (run aruco_calibration to configure)')
        except Exception as e:
            self.get_logger().warn(f'Calibration load failed: {e}, using defaults')
    
    def _load_port_goals(self):
        """저장된 포트 Goal 좌표 로드"""
        try:
            if os.path.exists(PORT_GOALS_FILE):
                with open(PORT_GOALS_FILE, 'r') as f:
                    self.port_goals = json.load(f)
                self.get_logger().info(f'📂 Loaded {len(self.port_goals)} port goals')
        except Exception as e:
            self.get_logger().warn(f'Failed to load port goals: {e}')
            self.port_goals = {}
    
    def _save_port_goals(self):
        """포트 Goal 좌표 파일로 저장"""
        try:
            os.makedirs(os.path.dirname(PORT_GOALS_FILE), exist_ok=True)
            with open(PORT_GOALS_FILE, 'w') as f:
                json.dump(self.port_goals, f, indent=2)
            self.get_logger().info(f'💾 Saved {len(self.port_goals)} port goals')
            
            # 발행도 함께
            self.port_goals_pub.publish(String(data=json.dumps(self.port_goals)))
        except Exception as e:
            self.get_logger().error(f'Failed to save port goals: {e}')
    
    def image_callback(self, msg: CompressedImage):
        """카메라 이미지 토픽 수신 (camera_stream_node에서)"""
        try:
            # JPEG 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # 이미지 회전 (camera_stream_node에서 이미 회전됨)
                with self.frame_lock:
                    self.current_frame = frame
        except Exception as e:
            self.get_logger().warn(f'Image decode failed: {e}')
    
    def target_port_callback(self, msg: String):
        """목표 포트 설정 (nav2_goal_node에서 수신)"""
        self.target_port = msg.data
        # 해당 포트의 마커 ID 찾기
        for marker_id, port_name in self.MARKER_PORT_MAP.items():
            if port_name == self.target_port:
                self.target_marker_id = marker_id
                break
        self.get_logger().info(f'📍 Target port set: {self.target_port} (marker ID: {self.target_marker_id})')
    
    def odom_callback(self, msg: Odometry):
        """현재 위치 업데이트"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Quaternion → Yaw
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = (pos.x, pos.y, yaw)
    
    def dock_enable_callback(self, msg: Bool):
        """도킹 모드 활성화/비활성화"""
        self.docking_enabled = msg.data
        if not self.docking_enabled:
            self.stop_robot()
            self.get_logger().info('🛑 Docking disabled')
    
    def learning_mode_callback(self, msg: Bool):
        """학습 모드 활성화/비활성화"""
        self.learning_mode = msg.data
        if self.learning_mode:
            self.get_logger().info('📚 Learning mode ENABLED - will save new port positions')
        else:
            self.get_logger().info('📚 Learning mode DISABLED')

    def detect_aruco(self) -> Optional[Tuple[int, float, float, float, float]]:
        """
        ArUco 마커 감지 및 위치 계산
        
        Returns:
            (marker_id, x_offset, y_offset, distance, yaw) or None
            - x_offset: 좌우 오프셋 (m), 양수=오른쪽
            - y_offset: 상하 오프셋 (m)
            - distance: 정면 거리 (m)
            - yaw: 마커 각도 (degrees)
        """
        with self.frame_lock:
            if self.current_frame is None:
                return None
            frame = self.current_frame.copy()
        
        # Grayscale 변환 (이미 BGR 형식)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ArUco 감지
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if ids is None or len(ids) == 0:
            return None
        
        # 목표 마커 찾기
        target_idx = None
        if self.target_marker_id is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.target_marker_id:
                    target_idx = i
                    break
        else:
            # 목표 없으면 첫 번째 마커 사용
            target_idx = 0
        
        if target_idx is None:
            return None
        
        marker_id = ids[target_idx][0]
        marker_corners = corners[target_idx]
        
        # 3D 위치 추정
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            [marker_corners], self.MARKER_SIZE, 
            self.camera_matrix, self.dist_coeffs
        )
        
        # tvecs: [x, y, z] in meters
        x_offset = tvecs[0][0][0]  # 좌우 (양수=오른쪽)
        y_offset = tvecs[0][0][1]  # 상하
        distance = tvecs[0][0][2]  # 정면 거리
        
        # 각도 계산 (yaw)
        rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
        yaw_rad = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        yaw_deg = math.degrees(yaw_rad)
        
        return (marker_id, x_offset, y_offset, distance, yaw_deg)
    
    def dock_control_loop(self):
        """도킹 제어 루프 (10Hz)"""
        if not self.docking_enabled:
            return
        
        # ArUco 감지
        result = self.detect_aruco()
        
        if result is None:
            # 마커 안 보임 → 정지하고 대기
            self.stop_robot()
            return
        
        marker_id, x_offset, y_offset, distance, yaw = result
        port_name = self.MARKER_PORT_MAP.get(marker_id, f"MARKER_{marker_id}")
        
        # 상태 발행
        status = {
            "port": port_name,
            "distance": round(distance, 3),
            "x_offset": round(x_offset, 3),
            "yaw": round(yaw, 1)
        }
        self.dock_status_pub.publish(String(data=json.dumps(status)))
        
        # 도킹 완료 체크
        if self.is_docked(x_offset, distance, yaw):
            self.on_dock_complete(port_name)
            return
        
        # 도킹 제어
        cmd = Twist()
        
        # 1. 중앙 정렬 (x_offset → angular.z)
        if abs(x_offset) > self.CENTER_TOLERANCE:
            # 마커가 오른쪽에 있으면 오른쪽으로 회전 (음수)
            cmd.angular.z = -self.ANGULAR_SPEED * np.sign(x_offset)
            cmd.angular.z = np.clip(cmd.angular.z, -self.ANGULAR_SPEED, self.ANGULAR_SPEED)
        
        # 2. 거리 조절 (distance → linear.x)
        distance_error = distance - self.DOCK_DISTANCE
        if abs(distance_error) > self.DOCK_TOLERANCE:
            if distance_error > 0:
                # 아직 멀다 → 전진
                cmd.linear.x = self.LINEAR_SPEED
            else:
                # 너무 가깝다 → 후진
                cmd.linear.x = -self.LINEAR_SPEED * 0.5
        
        # 중앙 정렬이 안 되면 전진 속도 줄임
        if abs(x_offset) > self.CENTER_TOLERANCE * 2:
            cmd.linear.x *= 0.3
        
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info(
            f'🎯 Docking: dist={distance:.2f}m, x={x_offset:.2f}m, yaw={yaw:.1f}°'
        )
    
    def is_docked(self, x_offset: float, distance: float, yaw: float) -> bool:
        """도킹 완료 판정"""
        centered = abs(x_offset) < self.CENTER_TOLERANCE
        close_enough = abs(distance - self.DOCK_DISTANCE) < self.DOCK_TOLERANCE
        aligned = abs(yaw) < self.ANGLE_TOLERANCE
        
        return centered and close_enough and aligned
    
    def on_dock_complete(self, port_name: str):
        """도킹 완료 처리"""
        self.stop_robot()
        self.docking_enabled = False
        
        # 현재 위치 저장 (학습 모드이거나, 아직 저장 안 된 포트)
        if self.current_pose:
            x, y, yaw = self.current_pose
            
            # 학습 모드이거나 새로운 포트면 저장
            if self.learning_mode or port_name not in self.port_goals:
                self.port_goals[port_name] = {
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "yaw": round(yaw, 3)
                }
                self._save_port_goals()
                self.get_logger().info(
                    f'📍 {port_name} goal SAVED: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°'
                )
            else:
                self.get_logger().info(
                    f'📍 {port_name} position: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°'
                )
        
        # 도착 신호 발행
        arrival_msg = String()
        arrival_msg.data = json.dumps({
            "port": port_name,
            "position": {
                "x": self.current_pose[0] if self.current_pose else 0,
                "y": self.current_pose[1] if self.current_pose else 0,
                "yaw": self.current_pose[2] if self.current_pose else 0
            },
            "saved_goal": self.port_goals.get(port_name, {})
        })
        self.arrival_pub.publish(arrival_msg)
        
        self.get_logger().info(f'✅ ARRIVED at {port_name}!')
    
    def stop_robot(self):
        """로봇 정지"""
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass
    
    def destroy_node(self):
        """노드 종료"""
        try:
            self.stop_robot()
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
