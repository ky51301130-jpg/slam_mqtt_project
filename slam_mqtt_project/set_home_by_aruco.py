#!/usr/bin/env python3
"""
ArUco 마커로 HOME 위치 설정 스크립트

사용법:
    ros2 launch slam_mqtt_project set_home.launch.py

동작:
1. ArUco ID 0 (HOME 마커) 감지
2. 마커의 3D 위치 (카메라 기준 x, y, z) 측정
3. 현재 로봇 pose (map/odom 기준) + 마커 3D 위치 저장
4. ~/.ros_home_config.json 에 저장
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import cv2
import numpy as np
import json
import os
import math
import time

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False

# ArUco 설정
HOME_MARKER_ID = 0
MARKER_SIZE = 0.10  # 마커 실제 크기 (10cm)
CONFIG_PATH = os.path.expanduser("~/.ros_home_config.json")

# 카메라 캘리브레이션 (Raspberry Pi Camera V2 기본값)
# 실제 캘리브레이션하면 더 정확함
CAMERA_MATRIX = np.array([
    [462.0, 0, 320],
    [0, 462.0, 240],
    [0, 0, 1]
], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float32)


class SetHomeByArucoNode(Node):
    def __init__(self):
        super().__init__('set_home_by_aruco')
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 카메라
        self.camera = None
        self.camera_ok = False
        if PICAMERA_AVAILABLE:
            self._init_camera()
        else:
            self.get_logger().error("❌ Picamera2 not available!")
            return
        
        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # 측정 데이터 수집 (안정화를 위해 여러 번 측정)
        self.measurements = []
        self.max_measurements = 10
        
        # 상태
        self.home_set = False
        self.search_start_time = time.time()
        self.max_search_time = 60.0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🏠 HOME 설정 모드 시작")
        self.get_logger().info(f"   ArUco ID {HOME_MARKER_ID} 마커를 로봇 앞에 놓으세요")
        self.get_logger().info(f"   마커 크기: {MARKER_SIZE*100:.0f}cm")
        self.get_logger().info("=" * 60)
        
        # 10Hz로 ArUco 감지
        self.create_timer(0.1, self.detect_aruco)
    
    def _init_camera(self):
        try:
            self.camera = Picamera2()
            config = self.camera.create_still_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            self.camera.configure(config)
            self.camera.start()
            self.camera_ok = True
            time.sleep(1.0)
            self.get_logger().info("✅ Camera initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Camera init failed: {e}")
    
    def detect_aruco(self):
        if self.home_set or not self.camera_ok:
            return
        
        # 타임아웃 체크
        elapsed = time.time() - self.search_start_time
        if elapsed > self.max_search_time:
            self.get_logger().error(f"❌ ArUco 마커를 {self.max_search_time}초 내 찾지 못함")
            self.shutdown()
            return
        
        try:
            frame = self.camera.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
            
            if ids is not None and HOME_MARKER_ID in ids:
                idx = list(ids.flatten()).index(HOME_MARKER_ID)
                corner = corners[idx]
                
                # 마커 3D 위치 추정 (카메라 기준) - OpenCV 4.7+ 호환
                # 마커 코너를 사용하여 solvePnP로 직접 계산
                obj_points = np.array([
                    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
                ], dtype=np.float32)
                
                img_points = corner[0].astype(np.float32)
                
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, CAMERA_MATRIX, DIST_COEFFS
                )
                
                if not success:
                    return
                
                tvec = tvec.flatten()  # [x, y, z] in meters (카메라 기준)
                rvec = rvec.flatten()
                
                # 마커 중심 (픽셀)
                center_x = np.mean(corner[0][:, 0])
                center_y = np.mean(corner[0][:, 1])
                
                # 마커 크기 (픽셀)
                marker_size_px = np.linalg.norm(corner[0][0] - corner[0][1])
                
                # 측정 데이터 수집
                self.measurements.append({
                    'tvec': tvec.tolist(),
                    'rvec': rvec.tolist(),
                    'center_px': [center_x, center_y],
                    'size_px': marker_size_px
                })
                
                self.get_logger().info(
                    f"📍 측정 {len(self.measurements)}/{self.max_measurements}: "
                    f"x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}m"
                )
                
                # 충분한 측정이 모이면 저장
                if len(self.measurements) >= self.max_measurements:
                    self.save_home_config()
            else:
                if int(elapsed) % 5 == 0:
                    self.get_logger().info(f"🔍 마커 탐색 중... ({int(elapsed)}s)")
        
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
    
    def save_home_config(self):
        """HOME 설정 저장"""
        try:
            # 평균값 계산 (노이즈 제거)
            tvecs = np.array([m['tvec'] for m in self.measurements])
            rvecs = np.array([m['rvec'] for m in self.measurements])
            sizes = np.array([m['size_px'] for m in self.measurements])
            
            avg_tvec = np.mean(tvecs, axis=0)
            avg_rvec = np.mean(rvecs, axis=0)
            avg_size = np.mean(sizes)
            std_tvec = np.std(tvecs, axis=0)
            
            # 로봇 현재 위치 (odom 기준)
            robot_pose = self.get_robot_pose()
            
            # 설정 저장
            config = {
                "marker_id": HOME_MARKER_ID,
                "marker_size_m": MARKER_SIZE,
                "marker_3d_pose": {
                    "x": round(float(avg_tvec[0]), 4),  # 카메라 기준 좌우 (+ = 오른쪽)
                    "y": round(float(avg_tvec[1]), 4),  # 카메라 기준 상하 (+ = 아래)
                    "z": round(float(avg_tvec[2]), 4),  # 카메라 기준 거리 (앞)
                },
                "marker_rotation": {
                    "rx": round(float(avg_rvec[0]), 4),
                    "ry": round(float(avg_rvec[1]), 4),
                    "rz": round(float(avg_rvec[2]), 4),
                },
                "marker_size_px": round(float(avg_size), 1),
                "measurement_std": {
                    "x": round(float(std_tvec[0]), 4),
                    "y": round(float(std_tvec[1]), 4),
                    "z": round(float(std_tvec[2]), 4),
                },
                "robot_pose_at_home": robot_pose,
                "home_tolerance": 0.3,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "measurements_count": len(self.measurements)
            }
            
            with open(CONFIG_PATH, 'w') as f:
                json.dump(config, f, indent=2)
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ HOME 설정 저장 완료!")
            self.get_logger().info(f"   마커 3D 위치 (카메라 기준):")
            self.get_logger().info(f"     X: {avg_tvec[0]:+.4f} m (좌우, +오른쪽)")
            self.get_logger().info(f"     Y: {avg_tvec[1]:+.4f} m (상하, +아래)")
            self.get_logger().info(f"     Z: {avg_tvec[2]:+.4f} m (거리)")
            self.get_logger().info(f"   마커 크기: {avg_size:.1f} px")
            if robot_pose:
                self.get_logger().info(f"   로봇 위치: ({robot_pose['x']:.3f}, {robot_pose['y']:.3f})")
            self.get_logger().info(f"   저장: {CONFIG_PATH}")
            self.get_logger().info("=" * 60)
            self.get_logger().info("🚀 이제 SLAM 모드를 시작할 수 있습니다!")
            
            self.home_set = True
            self.shutdown()
            
        except Exception as e:
            self.get_logger().error(f"❌ 저장 실패: {e}")
    
    def get_robot_pose(self):
        """현재 로봇 위치 (odom 또는 map 기준)"""
        try:
            # map 먼저 시도, 없으면 odom
            for frame in ['map', 'odom']:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        frame, 'base_footprint',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    q = tf.transform.rotation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    return {
                        'frame': frame,
                        'x': round(tf.transform.translation.x, 4),
                        'y': round(tf.transform.translation.y, 4),
                        'yaw': round(yaw, 4),
                        'yaw_deg': round(math.degrees(yaw), 1)
                    }
                except:
                    continue
            return None
        except:
            return None
    
    def shutdown(self):
        """정리 및 종료"""
        if self.camera:
            try:
                self.camera.stop()
            except:
                pass
        self.create_timer(2.0, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = SetHomeByArucoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
