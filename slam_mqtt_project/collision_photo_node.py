#!/usr/bin/env python3
"""SLAM 충돌 사진 + ArUco 감지 노드 (간소화 버전)

기능:
- 초음파 충돌 감지 시 사진 촬영
- ArUco 마커 감지 → HOME/PORT 발행
- SLAM 모드에서 PORT 좌표 자동 저장
- Flask HTTP 서버 (사진 다운로드)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os
import time
import threading
import json
import math
import numpy as np
from datetime import datetime

cv2 = None  # Lazy import

try:
    from flask import Flask, jsonify, send_file
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

from slam_mqtt_project.topics import ROS, MQTT as MQTT_TOPICS, NET, ARUCO

# ====== 상수 ======
PHOTO_DIR = "/home/pinky/collision_photos"
MAX_PHOTOS = 30
COLLISION_DISTANCE = 0.25
COOLDOWN_TIME = 3.0
HTTP_PORT = NET.COLLISION_PORT

# ArUco 도킹 설정 - topics.py에서 가져옴
ARUCO_DETECT_RATE = 10.0  # 10 FPS (0.1초마다 감지)
DOCK_MARKER_IDS = ARUCO.DOCK_MARKER_IDS
ARUCO_PORT_MAP = ARUCO.PORT_MAP

# 측정된 도킹 목표값 (캘리브레이션: 2025-12-17)
DOCK_TARGET_X = 0.0       # 마커 중앙 정렬 (normalized: -1 ~ 1)
DOCK_TARGET_SIZE = ARUCO.DOCK_SIZE_TARGET
DOCK_CENTER_TOLERANCE = ARUCO.DOCK_CENTER_TOLERANCE
DOCK_SIZE_TOLERANCE = 30

# PORT 저장
PORT_GOALS_FILE = "/home/pinky/saved_maps/port_goals.json"


class CollisionPhotoNode(Node):
    def __init__(self):
        super().__init__('collision_photo_node')
        
        # 상태
        self.last_photo_time = 0.0
        self.camera = None
        self.camera_ok = False
        self.aruco_dict = None
        self.aruco_params = None
        self._last_aruco_log = 0
        self.collision_enabled = True
        
        # PORT 저장
        self.port_goals = self._load_port_goals()
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.port_save_enabled = False
        self.saved_ports = set()
        
        os.makedirs(PHOTO_DIR, exist_ok=True)
        
        # 카메라 초기화
        if PICAMERA_AVAILABLE:
            self._init_camera()
        
        # Subscribers
        self.create_subscription(Float32, ROS.ULTRASONIC, self.ultrasonic_cb, 10)
        self.create_subscription(Bool, ROS.COLLISION_TRIGGER, self.trigger_cb, 10)
        self.create_subscription(Odometry, ROS.ODOM, self.odom_cb, 10)
        self.create_subscription(String, ROS.ROBOT_MODE, self.mode_cb, 10)
        
        # Publishers
        self.photo_pub = self.create_publisher(String, ROS.COLLISION_PHOTO_READY, 10)
        self.image_pub = self.create_publisher(CompressedImage, ROS.COLLISION_IMAGE, 1)
        self.aruco_pub = self.create_publisher(String, ROS.ARUCO_HOME_DETECTED, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, ROS.CMD_VEL, 10)
        
        # MQTT
        self.mqtt = None
        if MQTT_AVAILABLE:
            self._init_mqtt()
        
        # Flask
        if FLASK_AVAILABLE:
            self._start_flask()
        
        # ArUco 타이머
        self.create_timer(1.0 / ARUCO_DETECT_RATE, self.detect_aruco)
        
        self.get_logger().info(f"📷 Collision Photo Node Started")
        self.get_logger().info(f"   Photo Dir: {PHOTO_DIR}, HTTP Port: {HTTP_PORT}")

    def _init_camera(self):
        global cv2
        if cv2 is None:
            import cv2 as _cv2
            cv2 = _cv2
        try:
            self.camera = Picamera2()
            config = self.camera.create_still_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            self.camera.configure(config)
            self.camera.start()
            self.camera_ok = True
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.get_logger().info("✅ Camera + ArUco initialized")
        except Exception as e:
            self.get_logger().error(f"Camera init failed: {e}")

    def _init_mqtt(self):
        try:
            self.mqtt = mqtt.Client()
            self.mqtt.connect_async(MQTT_TOPICS.HOST, MQTT_TOPICS.PORT, 60)
            self.mqtt.loop_start()
        except Exception as e:
            self.get_logger().warn(f"MQTT init failed: {e}")

    def _load_port_goals(self) -> dict:
        try:
            if os.path.exists(PORT_GOALS_FILE):
                with open(PORT_GOALS_FILE, 'r') as f:
                    data = json.load(f)
                    self.get_logger().info(f"📂 Loaded {len(data)} port goals")
                    return data
        except Exception as e:
            self.get_logger().warn(f"Failed to load port goals: {e}")
        return {}

    def _save_port_goals(self):
        try:
            os.makedirs(os.path.dirname(PORT_GOALS_FILE), exist_ok=True)
            with open(PORT_GOALS_FILE, 'w') as f:
                json.dump(self.port_goals, f, indent=2)
            self.get_logger().info(f"💾 Saved: {list(self.port_goals.keys())}")
        except Exception as e:
            self.get_logger().error(f"Save failed: {e}")

    def _start_flask(self):
        app = Flask(__name__)
        
        @app.route('/photos')
        def list_photos():
            files = sorted([f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')])
            return jsonify(files)
        
        @app.route('/photos/<filename>')
        def get_photo(filename):
            path = os.path.join(PHOTO_DIR, filename)
            if os.path.exists(path):
                return send_file(path, mimetype='image/jpeg')
            return "Not found", 404
        
        @app.route('/latest')
        def latest():
            files = sorted([f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')])
            if files:
                return send_file(os.path.join(PHOTO_DIR, files[-1]), mimetype='image/jpeg')
            return "No photos", 404
        
        threading.Thread(target=lambda: app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True), daemon=True).start()
        self.get_logger().info(f"🌐 HTTP server on port {HTTP_PORT}")

    # ====== Callbacks ======
    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def mode_cb(self, msg):
        mode = msg.data.upper()
        was_enabled = self.port_save_enabled
        self.port_save_enabled = (mode == "SLAM")
        if self.port_save_enabled and not was_enabled:
            self.saved_ports.clear()
            self.get_logger().info("📍 PORT save ENABLED (SLAM mode)")

    def ultrasonic_cb(self, msg):
        if not self.collision_enabled:
            return
        if msg.data < COLLISION_DISTANCE:
            self.capture_photo(msg.data)

    def trigger_cb(self, msg):
        if msg.data:
            self.capture_photo(0.0)

    def capture_photo(self, distance: float):
        now = time.time()
        if now - self.last_photo_time < COOLDOWN_TIME or not self.camera_ok:
            return
        
        try:
            frame = self.camera.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"collision_{timestamp}_{distance:.2f}m.jpg"
            filepath = os.path.join(PHOTO_DIR, filename)
            
            cv2.imwrite(filepath, frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.last_photo_time = now
            
            self.get_logger().info(f"📸 Photo saved: {filename}")
            
            # 오래된 사진 삭제
            self._cleanup_old_photos()
            
            # 발행
            self.photo_pub.publish(String(data=json.dumps({
                'filename': filename,
                'distance': distance,
                'timestamp': timestamp
            })))
            
            if self.mqtt:
                self.mqtt.publish(MQTT_TOPICS.COLLISION_PHOTO, filename)
        except Exception as e:
            self.get_logger().error(f"Photo capture failed: {e}")

    def _cleanup_old_photos(self):
        files = sorted([f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')])
        while len(files) > MAX_PHOTOS:
            old = files.pop(0)
            try:
                os.remove(os.path.join(PHOTO_DIR, old))
                self.get_logger().info(f"🗑️ Removed: {old}")
            except:
                pass

    def detect_aruco(self):
        if not self.camera_ok or self.aruco_dict is None:
            return
        
        try:
            frame = self.camera.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    marker_id = int(marker_id)  # numpy.int32 → int (JSON 직렬화용)
                    corner = corners[i][0]
                    
                    # 중심/크기 계산
                    center_x = np.mean(corner[:, 0])
                    center_y = np.mean(corner[:, 1])
                    normalized_x = (center_x - 320) / 320
                    normalized_y = (center_y - 240) / 240
                    
                    width = np.linalg.norm(corner[0] - corner[1])
                    height = np.linalg.norm(corner[1] - corner[2])
                    marker_size = (width + height) / 2
                    
                    # 도킹용 마커 발행 (PORT_A=0, PORT_B=1만)
                    if marker_id in DOCK_MARKER_IDS:
                        port_name = ARUCO_PORT_MAP.get(marker_id, f"ID:{marker_id}")
                        self.aruco_pub.publish(String(data=json.dumps({
                            "detected": True,
                            "center_x": round(float(normalized_x), 3),
                            "center_y": round(float(normalized_y), 3),
                            "size": round(float(marker_size), 1),
                            "marker_id": marker_id,
                            "port_name": port_name
                        })))
                    
                    # PORT 저장 시도
                    self._try_save_port(marker_id, normalized_x, marker_size, frame, corner)
                    
                    # 로그 (PORT_A, B만 0.5초마다)
                    now = time.time()
                    if marker_id in DOCK_MARKER_IDS and now - self._last_aruco_log > 0.5:
                        name = ARUCO_PORT_MAP.get(marker_id, f"ID:{marker_id}")
                        self.get_logger().info(f"🎯 {name}: x={normalized_x:.2f}, size={marker_size:.0f}px")
                        self._last_aruco_log = now
            else:
                self.aruco_pub.publish(String(data=json.dumps({"detected": False})))
        except Exception as e:
            self.get_logger().error(f"ArUco detect error: {e}")

    def _try_save_port(self, marker_id, center_x, marker_size, frame, corner):
        if not self.port_save_enabled:
            return
        
        # PORT_A(0), PORT_B(1) 둘 다 저장
        if marker_id not in DOCK_MARKER_IDS:
            return
        
        port_name = ARUCO_PORT_MAP.get(marker_id)
        if not port_name or port_name in self.saved_ports:
            return
        
        # 정렬 체크
        if abs(center_x) > DOCK_CENTER_TOLERANCE:
            return
        if abs(marker_size - DOCK_TARGET_SIZE) > DOCK_SIZE_TOLERANCE:
            return
        
        # 저장
        self.port_goals[port_name] = {
            "x": round(self.current_x, 4),
            "y": round(self.current_y, 4),
            "yaw": round(self.current_yaw, 4),
            "aruco_id": marker_id,
            "timestamp": time.strftime('%Y-%m-%d %H:%M:%S')
        }
        self.saved_ports.add(port_name)
        self._save_port_goals()
        
        # 사진
        try:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            pts = corner.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame_bgr, [pts], True, (0, 255, 0), 3)
            cx, cy = int(np.mean(corner[:, 0])), int(np.mean(corner[:, 1]))
            cv2.putText(frame_bgr, f"{port_name} SAVED", (cx-50, cy-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            filepath = os.path.join(PHOTO_DIR, f"port_{port_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
            cv2.imwrite(filepath, frame_bgr)
        except:
            pass
        
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"✅ PORT SAVED: {port_name} (ID:{marker_id})")
        self.get_logger().info(f"   Pos: ({self.current_x:.3f}, {self.current_y:.3f})")
        self.get_logger().info("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionPhotoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
