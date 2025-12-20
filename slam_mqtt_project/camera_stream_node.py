#!/usr/bin/env python3
"""
📷 Nav2용 카메라 스트리밍 + 장애물 분석 노드 (1fps)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
기능:
- Flask 웹서버로 카메라 이미지 제공 (http://192.168.0.5:5000/image.jpg)
- 1fps 저속 캡처로 CPU 부하 최소화
- 외부 YOLO 서버(192.168.0.3)에서 분석 가능
- MQTT로 감지 결과 수신 및 Nav2 연동
- 장애물 타입에 따른 처리:
  * wall: 정상 회피 네비게이션
  * temporary: 로그 기록 후 재시도 대기
  * qr_port: 포트 도착 알림 → 도킹/적재/하역

엔드포인트:
- GET /image.jpg : 현재 프레임 (JPEG)
- GET /status    : 상태 정보 (JSON)
- GET /          : 웹 뷰어

토픽 (발행):
- /camera/image/compressed : CompressedImage 발행
- /nav2/obstacle_action    : Nav2에 장애물 처리 명령
- /nav2/port_arrival       : 포트 도착 알림

토픽 (수신):
- /mqtt/pinky/detection    : YOLO 감지 결과
- /mqtt/pinky/obstacle_type: 장애물 타입

사용법:
  ros2 run slam_mqtt_project camera_stream_node
  
  # 파라미터 조정
  ros2 run slam_mqtt_project camera_stream_node --ros-args \\
    -p fps:=1.0 -p width:=320 -p height:=240 -p port:=5000
"""
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
import cv2
import numpy as np
import time
import threading
import json
from datetime import datetime

# 중앙 집중식 토픽 관리
from slam_mqtt_project.topics import ROS, NET, ARUCO

# Flask 웹서버
try:
    from flask import Flask, Response, jsonify
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

# Picamera2
try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False


class CameraStreamNode(Node):
    """Nav2용 카메라 스트리밍 + AI 분석 연동 노드"""
    
    # 장애물 타입 상수
    OBSTACLE_WALL = "wall"
    OBSTACLE_TEMP = "temporary"
    OBSTACLE_QR = "qr_port"
    
    def __init__(self):
        super().__init__('camera_stream_node')
        
        # ===== 파라미터 =====
        self.declare_parameter('fps', 1.0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('jpeg_quality', 50)
        self.declare_parameter('port', NET.STREAMING_PORT)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('target_port', '')  # 목표 포트 (e.g., "PORT_A")
        
        self.fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.target_port = self.get_parameter('target_port').value
        
        # ===== 상태 변수 =====
        self.current_frame = None
        self.current_jpeg = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        self.last_capture_time = 0
        self.start_time = time.time()
        self.running = True
        
        # 장애물/감지 상태
        self.last_obstacle_type = 'unknown'
        self.last_detection = None
        self.detected_qr_codes = []
        self.port_arrival_triggered = False
        self.temp_obstacle_count = 0
        
        # ===== 카메라 초기화 =====
        self.camera = None
        self.camera_ok = False
        if PICAMERA_AVAILABLE:
            self._init_camera()
        else:
            self.get_logger().error('❌ Picamera2 not available')
        
        # ===== ROS2 Publishers =====
        self.pub_image = self.create_publisher(
            CompressedImage, ROS.CAMERA_IMAGE + '/compressed', 1
        )
        self.pub_obstacle_action = self.create_publisher(
            String, ROS.OBSTACLE_ACTION, 10
        )
        self.pub_port_arrival = self.create_publisher(
            String, ARUCO.PORT_ARRIVAL, 10
        )
        self.pub_stop_robot = self.create_publisher(
            Bool, ROS.STOP_ROBOT, 10
        )
        
        # ===== ROS2 Subscribers (YOLO 결과) =====
        self.sub_detection = self.create_subscription(
            String, ROS.AI_DETECTION, self.detection_callback, 10
        )
        self.sub_obstacle = self.create_subscription(
            String, ROS.AI_OBSTACLE_TYPE, self.obstacle_callback, 10
        )
        self.sub_target_port = self.create_subscription(
            String, ARUCO.TARGET_PORT, self.target_port_callback, 10
        )
        
        # ===== 캡처 스레드 =====
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        
        # ===== Flask 웹서버 =====
        if FLASK_AVAILABLE:
            self._start_flask_server()
        else:
            self.get_logger().error('❌ Flask not available - pip install flask')
        
        # ===== 상태 로그 타이머 =====
        self.log_timer = self.create_timer(10.0, self._log_status)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('📷 Nav2 Camera Stream Server Started')
        self.get_logger().info(f'   URL: http://0.0.0.0:{self.port}/image.jpg')
        self.get_logger().info(f'   Resolution: {self.width}x{self.height}')
        self.get_logger().info(f'   FPS: {self.fps}, Quality: {self.jpeg_quality}%')
        self.get_logger().info(f'   Camera: {"✅" if self.camera_ok else "❌"}')
        self.get_logger().info('=' * 60)
    
    def _init_camera(self):
        """Picamera2 초기화"""
        try:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={"format": "RGB888", "size": (self.width, self.height)}
            )
            self.camera.configure(config)
            self.camera.start()
            self.camera_ok = True
            self.get_logger().info('📷 Camera initialized')
        except Exception as e:
            self.camera_ok = False
            self.get_logger().error(f'❌ Camera init error: {e}')
    
    def _capture_loop(self):
        """1fps 캡처 루프"""
        interval = 1.0 / self.fps
        
        while self.running:
            try:
                if self.camera_ok and self.camera is not None:
                    frame = self.camera.capture_array()
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    
                    _, jpeg = cv2.imencode('.jpg', frame_bgr, 
                        [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                    
                    with self.frame_lock:
                        self.current_frame = frame_bgr
                        self.current_jpeg = jpeg.tobytes()
                    
                    self._publish_ros_image()
                    
                    self.frame_count += 1
                    self.last_capture_time = time.time()
                
            except Exception as e:
                self.get_logger().error(f'Capture error: {e}')
            
            time.sleep(interval)
    
    def _publish_ros_image(self):
        """ROS2 CompressedImage 발행"""
        with self.frame_lock:
            if self.current_jpeg is None:
                return
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            msg.data = self.current_jpeg
            self.pub_image.publish(msg)
    
    def detection_callback(self, msg):
        """YOLO 서버에서 MQTT로 보낸 감지 결과 수신"""
        try:
            self.last_detection = json.loads(msg.data)
            self.get_logger().debug(f'Detection: {self.last_detection}')
            
            # QR 코드 감지 처리
            if 'qr_codes' in self.last_detection:
                self._process_qr_codes(self.last_detection['qr_codes'])
            
        except Exception as e:
            self.get_logger().warn(f'Detection parse error: {e}')
    
    def _process_qr_codes(self, qr_codes: list):
        """QR 코드 감지 처리"""
        for qr in qr_codes:
            qr_data = qr.get('data', '')
            
            # 포트 QR 코드 확인 (e.g., "PORT_A", "PORT_B")
            if qr_data.startswith('PORT_'):
                self.get_logger().info(f'🏷️ QR Port detected: {qr_data}')
                
                # 목표 포트와 일치하면 도착 알림
                if self.target_port and qr_data == self.target_port:
                    self._trigger_port_arrival(qr_data)
                elif not self.target_port:
                    # 목표 포트 없으면 일단 기록
                    if qr_data not in self.detected_qr_codes:
                        self.detected_qr_codes.append(qr_data)
    
    def _trigger_port_arrival(self, port_name: str):
        """포트 도착 알림"""
        if self.port_arrival_triggered:
            return
        
        self.port_arrival_triggered = True
        self.get_logger().info(f'🎯 PORT ARRIVAL: {port_name}')
        
        # 로봇 정지
        stop_msg = Bool()
        stop_msg.data = True
        self.pub_stop_robot.publish(stop_msg)
        
        # 포트 도착 메시지
        msg = String()
        msg.data = json.dumps({
            "event": "port_arrival",
            "port": port_name,
            "timestamp": datetime.now().isoformat(),
            "action": "dock_for_load_unload"
        })
        self.pub_port_arrival.publish(msg)
    
    def obstacle_callback(self, msg):
        """장애물 타입 수신 (wall/temporary/qr_port)"""
        obstacle_type = msg.data
        self.last_obstacle_type = obstacle_type
        
        self.get_logger().info(f'🚨 Obstacle type: {obstacle_type}')
        
        # 장애물 타입에 따른 처리
        if obstacle_type == self.OBSTACLE_WALL:
            self._handle_wall_obstacle()
        elif obstacle_type == self.OBSTACLE_TEMP:
            self._handle_temp_obstacle()
        elif obstacle_type == self.OBSTACLE_QR:
            self._handle_qr_obstacle()
    
    def _handle_wall_obstacle(self):
        """벽 장애물 처리 - Nav2가 알아서 회피"""
        msg = String()
        msg.data = json.dumps({
            "obstacle_type": "wall",
            "action": "navigate_around",
            "description": "Wall detected - Nav2 will handle navigation"
        })
        self.pub_obstacle_action.publish(msg)
    
    def _handle_temp_obstacle(self):
        """임시 장애물 처리 - 로그 + 재시도 대기"""
        self.temp_obstacle_count += 1
        
        self.get_logger().warn(
            f'⚠️ Temporary obstacle detected (count: {self.temp_obstacle_count})'
        )
        
        msg = String()
        msg.data = json.dumps({
            "obstacle_type": "temporary",
            "action": "wait_and_retry",
            "count": self.temp_obstacle_count,
            "description": "Temporary obstacle - waiting for clearance"
        })
        self.pub_obstacle_action.publish(msg)
    
    def _handle_qr_obstacle(self):
        """QR 장애물 (포트) 처리"""
        msg = String()
        msg.data = json.dumps({
            "obstacle_type": "qr_port",
            "action": "check_target_port",
            "detected_qr": self.detected_qr_codes,
            "target_port": self.target_port,
            "description": "QR port detected - checking if target"
        })
        self.pub_obstacle_action.publish(msg)
    
    def target_port_callback(self, msg):
        """목표 포트 설정"""
        self.target_port = msg.data
        self.port_arrival_triggered = False
        self.get_logger().info(f'🎯 Target port set: {self.target_port}')
    
    def _start_flask_server(self):
        """Flask HTTP 서버 시작"""
        app = Flask(__name__)
        node = self
        
        # Flask 로깅 최소화
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        @app.route('/image.jpg')
        def get_image():
            """현재 프레임 반환 (YOLO 서버가 이 URL로 접근)"""
            with node.frame_lock:
                if node.current_jpeg is None:
                    return Response(node._create_dummy_image(), mimetype='image/jpeg')
                return Response(node.current_jpeg, mimetype='image/jpeg')
        
        @app.route('/status')
        def get_status():
            """상태 정보 JSON"""
            return jsonify({
                'camera_ok': node.camera_ok,
                'frame_count': node.frame_count,
                'fps': node.fps,
                'resolution': f'{node.width}x{node.height}',
                'last_capture': node.last_capture_time,
                'last_obstacle_type': node.last_obstacle_type,
                'target_port': node.target_port,
                'detected_qr_codes': node.detected_qr_codes,
                'port_arrival_triggered': node.port_arrival_triggered,
                'temp_obstacle_count': node.temp_obstacle_count,
                'uptime': time.time() - node.start_time
            })
        
        @app.route('/')
        def index():
            """간단한 HTML 뷰어"""
            return '''
            <!DOCTYPE html>
            <html>
            <head>
                <title>Pinky Camera - Nav2 Mode</title>
                <meta http-equiv="refresh" content="1">
            </head>
            <body style="background:#222; text-align:center; padding:20px; font-family:Arial;">
                <h1 style="color:#0f0;">🤖 Pinky Camera (Nav2 Mode)</h1>
                <img src="/image.jpg" style="max-width:100%; border:3px solid #0f0; border-radius:10px;"/>
                <p style="color:#fff; margin-top:20px;">
                    1 FPS Streaming for YOLO Analysis | 
                    <a href="/status" style="color:#0ff;">Status JSON</a>
                </p>
            </body>
            </html>
            '''
        
        # Flask 서버를 별도 스레드에서 실행
        def run_server():
            try:
                app.run(host=self.host, port=self.port, threaded=True, use_reloader=False)
            except Exception as e:
                self.get_logger().error(f'Flask error: {e}')
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        self.get_logger().info(f'🌐 Flask server started on port {self.port}')
    
    def _create_dummy_image(self):
        """카메라 없을 때 더미 이미지"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.putText(img, 'NO CAMERA', (self.width//4, self.height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(img, datetime.now().strftime('%H:%M:%S'), 
                   (self.width//3, self.height//2 + 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        _, jpeg = cv2.imencode('.jpg', img)
        return jpeg.tobytes()
    
    def _log_status(self):
        """주기적 상태 로그"""
        if self.frame_count > 0:
            elapsed = time.time() - self.start_time
            actual_fps = self.frame_count / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f'📷 Frames: {self.frame_count} | '
                f'FPS: {actual_fps:.2f} | '
                f'Obstacle: {self.last_obstacle_type} | '
                f'Target: {self.target_port or "none"}'
            )
    
    def destroy_node(self):
        """노드 종료"""
        self.running = False
        
        if self.camera is not None:
            try:
                self.camera.close()
                self.get_logger().info('📷 Camera closed')
            except Exception:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    if not FLASK_AVAILABLE:
        print('❌ Flask required: pip install flask')
        return
    
    node = CameraStreamNode()
    
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
