#!/usr/bin/env python3
"""Nav2용 카메라 스트리밍 노드 (1fps) - Flask 웹서버로 YOLO 서버에 이미지 제공"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time
import threading

from slam_mqtt_project.topics import ROS, NET

try:
    from flask import Flask, Response, jsonify
    FLASK_OK = True
except ImportError:
    FLASK_OK = False

try:
    from picamera2 import Picamera2
    PICAM_OK = True
except ImportError:
    PICAM_OK = False


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        
        # 파라미터
        self.declare_parameter('fps', 1.0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('quality', 50)
        self.declare_parameter('port', NET.STREAMING_PORT)
        
        self.fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.quality = self.get_parameter('quality').value
        self.port = self.get_parameter('port').value
        
        # 상태
        self.jpeg = None
        self.lock = threading.Lock()
        self.count = 0
        self.running = True
        self.camera = None
        self.cam_ok = False
        
        # 카메라 초기화
        if PICAM_OK:
            try:
                self.camera = Picamera2()
                self.camera.configure(self.camera.create_preview_configuration(
                    main={"format": "RGB888", "size": (self.width, self.height)}
                ))
                self.camera.start()
                self.cam_ok = True
            except Exception as e:
                self.get_logger().error(f'Camera error: {e}')
        
        # ROS Publisher
        self.pub = self.create_publisher(CompressedImage, ROS.CAMERA_IMAGE + '/compressed', 1)
        
        # 캡처 스레드
        threading.Thread(target=self._capture_loop, daemon=True).start()
        
        # Flask 서버
        if FLASK_OK:
            self._start_flask()
        
        self.get_logger().info(f'📷 Camera Stream: http://0.0.0.0:{self.port}/image.jpg ({self.width}x{self.height} @ {self.fps}fps)')

    def _capture_loop(self):
        interval = 1.0 / self.fps
        while self.running:
            if self.cam_ok:
                try:
                    frame = cv2.rotate(self.camera.capture_array(), cv2.ROTATE_180)
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                    with self.lock:
                        self.jpeg = jpg.tobytes()
                    self._publish()
                    self.count += 1
                except Exception as e:
                    self.get_logger().error(f'Capture: {e}', throttle_duration_sec=5.0)
            time.sleep(interval)

    def _publish(self):
        with self.lock:
            if not self.jpeg:
                return
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = self.jpeg
            self.pub.publish(msg)

    def _start_flask(self):
        app = Flask(__name__)
        import logging
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        
        @app.route('/image.jpg')
        def image():
            with self.lock:
                return Response(self.jpeg or self._dummy(), mimetype='image/jpeg')
        
        @app.route('/status')
        def status():
            return jsonify({'ok': self.cam_ok, 'frames': self.count, 'fps': self.fps})
        
        @app.route('/')
        def index():
            return f'''<!DOCTYPE html><html><head><title>Pinky Cam</title>
<meta http-equiv="refresh" content="1"></head>
<body style="background:#222;text-align:center;color:#fff;font-family:Arial">
<h2>🤖 Pinky Camera ({self.fps} fps)</h2>
<img src="/image.jpg" style="max-width:100%;border:2px solid #0f0;border-radius:8px"/>
<p><a href="/status" style="color:#0ff">Status</a></p></body></html>'''
        
        threading.Thread(target=lambda: app.run(host='0.0.0.0', port=self.port, threaded=True, use_reloader=False), daemon=True).start()

    def _dummy(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.putText(img, 'NO CAM', (self.width//4, self.height//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        _, jpg = cv2.imencode('.jpg', img)
        return jpg.tobytes()

    def destroy_node(self):
        self.running = False
        if self.camera:
            try:
                self.camera.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    if not FLASK_OK:
        print('❌ pip install flask')
        return
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == '__main__':
    main()
