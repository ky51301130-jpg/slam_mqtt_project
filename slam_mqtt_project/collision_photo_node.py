#!/usr/bin/env python3
"""충돌 사진 + ArUco 감지 + Flask 웹서버 노드"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Odometry
import os, time, threading, json, math
import numpy as np
from datetime import datetime

cv2 = None

try:
    from flask import Flask, jsonify, send_file
    FLASK_OK = True
except ImportError:
    FLASK_OK = False

try:
    from picamera2 import Picamera2
    PICAM_OK = True
except ImportError:
    PICAM_OK = False

from slam_mqtt_project.topics import ROS, NET, ARUCO

PHOTO_DIR = "/home/pinky/collision_photos"
MAP_DIR = "/home/pinky/saved_maps"
MAX_PHOTOS = 30
COOLDOWN = 3.0


class CollisionPhotoNode(Node):
    def __init__(self):
        super().__init__('collision_photo_node')
        
        self.last_photo = 0.0
        self.camera = None
        self.cam_ok = False
        self.aruco_dict = self.aruco_params = None
        self.x = self.y = self.yaw = 0.0
        self.port_save = False
        self.saved_ports = set()
        self.port_goals = self._load_ports()
        
        os.makedirs(PHOTO_DIR, exist_ok=True)
        
        if PICAM_OK:
            self._init_cam()
        
        # Subscribers
        self.create_subscription(Float32, ROS.ULTRASONIC, self.us_cb, 10)
        self.create_subscription(Bool, ROS.COLLISION_TRIGGER, self.trig_cb, 10)
        self.create_subscription(Odometry, ROS.ODOM, self.odom_cb, 10)
        self.create_subscription(String, ROS.ROBOT_MODE, self.mode_cb, 10)
        
        # Publishers
        self.photo_pub = self.create_publisher(String, ROS.COLLISION_PHOTO_READY, 10)
        self.aruco_pub = self.create_publisher(String, ROS.ARUCO_HOME_DETECTED, 10)
        
        if FLASK_OK:
            self._start_flask()
        
        self.create_timer(0.1, self.detect_aruco)
        self.get_logger().info(f"📷 Collision Photo Node (port {NET.COLLISION_PORT})")

    def _init_cam(self):
        global cv2
        if cv2 is None:
            import cv2 as _cv2
            cv2 = _cv2
        try:
            self.camera = Picamera2()
            self.camera.configure(self.camera.create_still_configuration(
                main={"format": "RGB888", "size": (640, 480)}))
            self.camera.start()
            self.cam_ok = True
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def _load_ports(self):
        try:
            with open("/home/pinky/saved_maps/port_goals.json") as f:
                return json.load(f)
        except:
            return {}

    def _save_ports(self):
        try:
            with open("/home/pinky/saved_maps/port_goals.json", 'w') as f:
                json.dump(self.port_goals, f, indent=2)
        except: pass

    def _start_flask(self):
        app = Flask(__name__)
        import logging
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        
        @app.route('/')
        def index():
            maps = sorted([f for f in os.listdir(MAP_DIR) if f.endswith('.png')], reverse=True)
            photos = sorted([f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')], reverse=True)[:5]
            html = '''<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pinky Robot</title>
<style>body{font-family:Arial;margin:20px;background:#1a1a2e;color:#eee}
h1,h2{color:#4fc3f7}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:15px}
.card{background:#16213e;border-radius:10px;padding:10px;text-align:center}
.card img{max-width:100%;border-radius:5px}.card a{color:#4fc3f7;text-decoration:none}</style></head>
<body><h1>🤖 Pinky Robot</h1><h2>🗺️ Maps</h2><div class="grid">'''
            for m in maps:
                html += f'<div class="card"><a href="/maps/{m}"><img src="/maps/{m}"><br>{m}</a></div>'
            html += '</div><h2>📸 Photos</h2><div class="grid">'
            for p in photos:
                html += f'<div class="card"><a href="/photos/{p}"><img src="/photos/{p}"><br>{p}</a></div>'
            return html + '</div></body></html>'
        
        @app.route('/maps')
        def list_maps():
            return jsonify(sorted([f for f in os.listdir(MAP_DIR) if f.endswith(('.png','.pgm','.yaml'))]))
        
        @app.route('/maps/<fn>')
        def get_map(fn):
            p = os.path.join(MAP_DIR, fn)
            return send_file(p, mimetype='image/png') if os.path.exists(p) else ("Not found", 404)
        
        @app.route('/photos')
        def list_photos():
            return jsonify(sorted([f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')]))
        
        @app.route('/photos/<fn>')
        def get_photo(fn):
            p = os.path.join(PHOTO_DIR, fn)
            return send_file(p, mimetype='image/jpeg') if os.path.exists(p) else ("Not found", 404)
        
        @app.route('/latest')
        def latest():
            files = sorted([f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')])
            return send_file(os.path.join(PHOTO_DIR, files[-1]), mimetype='image/jpeg') if files else ("No photos", 404)
        
        threading.Thread(target=lambda: app.run(host='0.0.0.0', port=NET.COLLISION_PORT, threaded=True), daemon=True).start()

    def odom_cb(self, m):
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def mode_cb(self, m):
        self.port_save = m.data.upper() == "SLAM"
        if self.port_save:
            self.saved_ports.clear()

    def us_cb(self, m):
        if m.data < 0.25:
            self.capture(m.data)

    def trig_cb(self, m):
        if m.data:
            self.capture(0.0)

    def capture(self, dist):
        now = time.time()
        if now - self.last_photo < COOLDOWN or not self.cam_ok:
            return
        try:
            frame = cv2.rotate(self.camera.capture_array(), cv2.ROTATE_180)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fn = f"collision_{ts}_{dist:.2f}m.jpg"
            cv2.imwrite(os.path.join(PHOTO_DIR, fn), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.last_photo = now
            self._cleanup()
            self.photo_pub.publish(String(data=json.dumps({'filename': fn, 'distance': dist})))
        except Exception as e:
            self.get_logger().error(f"Capture error: {e}")

    def _cleanup(self):
        files = sorted([f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')])
        while len(files) > MAX_PHOTOS:
            try:
                os.remove(os.path.join(PHOTO_DIR, files.pop(0)))
            except: pass

    def detect_aruco(self):
        if not self.cam_ok or not self.aruco_dict:
            return
        try:
            frame = cv2.rotate(self.camera.capture_array(), cv2.ROTATE_180)
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners, ids, _ = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params).detectMarkers(gray)
            
            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    mid = int(mid)
                    if mid not in ARUCO.DOCK_MARKER_IDS:
                        continue
                    c = corners[i][0]
                    cx = (np.mean(c[:, 0]) - 320) / 320
                    sz = (np.linalg.norm(c[0]-c[1]) + np.linalg.norm(c[1]-c[2])) / 2
                    
                    self.aruco_pub.publish(String(data=json.dumps({
                        "detected": True, "marker_id": mid,
                        "center_x": round(float(cx), 3), "size": round(float(sz), 1),
                        "port_name": ARUCO.PORT_MAP.get(mid, f"ID:{mid}")
                    })))
                    
                    # PORT 저장
                    if self.port_save and mid in ARUCO.DOCK_MARKER_IDS:
                        pn = ARUCO.PORT_MAP.get(mid)
                        if pn and pn not in self.saved_ports and abs(cx) < ARUCO.DOCK_CENTER_TOLERANCE:
                            if abs(sz - ARUCO.DOCK_SIZE_TARGET) < 30:
                                self.port_goals[pn] = {"x": round(self.x,4), "y": round(self.y,4), "yaw": round(self.yaw,4)}
                                self.saved_ports.add(pn)
                                self._save_ports()
                                self.get_logger().info(f"✅ PORT SAVED: {pn}")
            else:
                self.aruco_pub.publish(String(data=json.dumps({"detected": False})))
        except: pass


def main(args=None):
    rclpy.init(args=args)
    node = CollisionPhotoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == '__main__':
    main()
