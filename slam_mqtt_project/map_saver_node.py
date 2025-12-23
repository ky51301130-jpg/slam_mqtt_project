#!/usr/bin/env python3
"""맵 자동 저장 + SLAM 리셋 노드"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32, String, Bool
from slam_toolbox.srv import Reset
import tf2_ros
import numpy as np
import cv2
import os, json, math, threading, requests

from slam_mqtt_project.topics import ROS, NET, ARUCO

try:
    from pinkylib import Buzzer
    BUZZER_OK = True
except ImportError:
    BUZZER_OK = False

SAVE_DIR = '/home/pinky/saved_maps'
MAX_FILES = 8
STABLE_INTERVAL = 10.0
STABLE_THRESHOLD = 3
MIN_EXPLORE = 30.0


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        
        self.latest_map = None
        self.file_idx = 0
        self.init_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.init_saved = False
        self.port_odom = {}
        self.waiting = self.returning = False
        self.prev_free = 0
        self.stable_cnt = 0
        self.start_time = self.get_clock().now()
        
        self.tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buf, self)
        
        # Subs/Pubs
        self.create_subscription(OccupancyGrid, ROS.MAP, self.map_cb, 1)
        self.create_subscription(Bool, ROS.ROBOT_AT_HOME, self.home_cb, 1)
        self.create_subscription(String, ARUCO.PORT_ODOM, self.port_cb, 10)
        
        self.save_pub = self.create_publisher(Int32, ROS.MAP_SAVER_SAVED, 1)
        self.cycle_pub = self.create_publisher(String, ROS.MAP_SAVER_CYCLE, 1)
        self.return_pub = self.create_publisher(Bool, ROS.ROBOT_RETURN_HOME, 1)
        
        self.reset_cli = self.create_client(Reset, '/slam_toolbox/reset')
        
        self.create_timer(STABLE_INTERVAL, self.check_stable)
        self.pose_timer = self.create_timer(5.0, self.capture_pose)
        
        os.makedirs(SAVE_DIR, exist_ok=True)
        self.get_logger().info("MapSaver started")

    def beep(self, n=2):
        if not BUZZER_OK: return
        def _b():
            try:
                b = Buzzer()
                b.buzzer_start()
                b.buzzer(n)
                b.buzzer_stop()
                b.clean()
            except: pass
        threading.Thread(target=_b, daemon=True).start()

    @staticmethod
    def q2yaw(r):
        return math.atan2(2*(r.w*r.z+r.x*r.y), 1-2*(r.y*r.y+r.z*r.z))

    def capture_pose(self):
        if self.init_saved:
            self.pose_timer.cancel()
            return
        try:
            tf = self.tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.init_pose = {'x': tf.transform.translation.x, 'y': tf.transform.translation.y,
                              'yaw': self.q2yaw(tf.transform.rotation)}
            self.init_saved = True
            self.pose_timer.cancel()
        except: pass

    def map_cb(self, m): self.latest_map = m

    def port_cb(self, m):
        try:
            d = json.loads(m.data)
            if d.get('port_name'):
                self.port_odom[d['port_name']] = {'x': d['x'], 'y': d['y'], 'yaw': d['yaw']}
        except: pass

    def home_cb(self, m):
        if m.data and self.waiting:
            self.beep(2)
            self.waiting = self.returning = False
            import time; time.sleep(0.3)
            self.save_and_reset()

    def check_stable(self):
        if self.returning or not self.latest_map: return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < MIN_EXPLORE: return
        
        data = np.array(self.latest_map.data, dtype=np.int8)
        total = len(data)
        free = np.sum(data == 0)         # 자유 공간
        occupied = np.sum(data == 100)   # 장애물
        unknown = np.sum(data == -1)     # 미탐색
        explored = free + occupied       # 탐색된 영역
        
        # 맵 완성도 계산 (탐색된 영역 / 전체)
        completion = (explored / total * 100) if total > 0 else 0
        
        if free < 100: return
        
        # 10초마다 완성도 로그
        if int(elapsed) % 10 == 0 and int(elapsed) != getattr(self, '_last_log_time', -1):
            self._last_log_time = int(elapsed)
            self.get_logger().info(f"🗺️ 맵 상태: 완성도 {completion:.1f}% | 자유:{free} 장애물:{occupied} 미탐색:{unknown}")
        
        if self.prev_free > 0:
            rate = abs(free - self.prev_free) / self.prev_free * 100
            if rate < 0.5:
                self.stable_cnt += 1
                if self.stable_cnt >= STABLE_THRESHOLD:
                    self.get_logger().info(f"✅ 맵 안정화! 완성도 {completion:.1f}% → 귀환 요청")
                    self.stable_cnt = 0
                    self.request_return()
            else:
                self.stable_cnt = 0
        self.prev_free = free

    def request_return(self):
        if self.returning: return
        self.returning = self.waiting = True
        self.return_pub.publish(Bool(data=True))
        self.beep(3)

    def save_and_reset(self):
        if self.save_map():
            self.reset_slam()
        self._reset_state()

    def _reset_state(self):
        self.prev_free = self.stable_cnt = 0
        self.start_time = self.get_clock().now()

    def reset_slam(self):
        if not self.reset_cli.wait_for_service(timeout_sec=2.0): return
        try:
            fut = self.reset_cli.call_async(Reset.Request())
            fut.add_done_callback(lambda f: self._on_reset())
        except: pass

    def _on_reset(self):
        self.latest_map = None
        self.init_saved = False
        self.waiting = self.returning = False
        self.pose_timer = self.create_timer(5.0, self.capture_pose)

    def save_map(self):
        if not self.latest_map: return False
        try:
            info = self.latest_map.info
            data = np.array(self.latest_map.data, dtype=np.int8).reshape((info.height, info.width))
            
            # 맵 완성도 계산
            total = data.size
            free = np.sum(data == 0)
            occupied = np.sum(data == 100)
            explored = free + occupied
            completion = (explored / total * 100) if total > 0 else 0
            
            # OccupancyGrid → 그레이스케일 변환
            # -1: 미탐색(205), 0: 자유공간(254), 100: 장애물(0)
            img = np.full_like(data, 205, dtype=np.uint8)
            img[data == 0] = 254      # 자유 공간 = 흰색
            img[data == 100] = 0       # 장애물 = 검정
            
            # 중간값 (확률) 처리 - 회색 그라데이션
            mask = (data > 0) & (data < 100)
            if np.any(mask):
                img[mask] = (254 - (data[mask].astype(np.float32) * 254 / 100)).astype(np.uint8)
            
            img = np.flipud(img)  # 좌표계 뒤집기
            
            base = f"map_{self.file_idx:02d}"
            pgm, png, yam = [os.path.join(SAVE_DIR, f"{base}.{e}") for e in ['pgm','png','yaml']]
            
            cv2.imwrite(pgm, img)
            cv2.imwrite(png, img)
            
            r = info.origin.orientation
            yaml_str = f"""image: {base}.pgm
mode: trinary
resolution: {info.resolution}
origin: [{info.origin.position.x}, {info.origin.position.y}, {self.q2yaw(r)}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
initial_pose:
  x: {self.init_pose['x']}
  y: {self.init_pose['y']}
  yaw: {self.init_pose['yaw']}
map_info:
  completion: {completion:.1f}
  free_cells: {free}
  occupied_cells: {occupied}
  total_cells: {total}
"""
            if self.port_odom:
                yaml_str += "ports:\n"
                for pn, ps in self.port_odom.items():
                    yaml_str += f"  {pn}:\n    x: {ps['x']}\n    y: {ps['y']}\n    yaw: {ps['yaw']}\n"
            
            with open(yam, 'w') as f: f.write(yaml_str)
            
            self.get_logger().info(f"💾 맵 저장: {base} (완성도 {completion:.1f}%, PORT: {list(self.port_odom.keys())})")
            
            self.beep(1)
            self.save_pub.publish(Int32(data=self.file_idx + 1))
            
            # Upload
            url = NET.map_upload_url()
            for fp, fn in [(pgm, f"{base}.pgm"), (png, f"{base}.png"), (yam, f"{base}.yaml")]:
                try:
                    with open(fp, 'rb') as f: requests.post(url, files={'file': (fn, f)}, timeout=5)
                except: pass
            
            # Publish events
            self.cycle_pub.publish(String(data=json.dumps({
                "event": "map_saved", "cycle_number": self.file_idx, "total_cycles": MAX_FILES
            })))
            
            self.file_idx = (self.file_idx + 1) % MAX_FILES
            if self.file_idx == 0:
                self.cycle_pub.publish(String(data=json.dumps({"event": "cycle_complete", "maps_count": MAX_FILES})))
                self.beep(3)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Save error: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
