#!/usr/bin/env python3
"""맵 자동 저장 + SLAM 리셋 노드 (경량화 버전)"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32, String, Bool
from slam_toolbox.srv import Reset
import tf2_ros
import numpy as np
import cv2
import os
import requests
from datetime import datetime
import threading
import math
import yaml

# 중앙 집중식 토픽 관리
from slam_mqtt_project.topics import ROS, NET, ARUCO

try:
    from pinkylib import Buzzer
    BUZZER_AVAILABLE = True
except ImportError:
    BUZZER_AVAILABLE = False

# 상수
SAVE_DIR = '/home/pinky/saved_maps'
MAX_FILES = 8
STABLE_CHECK_INTERVAL = 10.0  # 변화율 체크 간격 (초)
STABLE_COUNT_THRESHOLD = 3    # 연속 3회 (약 30초) 변화 없으면 복귀
MIN_EXPLORE_TIME = 30.0       # 최소 탐색 시간 (초) - 이전에는 stable 체크 안 함
MIN_FREE_SPACE = 100          # 최소 free space 픽셀 수 - 맵이 조금이라도 있으면 체크
UPLOAD_URL = NET.map_upload_url()


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        
        # 상태
        self.latest_map = None
        self.file_index = 0
        self.initial_pose_saved = False
        self.initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.port_locations = {}
        self.port_odom = {}  # PORT 좌표 (odom 기준) - YAML에 저장용
        self.waiting_for_home = False
        self.returning_home = False
        
        # 흰색(free space) 변화율 추적
        self.prev_free_count = 0
        self.stable_count = 0
        self.start_time = self.get_clock().now()  # 시작 시간 기록
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 구독/발행
        self.create_subscription(OccupancyGrid, ROS.MAP, self.map_cb, 1)
        self.create_subscription(Bool, ROS.ROBOT_AT_HOME, self.at_home_cb, 1)
        self.create_subscription(String, ROS.MQTT_PLC_LOCATION, self.plc_location_cb, 1)
        self.create_subscription(String, ARUCO.PORT_ODOM, self.port_odom_cb, 10)  # PORT 좌표 구독
        
        self.save_pub = self.create_publisher(Int32, ROS.MAP_SAVER_SAVED, 1)
        self.cycle_pub = self.create_publisher(String, ROS.MAP_SAVER_CYCLE, 1)
        self.return_home_pub = self.create_publisher(Bool, ROS.ROBOT_RETURN_HOME, 1)
        self.map_complete_pub = self.create_publisher(Bool, ROS.MAP_SAVER_COMPLETE, 1)
        # odom_reset_pub 제거 - SLAM 리셋만으로 충분, 정밀 도킹이 HOME 위치 보장
        
        self.reset_client = self.create_client(Reset, '/slam_toolbox/reset')
        
        # 타이머
        self.create_timer(STABLE_CHECK_INTERVAL, self.check_free_space_change)  # 변화율 체크
        self.initial_pose_timer = self.create_timer(5.0, self.capture_initial_pose)
        
        os.makedirs(SAVE_DIR, exist_ok=True)
        self.get_logger().info(f"MapSaver started (stable threshold: {STABLE_COUNT_THRESHOLD} checks)")

    @staticmethod
    def quat_to_yaw(qx, qy, qz, qw):
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def capture_initial_pose(self):
        """초기 pose 캡처"""
        if self.initial_pose_saved:
            self.initial_pose_timer.cancel()
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.initial_pose['x'] = tf.transform.translation.x
            self.initial_pose['y'] = tf.transform.translation.y
            r = tf.transform.rotation
            self.initial_pose['yaw'] = self.quat_to_yaw(r.x, r.y, r.z, r.w)
            self.initial_pose_saved = True
            self.initial_pose_timer.cancel()
        except:
            pass

    def beep_success(self, count=2):
        if not BUZZER_AVAILABLE:
            return
        def _beep():
            try:
                b = Buzzer()
                b.buzzer_start()
                b.buzzer(count)
                b.buzzer_stop()
                b.clean()
            except:
                pass
        threading.Thread(target=_beep, daemon=True).start()

    def map_cb(self, msg):
        self.latest_map = msg

    def port_odom_cb(self, msg):
        """PORT 좌표(odom 기준) 수신 → YAML 저장용으로 기록"""
        try:
            import json
            data = json.loads(msg.data)
            port_name = data.get('port_name', '')
            if port_name:
                self.port_odom[port_name] = {
                    'x': data.get('x', 0.0),
                    'y': data.get('y', 0.0),
                    'yaw': data.get('yaw', 0.0)
                }
                self.get_logger().info(f"📍 PORT_ODOM 수신: {port_name} ({data.get('x'):.2f}, {data.get('y'):.2f})")
        except Exception as e:
            self.get_logger().warn(f"PORT_ODOM 파싱 실패: {e}")

    def plc_location_cb(self, msg):
        try:
            if '=' not in msg.data:
                return
            key, value = msg.data.strip().split('=', 1)
            if value.strip() != '1':
                return
            key = key.strip().lower()
            if key not in ['port_a', 'port_b', 'port_c', 'port_d', 'home', 'charge']:
                return
            pose = self.get_current_pose()
            if pose:
                self.port_locations[key] = pose
                self.save_port_locations()
                self.beep_success(1)
        except:
            pass

    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), 
                                                  timeout=rclpy.duration.Duration(seconds=0.5))
            r = tf.transform.rotation
            return {'x': tf.transform.translation.x, 'y': tf.transform.translation.y, 
                    'yaw': self.quat_to_yaw(r.x, r.y, r.z, r.w)}
        except:
            return None

    def save_port_locations(self):
        try:
            path = os.path.join(SAVE_DIR, 'port_locations.yaml')
            data = {'port_locations': {p: {'x': v['x'], 'y': v['y'], 'yaw': v['yaw']} 
                                       for p, v in self.port_locations.items()}}
            with open(path, 'w') as f:
                yaml.dump(data, f)
            self.upload_file(path, 'port_locations.yaml')
        except:
            pass

    def upload_file(self, filepath, filename):
        try:
            with open(filepath, 'rb') as f:
                requests.post(UPLOAD_URL, files={'file': (filename, f)}, timeout=5)
        except:
            pass

    def check_free_space_change(self):
        """흰색(free space) 변화율이 0이면 복귀 요청"""
        if self.returning_home or self.waiting_for_home:
            self.get_logger().info(f"🏠 Returning home... (skipping map check)", throttle_duration_sec=5.0)
            return
        
        if not self.latest_map:
            return
        
        # 최소 탐색 시간 체크 - 시작 후 일정 시간은 stable 체크 안 함
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < MIN_EXPLORE_TIME:
            self.get_logger().info(f"⏳ Exploring: {elapsed:.0f}/{MIN_EXPLORE_TIME:.0f}s")
            return
        
        # 흰색(free=0) 픽셀 개수 계산
        map_data = np.array(self.latest_map.data, dtype=np.int8)
        current_free_count = np.sum(map_data == 0)
        
        # 최소 free space 체크 - 맵이 충분히 탐색되어야 함
        if current_free_count < MIN_FREE_SPACE:
            self.get_logger().info(f"📊 Map too small: {current_free_count}/{MIN_FREE_SPACE} free pixels")
            return
        
        # 변화율 계산
        if self.prev_free_count > 0:
            change = abs(current_free_count - self.prev_free_count)
            change_rate = change / self.prev_free_count * 100
            
            if change_rate < 0.5:  # 0.5% 미만 변화 = 거의 변화 없음
                self.stable_count += 1
                self.get_logger().info(f"📊 Free space stable: {self.stable_count}/{STABLE_COUNT_THRESHOLD} (change: {change_rate:.2f}%, free: {current_free_count})")
            else:
                self.stable_count = 0
                self.get_logger().info(f"📊 Free space changing: {change_rate:.2f}% (free: {current_free_count})")
            
            # 연속으로 변화 없으면 복귀
            if self.stable_count >= STABLE_COUNT_THRESHOLD:
                self.stable_count = 0
                self.get_logger().info(f"🏠 Map complete! Free space stable for {STABLE_COUNT_THRESHOLD * STABLE_CHECK_INTERVAL}s → Return home")
                self.request_return_home()
        
        self.prev_free_count = current_free_count

    def at_home_cb(self, msg):
        """복귀 완료 시 저장 (PORT_ODOM 수신 대기 후)"""
        if msg.data and self.waiting_for_home:
            self.get_logger().info("🏠 Arrived home → Saving map")
            self.waiting_for_home = False
            self.returning_home = False
            # ★ PORT_ODOM 수신 대기 (0.3초) - 동시 발행된 메시지 처리 보장 ★
            import time
            time.sleep(0.3)
            self.do_save_and_reset()

    def request_return_home(self):
        """복귀 요청"""
        if self.returning_home:
            return
        self.returning_home = True
        self.waiting_for_home = True
        msg = Bool()
        msg.data = True
        self.return_home_pub.publish(msg)
        self.beep_success(3)
        self.get_logger().info("🏠 Return home requested!")
        # 타임아웃 자동저장 제거 - 마커 도킹 성공할 때만 저장

    def do_save_and_reset(self):
        """저장 후 리셋 (SLAM만 리셋 - odom은 리셋하지 않음)
        
        정밀 도킹으로 HOME 위치가 보장되므로, SLAM 리셋 시
        map→odom TF가 초기화되면 자연스럽게 원점 정합됨.
        odom 리셋 시 점프(teleport) 문제 발생하므로 제거.
        """
        if self.save_map():
            # SLAM 리셋 (map→odom TF 초기화)
            self.reset_slam()
            
            # 다음 사이클을 위해 stable 상태 초기화
            self.prev_free_count = 0
            self.stable_count = 0
            self.start_time = self.get_clock().now()
            
            self.get_logger().info("🔄 SLAM reset complete → Ready for next cycle")
        else:
            # 저장 실패 시에도 stable 초기화
            self.prev_free_count = 0
            self.stable_count = 0
            self.start_time = self.get_clock().now()
    
    # _delayed_odom_reset 제거 - odom 리셋 사용 안함

    def _delayed_slam_reset(self):
        """지연된 SLAM 리셋 (odom 리셋 후 호출)"""
        # 타이머 취소 (one-shot)
        if hasattr(self, '_slam_reset_timer'):
            self._slam_reset_timer.cancel()
        
        self.reset_slam()
        # 다음 사이클을 위해 stable 상태 초기화
        self.prev_free_count = 0
        self.stable_count = 0
        self.start_time = self.get_clock().now()

    def reset_slam(self):
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            return
        try:
            self.reset_client.call_async(Reset.Request()).add_done_callback(self._reset_cb)
        except:
            pass
    
    def _reset_cb(self, future):
        try:
            future.result()
            self.latest_map = None
            self.initial_pose_saved = False
            self.initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            # 다음 사이클을 위해 stable 상태 완전 초기화
            self.prev_free_count = 0
            self.stable_count = 0
            self.start_time = self.get_clock().now()
            self.waiting_for_home = False
            self.returning_home = False
            self.initial_pose_timer = self.create_timer(5.0, self.capture_initial_pose)
            
            self.get_logger().info("🔄 SLAM reset complete")
        except:
            pass

    def save_map(self):
        if not self.latest_map:
            return False
        
        try:
            info = self.latest_map.info
            map_data = np.array(self.latest_map.data, dtype=np.int8).reshape((info.height, info.width))
            
            # 값 변환
            img = np.full_like(map_data, 205, dtype=np.uint8)
            img[map_data == 0] = 254
            img[map_data == 100] = 0
            mask = (map_data > 0) & (map_data < 100)
            img[mask] = (254 - (map_data[mask] * 254 / 100)).astype(np.uint8)
            img = np.flipud(img)
            
            base = f"map_{self.file_index:02d}"
            pgm = os.path.join(SAVE_DIR, f"{base}.pgm")
            png = os.path.join(SAVE_DIR, f"{base}.png")
            yam = os.path.join(SAVE_DIR, f"{base}.yaml")
            
            cv2.imwrite(pgm, img)
            cv2.imwrite(png, img)
            
            r = info.origin.orientation
            
            # YAML 내용 생성
            yaml_content = f"""image: {base}.pgm
mode: trinary
resolution: {info.resolution}
origin: [{info.origin.position.x}, {info.origin.position.y}, {self.quat_to_yaw(r.x, r.y, r.z, r.w)}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
initial_pose:
  x: {self.initial_pose['x']}
  y: {self.initial_pose['y']}
  yaw: {self.initial_pose['yaw']}
"""
            # PORT 좌표 추가 (odom 기준 - Nav2 goal로 사용)
            if self.port_odom:
                yaml_content += "ports:\n"
                for port_name, pose in self.port_odom.items():
                    yaml_content += f"  {port_name}:\n"
                    yaml_content += f"    x: {pose['x']}\n"
                    yaml_content += f"    y: {pose['y']}\n"
                    yaml_content += f"    yaw: {pose['yaw']}\n"
                self.get_logger().info(f"📍 YAML에 PORT 좌표 저장: {list(self.port_odom.keys())}")
            
            with open(yam, 'w') as f:
                f.write(yaml_content)
            
            self.beep_success(1)  # 맵 저장 알림 (1번)
            self.get_logger().info(f"💾 Map saved: {base} ({self.file_index + 1}/{MAX_FILES})")
            msg = Int32()
            msg.data = self.file_index + 1  # 1부터 시작 (LED 진행률용)
            self.save_pub.publish(msg)
            
            self.upload_file(pgm, f"{base}.pgm")
            self.upload_file(png, f"{base}.png")
            self.upload_file(yam, f"{base}.yaml")
            
            # 각 맵 저장 시 서버에 알림 (개별 cycle)
            self.publish_map_saved(self.file_index, pgm, png, yam)
            
            prev_index = self.file_index
            self.file_index = (self.file_index + 1) % MAX_FILES
            
            # 8개 완료 시 (0으로 돌아갈 때)
            if self.file_index == 0:
                self.publish_cycle_complete()
            
            return True
        except Exception as e:
            self.get_logger().error(f"Save: {e}")
            return False

    def publish_map_saved(self, cycle_num, pgm_path, png_path, yaml_path):
        """개별 맵 저장 시 서버에 알림 (서버가 개별 맵 수집용)"""
        import json
        msg = String()
        msg.data = json.dumps({
            "event": "map_saved",
            "cycle_number": cycle_num,
            "pgm_path": pgm_path,
            "png_path": png_path,
            "yaml_path": yaml_path,
            "total_cycles": MAX_FILES
        })
        self.cycle_pub.publish(msg)

    def publish_cycle_complete(self):
        """8개 맵 완료 시 서버에 알림 (서버가 통합 맵 생성 시작)"""
        import json
        msg = String()
        msg.data = json.dumps({
            "event": "cycle_complete", 
            "maps_count": MAX_FILES,
            "save_dir": SAVE_DIR
        })
        self.cycle_pub.publish(msg)
        self.beep_success(3)  # 사이클 완료 알림 (3번)
        self.get_logger().info(f"🎉 8-Cycle complete! Server will build Nav2 map.")


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
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
