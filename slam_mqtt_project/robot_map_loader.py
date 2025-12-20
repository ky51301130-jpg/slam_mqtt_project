#!/usr/bin/env python3
"""
=============================================================================
                        Robot Map Loader Node
=============================================================================
PC 서버에서 병합된 맵을 다운로드하여 Nav2에 로드하는 노드

흐름:
1. MQTT 'robot/map_ready' 구독 → 맵 병합 완료 신호 대기
2. HTTP GET으로 PC 서버에서 맵 파일 다운로드
3. /home/pinky/nav2_maps/ 에 저장
4. Nav2 map_server에 새 맵 로드 요청

MQTT 메시지 형식:
{
    "pgm": "merged_map.pgm",
    "yaml": "merged_map.yaml",
    "timestamp": "20251217_193000"
}
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import paho.mqtt.client as mqtt
import requests
import json
import os
from datetime import datetime

from slam_mqtt_project.topics import ROS, MQTT as MQTT_TOPICS, NET


class RobotMapLoader(Node):
    """PC 서버에서 병합된 맵을 다운로드하여 Nav2에 로드"""
    
    def __init__(self):
        super().__init__('robot_map_loader')
        
        # ======================== 설정 ========================
        self.map_save_dir = "/home/pinky/saved_maps/renew"
        self.download_base_url = f"http://{NET.SERVER_IP}:{NET.MAP_UPLOAD_PORT}/download"
        
        # 디렉토리 생성
        os.makedirs(self.map_save_dir, exist_ok=True)
        
        # ======================== ROS2 Publisher ========================
        self.pub_map_ready = self.create_publisher(Bool, ROS.NAV2_MAP_READY, 10)
        self.pub_status = self.create_publisher(String, ROS.ROBOT_STATUS, 10)
        
        # ======================== MQTT 설정 ========================
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(MQTT_TOPICS.HOST, MQTT_TOPICS.PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"MQTT 연결: {MQTT_TOPICS.HOST}:{MQTT_TOPICS.PORT}")
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
        
        # ======================== 상태 ========================
        self.last_download_time = None
        self.current_map_file = None
        self.port_goals = {}  # YAML에서 읽은 PORT 좌표
        
        self.get_logger().info("="*50)
        self.get_logger().info("Robot Map Loader 시작")
        self.get_logger().info(f"  다운로드 URL: {self.download_base_url}")
        self.get_logger().info(f"  저장 경로: {self.map_save_dir}")
        self.get_logger().info("  MQTT 'robot/map_ready' 대기 중...")
        self.get_logger().info("="*50)
    
    # ======================== MQTT 콜백 ========================
    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT 연결 시 토픽 구독"""
        if rc == 0:
            # 맵 준비 신호 구독
            client.subscribe("robot/map_ready")
            self.get_logger().info("MQTT 구독: robot/map_ready")
        else:
            self.get_logger().error(f"MQTT 연결 실패: rc={rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT 메시지 수신"""
        topic = msg.topic
        try:
            payload = msg.payload.decode('utf-8')
            self.get_logger().info(f"MQTT 수신 [{topic}]: {payload}")
            
            if topic == "robot/map_ready":
                self.handle_map_ready(payload)
                
        except Exception as e:
            self.get_logger().error(f"MQTT 메시지 처리 오류: {e}")
    
    # ======================== 맵 다운로드 처리 ========================
    def handle_map_ready(self, payload):
        """맵 준비 신호 처리 → 다운로드 시작
        
        서버에서 보내는 형식:
        {
            "event": "map_ready",
            "pgm": "nav2_final_map_20251217_200630.pgm",
            "yaml": "nav2_final_map_20251217_200630.yaml",
            "download_url": {
                "base": "http://192.168.0.3:5100",
                "yaml": "http://192.168.0.3:5100/download/nav2_final_map_20251217_200630.yaml",
                "pgm": "http://192.168.0.3:5100/download/nav2_final_map_20251217_200630.pgm",
                "qr_positions": "http://192.168.0.3:5100/download/qr_positions.yaml"
            },
            "timestamp": 1234567890.123
        }
        """
        try:
            data = json.loads(payload)
            pgm_file = data.get("pgm", "merged_map.pgm")
            yaml_file = data.get("yaml", "merged_map.yaml")
            download_urls = data.get("download_url", {})
            timestamp = data.get("timestamp", datetime.now().strftime("%Y%m%d_%H%M%S"))
            
            # URL 추출
            pgm_url = download_urls.get("pgm", f"{self.download_base_url}/{pgm_file}")
            yaml_url = download_urls.get("yaml", f"{self.download_base_url}/{yaml_file}")
            qr_positions_url = download_urls.get("qr_positions", f"{self.download_base_url}/qr_positions.yaml")
            
        except json.JSONDecodeError:
            # JSON이 아니면 기본 파일명 사용
            pgm_file = "merged_map.pgm"
            yaml_file = "merged_map.yaml"
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            pgm_url = f"{self.download_base_url}/{pgm_file}"
            yaml_url = f"{self.download_base_url}/{yaml_file}"
            qr_positions_url = f"{self.download_base_url}/qr_positions.yaml"
        
        self.get_logger().info("="*50)
        self.get_logger().info(f"📥 맵 다운로드 시작")
        self.get_logger().info(f"  PGM: {pgm_file}")
        self.get_logger().info(f"  YAML: {yaml_file}")
        self.get_logger().info("="*50)
        self.publish_status(f"맵 다운로드 시작: {pgm_file}")
        
        # PGM 다운로드 (URL 직접 사용)
        pgm_success = self.download_file_from_url(pgm_url, "nav2_map.pgm")
        
        # YAML 다운로드
        yaml_success = self.download_file_from_url(yaml_url, "nav2_map.yaml")
        
        # QR Positions 다운로드 (PORT 좌표)
        qr_success = self.download_file_from_url(qr_positions_url, "qr_positions.yaml")
        if qr_success:
            self.get_logger().info("  ✅ QR positions 다운로드 완료")
        
        if pgm_success and yaml_success:
            self.get_logger().info("✅ 맵 다운로드 완료!")
            self.publish_status("맵 다운로드 완료")
            self.last_download_time = timestamp
            self.current_map_file = os.path.join(self.map_save_dir, "nav2_map.yaml")
            
            # YAML 파일 내 image 경로 수정
            self.fix_yaml_image_path()
            
            # PORT 좌표 파싱 (qr_positions.yaml 우선, 없으면 맵 YAML에서)
            self.parse_port_goals()
            
            # Nav2 map_server에 로드 요청
            self.request_map_load()
            
            # 맵 준비 완료 발행
            msg = Bool()
            msg.data = True
            self.pub_map_ready.publish(msg)
            
        else:
            self.get_logger().error("❌ 맵 다운로드 실패!")
            self.publish_status("맵 다운로드 실패")
    
    def download_file_from_url(self, url, local_filename):
        """HTTP GET으로 파일 다운로드 (전체 URL 사용)"""
        local_path = os.path.join(self.map_save_dir, local_filename)
        
        try:
            self.get_logger().info(f"  다운로드: {url}")
            response = requests.get(url, timeout=30)
            
            if response.status_code == 200:
                with open(local_path, 'wb') as f:
                    f.write(response.content)
                self.get_logger().info(f"  ✅ 저장: {local_path} ({len(response.content)} bytes)")
                return True
            else:
                self.get_logger().error(f"  ❌ HTTP {response.status_code}: {url}")
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"  ❌ 다운로드 실패: {e}")
            return False

    def download_file(self, remote_filename, local_filename):
        """HTTP GET으로 파일 다운로드 (기존 호환용)"""
        url = f"{self.download_base_url}/{remote_filename}"
        return self.download_file_from_url(url, local_filename)
    
    def fix_yaml_image_path(self):
        """YAML 파일을 Nav2 호환 형식으로 수정
        
        Nav2 map_server가 요구하는 형식:
        - image: 절대 경로
        - origin: [x, y, yaw] 인라인 형식
        - ports 같은 추가 필드 제거 (별도 파일로 저장)
        """
        yaml_path = os.path.join(self.map_save_dir, "nav2_map.yaml")
        ports_path = os.path.join(self.map_save_dir, "nav2_map_ports.yaml")
        pgm_path = os.path.join(self.map_save_dir, "nav2_map.pgm")
        
        try:
            import yaml as pyyaml
            
            with open(yaml_path, 'r') as f:
                data = pyyaml.safe_load(f)
            
            # ports 정보 따로 저장 (parse_port_goals에서 사용)
            self.yaml_ports = data.pop('ports', None)
            
            # ★ ports 정보 별도 파일로 저장 (Nav2 goal용) ★
            if self.yaml_ports:
                with open(ports_path, 'w') as f:
                    pyyaml.dump(self.yaml_ports, f, default_flow_style=False)
                self.get_logger().info(f"  📍 PORT 정보 저장: {ports_path}")
            
            # origin을 리스트로 변환 (Nav2 호환)
            if 'origin' in data:
                origin = data['origin']
                if isinstance(origin, list):
                    # 이미 리스트면 OK
                    pass
                elif isinstance(origin, dict):
                    # dict면 리스트로 변환
                    data['origin'] = [origin.get('x', 0), origin.get('y', 0), origin.get('yaw', 0)]
            
            # image 경로 절대 경로로 수정
            data['image'] = pgm_path
            
            # Nav2 호환 YAML로 저장 (인라인 스타일)
            # 직접 문자열로 작성
            nav2_yaml = f"""image: {pgm_path}
resolution: {data.get('resolution', 0.05)}
origin: [{data['origin'][0]}, {data['origin'][1]}, {data['origin'][2]}]
negate: {data.get('negate', 0)}
occupied_thresh: {data.get('occupied_thresh', 0.65)}
free_thresh: {data.get('free_thresh', 0.196)}
"""
            
            with open(yaml_path, 'w') as f:
                f.write(nav2_yaml)
            
            self.get_logger().info(f"  YAML Nav2 형식으로 변환 완료")
            self.get_logger().info(f"  image: {pgm_path}")
            
        except Exception as e:
            self.get_logger().error(f"  YAML 수정 실패: {e}")
    
    def parse_port_goals(self):
        """PORT 좌표 파싱 → Nav2 goal로 사용
        
        우선순위:
        1. qr_positions.yaml (서버에서 다운로드)
        2. nav2_map.yaml 내 ports 섹션
        """
        # 1. qr_positions.yaml 먼저 시도
        qr_positions_path = os.path.join(self.map_save_dir, "qr_positions.yaml")
        if os.path.exists(qr_positions_path):
            try:
                import yaml as pyyaml
                with open(qr_positions_path, 'r') as f:
                    data = pyyaml.safe_load(f)
                
                if data:
                    self.port_goals = {}
                    for port_name, pose in data.items():
                        # 다양한 형식 지원
                        if isinstance(pose, dict):
                            if 'position' in pose:
                                # {position: {x, y, z}, orientation: {w}}
                                pos = pose['position']
                                self.port_goals[port_name] = {
                                    'x': pos.get('x', 0.0),
                                    'y': pos.get('y', 0.0),
                                    'yaw': pose.get('yaw', 0.0)
                                }
                            elif 'x' in pose:
                                # {x, y, yaw}
                                self.port_goals[port_name] = {
                                    'x': pose.get('x', 0.0),
                                    'y': pose.get('y', 0.0),
                                    'yaw': pose.get('yaw', 0.0)
                                }
                    
                    if self.port_goals:
                        self.get_logger().info("="*50)
                        self.get_logger().info("📍 PORT 좌표 로드 (qr_positions.yaml):")
                        for port_name, pose in self.port_goals.items():
                            self.get_logger().info(f"  {port_name}: x={pose['x']:.2f}, y={pose['y']:.2f}, yaw={pose.get('yaw', 0.0):.2f}")
                        self.get_logger().info("="*50)
                        self.publish_port_goals()
                        return
            except Exception as e:
                self.get_logger().warn(f"  qr_positions.yaml 파싱 실패: {e}")
        
        # 2. fix_yaml_image_path에서 추출한 ports 정보 사용
        if hasattr(self, 'yaml_ports') and self.yaml_ports:
            self.port_goals = {}
            for port_name, pose in self.yaml_ports.items():
                if isinstance(pose, dict):
                    self.port_goals[port_name] = {
                        'x': pose.get('x', 0.0),
                        'y': pose.get('y', 0.0),
                        'yaw': pose.get('yaw', 0.0)
                    }
            
            if self.port_goals:
                self.get_logger().info("="*50)
                self.get_logger().info("📍 PORT 좌표 로드 (맵 YAML에서 추출):")
                for port_name, pose in self.port_goals.items():
                    self.get_logger().info(f"  {port_name}: x={pose['x']:.2f}, y={pose['y']:.2f}, yaw={pose.get('yaw', 0.0):.2f}")
                self.get_logger().info("="*50)
                self.publish_port_goals()
                return
        
        self.get_logger().warn("  PORT 정보 없음")
    
    def publish_port_goals(self):
        """PORT 좌표를 ROS 토픽으로 발행"""
        if not self.port_goals:
            return
        
        try:
            msg = String()
            msg.data = json.dumps(self.port_goals)
            # ARUCO.PORT_GOALS 토픽으로 발행
            if not hasattr(self, 'pub_port_goals'):
                from slam_mqtt_project.topics import ARUCO
                self.pub_port_goals = self.create_publisher(String, ARUCO.PORT_GOALS, 10)
            self.pub_port_goals.publish(msg)
            self.get_logger().info(f"📡 PORT_GOALS 발행: {list(self.port_goals.keys())}")
        except Exception as e:
            self.get_logger().error(f"PORT_GOALS 발행 실패: {e}")
    
    def request_map_load(self):
        """Nav2 map_server에 새 맵 로드 요청"""
        # 방법 1: /map_server/load_map 서비스 호출 (nav2_map_server)
        # 방법 2: lifecycle 노드 재시작
        # 방법 3: ROS2 parameter로 yaml 경로 설정
        
        # 여기서는 상태만 발행하고, 실제 로드는 launch 파일이나 별도 스크립트에서 처리
        self.get_logger().info("="*50)
        self.get_logger().info("Nav2 맵 로드 준비 완료")
        self.get_logger().info(f"  맵 파일: {self.current_map_file}")
        self.get_logger().info("  Nav2 bringup 시 이 맵을 사용하세요:")
        self.get_logger().info(f"    map:={self.current_map_file}")
        self.get_logger().info("="*50)
        
        # MQTT로 완료 알림
        try:
            result = {
                "status": "map_loaded",
                "yaml": self.current_map_file,
                "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S")
            }
            self.mqtt_client.publish("robot/map_loaded", json.dumps(result))
        except Exception as e:
            self.get_logger().error(f"MQTT 발행 실패: {e}")
    
    def publish_status(self, message):
        """상태 메시지 발행"""
        msg = String()
        msg.data = f"[MapLoader] {message}"
        self.pub_status.publish(msg)
    
    def destroy_node(self):
        """노드 종료"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotMapLoader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
