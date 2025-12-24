#!/usr/bin/env python3
"""
Nav2 Goal Node - MQTT/PLC 명령을 Nav2 Goal로 변환 + ArUco 도킹 연계

역할:
1. MQTT에서 Goal 명령 수신 (plc/location, plc/goal)
2. port_goals.json에서 저장된 포트 위치 로드
3. Nav2 NavigateToPose 액션 호출
4. Nav2 도착 후 ArUco 정밀 도킹 트리거 (/target_port, /aruco/dock_enable)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool

import json
import math
import os
import paho.mqtt.client as mqtt
from threading import Thread
from queue import Queue, Empty
import time

# 중앙 집중식 토픽 관리
from slam_mqtt_project.topics import ROS, MQTT, ARUCO

# ============== Constants ==============
FEEDBACK_INTERVAL = 2.0
PORT_GOALS_FILE = "/home/pinky/saved_maps/port_goals.json"
# 서버에서 받은 정제된 PORT 좌표 (우선순위 높음)
RENEW_PORT_FILE = "/home/pinky/saved_maps/renew/qr_positions.yaml"


class Nav2GoalNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_node')
        
        # === Nav2 Action Client ===
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        
        # === 상태 변수 ===
        self.current_goal = None
        self.current_target_port = None  # 현재 목표 포트 이름
        self.is_navigating = False
        self.last_feedback_time = 0.0
        
        # === ROS2 Publishers ===
        self.status_pub = self.create_publisher(String, ROS.NAV2_STATUS, 10)
        self.arrived_pub = self.create_publisher(Bool, ROS.NAV2_ARRIVED, 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, ROS.INITIAL_POSE, 10
        )
        # ArUco 연동
        self.target_port_pub = self.create_publisher(String, ARUCO.TARGET_PORT, 10)
        self.dock_enable_pub = self.create_publisher(Bool, ARUCO.DOCK_ENABLE, 10)
        
        # === ROS2 Subscribers ===
        self.create_subscription(PoseStamped, ROS.NAV2_GOAL, self.ros_goal_cb, 10)
        self.create_subscription(Bool, ROS.NAV2_CANCEL, self.cancel_cb, 10)
        # 포트 위치 업데이트 구독 (ArUco 노드에서 발행)
        self.create_subscription(String, ARUCO.PORT_GOALS, self.port_goals_update_cb, 10)
        
        # === MQTT 설정 ===
        self.mqtt = mqtt.Client()
        self.mqtt.on_connect = self.on_mqtt_connect
        self.mqtt.on_message = self.on_mqtt_message
        Thread(target=self.mqtt_loop, daemon=True).start()
        
        # === 포트 위치 (port_goals.json에서 로드) ===
        self.locations = self._load_port_goals()
        
        self.get_logger().info(f"Nav2 Goal Node Started (MQTT: {MQTT.HOST})")
        self.get_logger().info(f"Loaded Ports: {list(self.locations.keys())}")
    
    def _load_port_goals(self) -> dict:
        """서버에서 받은 정제된 PORT 좌표 로드 (우선순위: renew > port_goals.json)"""
        import yaml
        import math
        
        locations = {
            'HOME': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},  # 기본 홈 위치
        }
        
        # 1. 서버에서 받은 정제된 좌표 (qr_positions.yaml) 우선 로드
        try:
            if os.path.exists(RENEW_PORT_FILE):
                with open(RENEW_PORT_FILE, 'r') as f:
                    data = yaml.safe_load(f)
                    if data:
                        for name, pos in data.items():
                            # orientation에서 yaw 계산 (quaternion → euler)
                            orientation = pos.get('orientation', {})
                            position = pos.get('position', {})
                            
                            # quaternion to yaw
                            w = orientation.get('w', 1.0)
                            z = orientation.get('z', 0.0)
                            yaw = 2.0 * math.atan2(z, w)
                            
                            locations[name.upper()] = {
                                'x': float(position.get('x', 0)),
                                'y': float(position.get('y', 0)),
                                'yaw': yaw
                            }
                        self.get_logger().info(f"📂 Loaded {len(data)} ports from {RENEW_PORT_FILE} (서버 정제 좌표)")
                        return locations
        except Exception as e:
            self.get_logger().warn(f"Failed to load renew port file: {e}")
        
        # 2. Fallback: 기존 port_goals.json
        try:
            if os.path.exists(PORT_GOALS_FILE):
                with open(PORT_GOALS_FILE, 'r') as f:
                    data = json.load(f)
                    for name, pos in data.items():
                        locations[name.upper()] = {
                            'x': float(pos.get('x', 0)),
                            'y': float(pos.get('y', 0)),
                            'yaw': float(pos.get('yaw', 0))
                        }
                self.get_logger().info(f"📂 Loaded {len(data)} ports from {PORT_GOALS_FILE} (fallback)")
        except Exception as e:
            self.get_logger().warn(f"Failed to load port goals: {e}")
        return locations
    
    def port_goals_update_cb(self, msg: String):
        """ArUco 노드에서 포트 위치 업데이트 수신"""
        try:
            data = json.loads(msg.data)
            for name, pos in data.items():
                self.locations[name.upper()] = {
                    'x': float(pos.get('x', 0)),
                    'y': float(pos.get('y', 0)),
                    'yaw': float(pos.get('yaw', 0))
                }
            self.get_logger().info(f"🔄 Port goals updated: {list(self.locations.keys())}")
        except Exception as e:
            self.get_logger().warn(f"Port goals update error: {e}")
    
    # ==================== MQTT ====================
    
    def mqtt_loop(self):
        while rclpy.ok():
            try:
                self.mqtt.connect(MQTT.HOST, MQTT.PORT, 60)
                self.mqtt.loop_forever()
            except Exception as e:
                self.get_logger().warn(f"MQTT fail: {e}, retry 10s")
                time.sleep(10)
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            topics = [
                MQTT.SUB_PLC_LOCATION,   # 위치 이름: 'A' 또는 'B'
                MQTT.SUB_PLC_GOAL,       # 좌표 (예: {"x":1.0, "y":2.0, "yaw":0})
                MQTT.SUB_ROBOT_NAV,      # PLC → robot/navigate_to_pose
                "nav2/initial_pose",     # 초기 위치 설정
            ]
            for topic in topics:
                client.subscribe(topic, qos=1)
            self.get_logger().info(f"MQTT subscribed: {topics}")
        else:
            self.get_logger().error(f"MQTT error: {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            topic = msg.topic
            
            if topic == MQTT.SUB_PLC_LOCATION or topic == MQTT.SUB_ROBOT_NAV:
                self.handle_location_command(payload)
            elif topic == MQTT.SUB_PLC_GOAL:
                self.handle_goal_command(payload)
            elif topic == "nav2/initial_pose":
                self.handle_initial_pose(payload)
        except Exception as e:
            self.get_logger().error(f"MQTT msg error: {e}")
    
    # ==================== Commands ====================
    
    def handle_location_command(self, payload: str):
        """
        PLC 위치 명령 처리
        
        PLC 포맷:
          - 단일 문자: 'A' → PORT_A, 'B' → PORT_B
          - JSON (PLC): {"A":1,"B":0} → PORT_A로 이동
          - JSON (ArUco): {"type":"aruco_port", "port":"A", "position":{...}} → 좌표로 이동
          - 문자열: "PORT_A", "HOME" 등
        """
        payload = payload.strip()
        self.get_logger().info(f"📨 PLC 명령 수신: '{payload}'")
        
        # 취소 명령 체크
        if payload.lower() in ['cancel', 'stop']:
            self.cancel_goal()
            return
        
        # 단일 문자 처리: 'A' → PORT_A, 'B' → PORT_B
        if len(payload) == 1 and payload.upper() in ['A', 'B']:
            target = f"PORT_{payload.upper()}"
            self.get_logger().info(f"🎯 단일 문자 '{payload}' → {target}")
            self._goto_port(target)
            return
        
        # JSON 포맷 처리
        if payload.startswith('{'):
            try:
                data = json.loads(payload)
                
                # ArUco 포트 형식: {"type":"aruco_port", "port":"A", "position":{x,y,z}, "orientation":{w,x,y,z}}
                if data.get('type') == 'aruco_port':
                    port = data.get('port', '').upper()
                    position = data.get('position', {})
                    orientation = data.get('orientation', {})
                    
                    x = float(position.get('x', 0))
                    y = float(position.get('y', 0))
                    
                    # quaternion to yaw
                    w = float(orientation.get('w', 1.0))
                    z = float(orientation.get('z', 0.0))
                    yaw = 2.0 * math.atan2(z, w)
                    
                    target = f"PORT_{port}" if len(port) == 1 else port
                    self.current_target_port = target
                    self.target_port_pub.publish(String(data=target))
                    
                    self.get_logger().info(f"🎯 ArUco 포트 '{port}' → ({x:.3f}, {y:.3f}, yaw={yaw:.2f})")
                    self.send_goal(x, y, yaw)
                    return
                
                # 기존 PLC 형식: {"A":1,"B":0}
                target = None
                for key, value in data.items():
                    if value == 1:
                        # A → PORT_A, B → PORT_B, HOME → HOME
                        if key.upper() in ['A', 'B']:
                            target = f"PORT_{key.upper()}"
                        else:
                            target = key.upper()
                        break
                
                if target is None:
                    self.get_logger().warn(f"No destination with value 1: {data}")
                    return
                
                self._goto_port(target)
                    
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON parse error: {e}")
            return
        
        # 문자열 포맷: "PORT_A", "HOME" 등
        target = payload.upper()
        self._goto_port(target)
    
    def _goto_port(self, port_name: str):
        """포트로 이동 + ArUco 도킹 연계"""
        if port_name not in self.locations:
            self.get_logger().warn(f"Unknown port: {port_name}")
            self.get_logger().info(f"Available: {list(self.locations.keys())}")
            return
        
        loc = self.locations[port_name]
        self.current_target_port = port_name
        
        # ArUco 노드에 목표 포트 알림
        self.target_port_pub.publish(String(data=port_name))
        
        self.get_logger().info(f"🎯 Going to: {port_name} ({loc['x']:.2f}, {loc['y']:.2f})")
        self.send_goal(loc['x'], loc['y'], loc['yaw'])
    
    def handle_goal_command(self, payload: str):
        """좌표로 이동 (예: {"x":1.0, "y":2.0, "yaw":0})"""
        try:
            data = json.loads(payload)
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            yaw = float(data.get('yaw', 0))
            self.current_target_port = None  # 좌표 이동은 포트 없음
            self.send_goal(x, y, yaw)
        except Exception as e:
            self.get_logger().error(f"Goal parse error: {e}")
    
    def handle_initial_pose(self, payload: str):
        """AMCL 초기 위치 설정"""
        try:
            data = json.loads(payload)
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            yaw = float(data.get('yaw', 0))
            self.set_initial_pose(x, y, yaw)
        except Exception as e:
            self.get_logger().error(f"Initial pose error: {e}")
    
    # ==================== ROS2 Callbacks ====================
    
    def ros_goal_cb(self, msg: PoseStamped):
        """ROS2 토픽으로 Goal 수신"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.send_goal(x, y, yaw)
    
    def cancel_cb(self, msg: Bool):
        if msg.data:
            self.cancel_goal()
    
    # ==================== Nav2 Navigation ====================
    
    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """Nav2에 Goal 전송 - BT Navigator가 알아서 처리"""
        if self.is_navigating:
            self.cancel_goal()
            time.sleep(0.5)
        
        # Nav2 액션 서버 대기
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            self.publish_status('ERROR', 'Nav2 not available')
            return
        
        # Goal 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.current_goal = {'x': x, 'y': y, 'yaw': yaw}
        self.is_navigating = True
        
        self.get_logger().info(f"🚀 Goal: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")
        self.publish_status('NAVIGATING', f'({x:.2f}, {y:.2f})')
        
        # 비동기 Goal 전송
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)
    
    def goal_response_cb(self, future):
        """Goal 수락/거절 응답"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.is_navigating = False
            self.publish_status('REJECTED', 'Goal rejected')
            return
        
        self._goal_handle = goal_handle
        self.get_logger().info("Goal accepted, navigating...")
        goal_handle.get_result_async().add_done_callback(self.result_cb)
    
    def feedback_cb(self, feedback_msg):
        """진행 상황 피드백"""
        fb = feedback_msg.feedback
        now = time.time()
        
        if now - self.last_feedback_time > FEEDBACK_INTERVAL:
            pos = fb.current_pose.pose.position
            dist = fb.distance_remaining
            self.get_logger().info(f"📍 ({pos.x:.2f}, {pos.y:.2f}), remain: {dist:.2f}m")
            self.last_feedback_time = now
    
    def result_cb(self, future):
        """네비게이션 결과"""
        status = future.result().status
        self.is_navigating = False
        
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: ('SUCCEEDED', '🎉 Goal reached!'),
            GoalStatus.STATUS_CANCELED: ('CANCELED', 'Canceled'),
            GoalStatus.STATUS_ABORTED: ('ABORTED', 'Failed - path blocked?'),
        }
        
        st, msg = status_map.get(status, ('UNKNOWN', f'Status: {status}'))
        self.get_logger().info(f"Result: {msg}")
        self.publish_status(st, msg)
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.arrived_pub.publish(Bool(data=True))
            
            # 포트로 이동했으면 ArUco 정밀 도킹 활성화
            if self.current_target_port:
                self.get_logger().info(f"📍 Near {self.current_target_port}, enabling ArUco docking...")
                self.dock_enable_pub.publish(Bool(data=True))
            
            try:
                self.mqtt.publish('robot/arrived', json.dumps({
                    'goal': self.current_goal,
                    'port': self.current_target_port,
                    'status': 'arrived'
                }), qos=1)
            except:
                pass
    
    def cancel_goal(self):
        """Goal 취소"""
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self.get_logger().info("Goal canceled")
        self.is_navigating = False
        self.publish_status('CANCELED', 'Canceled')
    
    # ==================== Initial Pose ====================
    
    def set_initial_pose(self, x: float, y: float, yaw: float):
        """AMCL에 초기 위치 발행"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance (작은 불확실성)
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.06  # yaw
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"📍 Initial pose: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")
    
    # ==================== Status ====================
    
    def publish_status(self, status: str, message: str):
        """상태 발행 (ROS2 + MQTT)"""
        data = {
            'status': status,
            'message': message,
            'goal': self.current_goal,
        }
        
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)
        
        try:
            self.mqtt.publish('robot/nav_status', msg.data, qos=1)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalNode()
    
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
