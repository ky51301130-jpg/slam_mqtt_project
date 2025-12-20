#!/usr/bin/env python3
"""
MQTT Bridge Node - 완전한 양방향 통신
PLC/MCU/Server ↔ ROS2 토픽 브릿지

토픽 정의는 topics.py 참조
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32, Bool
import paho.mqtt.client as mqtt
import threading
import time
import json

# 중앙 집중식 토픽 관리
from slam_mqtt_project.topics import ROS, MQTT


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__("mqtt_bridge_node")

        # ==================== ROS2 Publishers (MQTT → ROS2) ====================
        self.pub_mcu = self.create_publisher(String, ROS.MQTT_MCU_SENSORS, 10)
        self.pub_plc_location = self.create_publisher(String, ROS.MQTT_PLC_LOCATION, 10)
        self.pub_plc_goal = self.create_publisher(String, ROS.MQTT_PLC_GOAL, 10)

        # ==================== ROS2 Subscribers (ROS2 → MQTT) ====================
        # SLAM/Map 관련
        self.create_subscription(String, ROS.MAP_SAVER_CYCLE, self.cycle_complete_cb, 10)
        self.create_subscription(String, ROS.COLLISION_PHOTO_READY, self.collision_photo_cb, 10)
        
        # 로봇 상태
        self.create_subscription(String, ROS.ROBOT_MODE, self.robot_mode_cb, 10)
        
        # Nav2 상태
        self.create_subscription(String, ROS.NAV2_STATUS, self.nav_status_cb, 10)
        self.create_subscription(Bool, ROS.NAV2_ARRIVED, self.nav_arrived_cb, 10)
        
        # QR 장애물 감지
        self.create_subscription(String, ROS.QR_OBSTACLE, self.qr_obstacle_cb, 10)
        
        # 배터리
        self.create_subscription(Float32, ROS.BATTERY_VOLTAGE, self.battery_cb, 10)

        # ==================== MQTT Client ====================
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_connected = False
        threading.Thread(target=self.start_mqtt, daemon=True).start()
        
        # 상태 캐시 (중복 발행 방지)
        self.last_mode = ""
        
        self.get_logger().info("MQTT Bridge Started (Full Monitoring)")

    # ==================== MQTT Connection ====================
    
    def start_mqtt(self):
        """MQTT 연결 (재시도 포함)"""
        while rclpy.ok():
            try:
                self.mqtt_client.connect(MQTT.HOST, MQTT.PORT, 60)
                self.mqtt_client.loop_forever()
            except Exception as e:
                self.mqtt_connected = False
                self.get_logger().warn(f"MQTT fail: {e}, retry 10s")
                time.sleep(10)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            for topic in MQTT.SUBSCRIBE_LIST:
                client.subscribe(topic, qos=1)
            self.get_logger().info(f"MQTT connected, subscribed: {MQTT.SUBSCRIBE_LIST}")
        else:
            self.mqtt_connected = False
            self.get_logger().error(f"MQTT connect error: {rc}")

    def on_message(self, client, userdata, msg):
        """MQTT → ROS2"""
        try:
            payload = msg.payload.decode("utf-8")
            topic = msg.topic
            
            # MCU 센서 데이터
            if topic.startswith("mcu/"):
                self.pub_mcu.publish(String(data=payload))
            
            # PLC 명령
            elif topic == MQTT.SUB_PLC_LOCATION:
                self.pub_plc_location.publish(String(data=payload))
                # 모니터링용 재발행
                self.mqtt_publish(MQTT.NAVIGATE_TO_POSE, json.dumps({
                    "type": "location",
                    "value": payload,
                    "timestamp": time.time()
                }))
            
            elif topic == MQTT.SUB_PLC_GOAL:
                self.pub_plc_goal.publish(String(data=payload))
                # 모니터링용 재발행
                self.mqtt_publish(MQTT.NAVIGATE_TO_POSE, json.dumps({
                    "type": "coordinate",
                    "value": payload,
                    "timestamp": time.time()
                }))
                
        except Exception as e:
            self.get_logger().error(f"MQTT msg error: {e}")

    def mqtt_publish(self, topic: str, payload: str, qos: int = 1):
        """안전한 MQTT 발행"""
        if self.mqtt_connected:
            try:
                self.mqtt_client.publish(topic, payload, qos=qos)
            except Exception as e:
                self.get_logger().error(f"MQTT publish error [{topic}]: {e}")

    # ==================== ROS2 → MQTT Callbacks ====================
    
    def cycle_complete_cb(self, msg: String):
        """맵 저장 사이클 완료"""
        self.mqtt_publish(MQTT.MAP_CYCLE_COMPLETE, msg.data)
        self.get_logger().info(f"→ MQTT: {MQTT.MAP_CYCLE_COMPLETE}")

    def collision_photo_cb(self, msg: String):
        """충돌 사진 촬영 완료"""
        self.mqtt_publish(MQTT.COLLISION_PHOTO, msg.data)
        self.get_logger().info(f"→ MQTT: {MQTT.COLLISION_PHOTO}")

    def robot_mode_cb(self, msg: String):
        """로봇 모드 (SLAM/NAV2/IDLE)"""
        mode = msg.data.upper()
        if mode != self.last_mode:
            self.last_mode = mode
            self.mqtt_publish(MQTT.SLAM_MODE, json.dumps({
                "mode": mode,
                "timestamp": time.time()
            }))
            self.get_logger().info(f"→ MQTT: {MQTT.SLAM_MODE} = {mode}")

    def nav_status_cb(self, msg: String):
        """Nav2 상태 (진행/완료/실패)"""
        self.mqtt_publish(MQTT.NAV_STATUS, msg.data)
        
        # nav_result로도 발행 (결과 전용)
        try:
            data = json.loads(msg.data)
            status = data.get("status", "")
            if status in ["SUCCEEDED", "ABORTED", "CANCELED", "REJECTED"]:
                self.mqtt_publish(MQTT.NAV_RESULT, json.dumps({
                    "result": status,
                    "message": data.get("message", ""),
                    "goal": data.get("goal", {}),
                    "timestamp": time.time()
                }))
                self.get_logger().info(f"→ MQTT: {MQTT.NAV_RESULT} = {status}")
        except:
            pass

    def nav_arrived_cb(self, msg: Bool):
        """목표 도착"""
        if msg.data:
            self.mqtt_publish(MQTT.NAV_ARRIVED, json.dumps({
                "arrived": True,
                "timestamp": time.time()
            }))

    def qr_obstacle_cb(self, msg: String):
        """QR 장애물 감지"""
        self.mqtt_publish(MQTT.QR_OBSTACLE, msg.data)
        self.get_logger().info(f"→ MQTT: {MQTT.QR_OBSTACLE}")

    def battery_cb(self, msg: Float32):
        """배터리 전압"""
        self.mqtt_publish(MQTT.BATTERY_STATUS, json.dumps({
            "voltage": round(msg.data, 2),
            "timestamp": time.time()
        }))


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
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


if __name__ == "__main__":
    main()
