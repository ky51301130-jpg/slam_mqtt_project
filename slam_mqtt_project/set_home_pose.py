#!/usr/bin/env python3
"""HOME pose 설정/저장 도구

목표:
- HOME을 ArUco가 아니라 특정 (x, y, yaw) 도달로 판정하기 위해 HOME pose를 파일로 저장

저장 파일:
- ~/.pinky_home_pose.json

사용 예:
- 수동 지정:
  ros2 run slam_mqtt_project set_home_pose --ros-args -p x:=1.23 -p y:=0.45 -p yaw:=0.0

- 현재 /odom 기반으로 자동 저장:
  ros2 run slam_mqtt_project set_home_pose
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SetHomePoseNode(Node):
    def __init__(self):
        super().__init__('set_home_pose')

        self.declare_parameter('x', float('nan'))
        self.declare_parameter('y', float('nan'))
        self.declare_parameter('yaw', float('nan'))  # rad

        self.home_file = os.path.expanduser('~/.pinky_home_pose.json')

        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)

        if not (math.isnan(x) or math.isnan(y) or math.isnan(yaw)):
            self._save(x, y, yaw, source='params')
            rclpy.shutdown()
            return

        self.get_logger().info('No x/y/yaw provided → capturing from /odom (one-shot)')
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.timer = self.create_timer(5.0, self._timeout)
        self._done = False

    def _timeout(self):
        if self._done:
            return
        self.get_logger().error('Timeout waiting for /odom. Provide -p x/y/yaw explicitly.')
        self._done = True
        rclpy.shutdown()

    def _odom_cb(self, msg: Odometry):
        if self._done:
            return
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self._save(x, y, yaw, source='odom')
        self._done = True
        rclpy.shutdown()

    def _save(self, x: float, y: float, yaw: float, source: str):
        os.makedirs(os.path.dirname(self.home_file) or '.', exist_ok=True)
        data = {
            'x': round(x, 6),
            'y': round(y, 6),
            'yaw': round(yaw, 6),
            'yaw_deg': round(math.degrees(yaw), 2),
            'frame': 'odom',
            'source': source,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        with open(self.home_file, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(
            f"Saved HOME pose to {self.home_file}: (x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SetHomePoseNode()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
