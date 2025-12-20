#!/usr/bin/env python3
"""
HOME 위치 수동 설정 Launch 파일

사용법:
    ros2 launch slam_mqtt_project set_home.launch.py

동작:
1. pinky_bringup (모터, LiDAR, TF) 시작
2. ArUco ID 0 마커 감지 대기
3. 마커 감지 시 현재 위치를 HOME으로 저장
4. 저장 완료 후 종료

이후:
    ros2 launch slam_mqtt_project slam_exploration.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    
    # ===== 1. Pinky Bringup (모터, LiDAR, TF) =====
    bringup_node = Node(
        package='pinky_bringup',
        executable='bringup',
        name='pinky_bringup',
        output='screen',
        parameters=[{
            'wheel_separation': 0.17,
            'wheel_radius': 0.033,
            'encoder_resolution': 440.0,
            'max_speed': 0.6,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }]
    )
    
    # ===== 2. Set HOME by ArUco (3초 후 시작 - bringup 안정화 대기) =====
    set_home_node = Node(
        package='slam_mqtt_project',
        executable='set_home_by_aruco',
        name='set_home_by_aruco',
        output='screen',
    )
    
    return LaunchDescription([
        LogInfo(msg='=' * 60),
        LogInfo(msg='🏠 HOME 설정 모드'),
        LogInfo(msg='   1. ArUco ID 0 마커를 로봇 앞에 놓으세요'),
        LogInfo(msg='   2. 마커 감지 시 현재 위치가 HOME으로 저장됩니다'),
        LogInfo(msg='=' * 60),
        
        # Bringup 먼저 시작
        bringup_node,
        
        # 3초 후 ArUco HOME 설정 시작
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='🔍 ArUco 마커 탐색 시작...'),
                set_home_node,
            ]
        ),
    ])
