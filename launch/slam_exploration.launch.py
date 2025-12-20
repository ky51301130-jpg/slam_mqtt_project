#!/usr/bin/env python3
"""
SLAM 탐색 모드 Launch 파일 (경량화 버전)
- 실행 전 기존 프로세스 자동 정리
- bringup_robot.launch.xml: LiDAR, 모터, TF
- map_building.launch.xml: SLAM Toolbox
- 통합 노드: auto_drive (초음파 포함), status_display (LED+LCD), map_saver
- 종료 시 모터 정지 보장
- TimerAction으로 순차 실행 (TF 안정화)
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import subprocess
import time


def cleanup_existing_processes(context, *args, **kwargs):
    """기존 ROS2/SLAM 프로세스 정리"""
    processes_to_kill = [
        # SLAM
        "slam_toolbox", "async_slam_toolbox", "sync_slam_toolbox",
        # Nav2
        "nav2", "bt_navigator", "planner_server", "controller_server",
        "amcl", "map_server", "lifecycle_manager",
        # 로봇 드라이버
        "robot_state_publisher", "joint_state_publisher",
        "ld19", "ldlidar", "rplidar", "sllidar",
        # slam_mqtt_project 노드
        "mqtt_bridge", "auto_drive", "ultrasonic", "map_saver",
        "led_controller", "collision_photo", "nav2_goal", "lcd_status",
        "status_display",
        # 모터
        "pinky_motor", "motor_node",
    ]
    
    print("\n" + "="*50)
    print("  🧹 기존 프로세스 정리 중...")
    print("="*50)
    
    for proc in processes_to_kill:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", proc],
                capture_output=True, timeout=2
            )
        except:
            pass
    
    # ROS2 daemon 재시작
    try:
        subprocess.run(["ros2", "daemon", "stop"], capture_output=True, timeout=5)
        time.sleep(0.5)
        subprocess.run(["ros2", "daemon", "start"], capture_output=True, timeout=5)
    except:
        pass
    
    # 공유 메모리 정리
    try:
        subprocess.run("rm -rf /dev/shm/fastrtps_* 2>/dev/null", shell=True, timeout=2)
    except:
        pass
    
    print("  ✓ 정리 완료, 2초 대기...")
    time.sleep(2)
    print("  ✓ SLAM 시작!\n")
    
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. 로봇 Bringup (LiDAR, 모터, TF)
    bringup_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pinky_bringup'),
                'launch',
                'bringup_robot.launch.xml'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pinky_navigation'),
                'launch',
                'map_building.launch.xml'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. 자동 탐색 노드 (초음파 통합)
    auto_drive_node = Node(
        package='slam_mqtt_project',
        executable='slam_auto_drive',
        name='slam_auto_drive',
        output='screen',
    )

    # 4. 맵 저장 노드
    map_saver_node = Node(
        package='slam_mqtt_project',
        executable='slam_map_saver',
        name='slam_map_saver',
        output='screen',
    )

    # 5. 상태 표시 통합 노드 (LED + LCD + 모드 발행)
    status_display_node = Node(
        package='slam_mqtt_project',
        executable='status_display',
        name='status_display',
        output='screen',
        parameters=[{'mode': 'SLAM'}],
    )

    # 6. MQTT 브릿지 (서버와 통신)
    mqtt_bridge_node = Node(
        package='slam_mqtt_project',
        executable='mqtt_bridge',
        name='mqtt_bridge',
        output='screen',
    )

    # 7. 충돌 사진 노드 (SLAM 모드 - 포트 5001)
    collision_photo_node = Node(
        package='slam_mqtt_project',
        executable='slam_collision_photo',
        name='slam_collision_photo',
        output='screen',
    )

    # 종료 시 모터 정지 명령 (백업)
    motor_stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/cmd_vel', 
             'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen',
        shell=False,
    )

    # auto_drive_node 종료 시 모터 정지 명령 실행
    motor_stop_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=auto_drive_node,
            on_exit=[motor_stop_cmd],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # 0. 기존 프로세스 정리 (가장 먼저 실행)
        OpaqueFunction(function=cleanup_existing_processes),
        
        # 1. Bringup 먼저 (LiDAR, TF, 모터)
        bringup_launch,
        
        # 2. SLAM Toolbox (3초 후 - TF 안정화 대기)
        TimerAction(period=3.0, actions=[slam_launch]),
        
        # 3. 상태 표시 + MQTT (4초 후)
        TimerAction(
            period=4.0,
            actions=[
                status_display_node,
                mqtt_bridge_node,
                collision_photo_node,
            ]
        ),
        
        # 4. 자동 탐색 + 맵 저장 (5초 후 - SLAM 준비 대기)
        TimerAction(
            period=5.0,
            actions=[
                auto_drive_node,
                map_saver_node,
            ]
        ),
        
        motor_stop_on_exit,
    ])
