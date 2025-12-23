#!/usr/bin/env python3
"""
Nav2 네비게이션 모드 Launch 파일
- pinky_navigation의 기본 Nav2 스택 사용
- bringup_robot.launch.xml: LiDAR, 모터, TF
- pinky_navigation/bringup_launch.xml: AMCL, Planner, Controller, BT Navigator
- 추가 노드: nav2_goal, ultrasonic, status_display, mqtt_bridge
- 종료 시 모터 정지 보장

실행:
  ros2 launch slam_mqtt_project nav2_mode.launch.py
  ros2 launch slam_mqtt_project nav2_mode.launch.py map:=/path/to/map.yaml
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
    """기존 ROS2/Nav2 프로세스 정리 (현재 launch 제외)"""
    processes_to_kill = [
        # SLAM
        "slam_toolbox", "async_slam_toolbox", "sync_slam_toolbox",
        # Nav2 (component_container 내부 노드들)
        "bt_navigator", "planner_server", "controller_server",
        "amcl", "map_server", "lifecycle_manager", "behavior_server",
        "waypoint_follower", "velocity_smoother", "smoother_server",
        # 로봇 드라이버
        "robot_state_publisher", "joint_state_publisher",
        "ld19", "ldlidar", "rplidar", "sllidar_node",
        # slam_mqtt_project 노드
        "mqtt_bridge", "auto_drive", "ultrasonic", "map_saver",
        "led_controller", "collision_photo", "nav2_goal", "lcd_status",
        "status_display", "nav2_aruco_dock", "robot_map_loader",
        "nav2_camera_stream",
        # 모터
        "pinky_motor", "motor_node", "pinky_bringup",
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
    
    # component_container 정리 (nav2_container)
    try:
        subprocess.run(
            ["pkill", "-9", "-f", "component_container_isolated"],
            capture_output=True, timeout=2
        )
    except:
        pass
    
    # 공유 메모리 정리
    try:
        subprocess.run("rm -rf /dev/shm/fastrtps_* 2>/dev/null", shell=True, timeout=2)
    except:
        pass
    
    print("  ✓ 정리 완료, 2초 대기...")
    time.sleep(2)
    print("  ✓ Nav2 시작!\n")
    
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 기본 맵: robot_map_loader가 다운로드하는 위치
    map_yaml = LaunchConfiguration('map', default='/home/pinky/saved_maps/renew/nav2_map.yaml')
    autostart = LaunchConfiguration('autostart', default='true')

    # 패키지 경로 (pinky_navigation 사용)
    pinky_nav_pkg = FindPackageShare('pinky_navigation')

    # ===== 1. 로봇 Bringup (LiDAR, 모터, TF) =====
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

    # ===== 2. Nav2 전체 스택 - pinky_navigation 사용 =====
    # pinky_navigation/launch/bringup_launch.xml 사용 (AMCL, Planner, Controller, BT Navigator)
    nav2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                pinky_nav_pkg,
                'launch',
                'bringup_launch.xml'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'autostart': autostart,
        }.items()
    )

    # ===== 3. Nav2 Goal Node (MQTT → Nav2 Goal 연동) =====
    nav2_goal_node = Node(
        package='slam_mqtt_project',
        executable='nav2_goal',
        name='nav2_goal',
        output='screen',
    )

    # ===== 4. MQTT 브릿지 (서버와 통신) =====
    mqtt_bridge_node = Node(
        package='slam_mqtt_project',
        executable='mqtt_bridge',
        name='mqtt_bridge',
        output='screen',
    )

    # ===== 5. LED + LCD 상태 표시 (통합) =====
    status_display_node = Node(
        package='slam_mqtt_project',
        executable='status_display',
        name='status_display',
        output='screen',
    )

    # ===== 7. 카메라 스트리밍 (Nav2 모드 - YOLO 분석용) =====
    camera_stream_node = Node(
        package='slam_mqtt_project',
        executable='nav2_camera_stream',
        name='nav2_camera_stream',
        output='screen',
        parameters=[{
            'fps': 1.0,
            'width': 320,
            'height': 240,
            'port': 5200,  # collision_photo는 5000 사용
        }],
    )

    # ===== 10. ArUco 도킹 노드 (정밀 포트 접근) =====
    aruco_dock_node = Node(
        package='slam_mqtt_project',
        executable='nav2_aruco_dock',
        name='nav2_aruco_dock',
        output='screen',
    )

    # ===== 11. Robot Map Loader (서버에서 맵 다운로드) =====
    map_loader_node = Node(
        package='slam_mqtt_project',
        executable='robot_map_loader',
        name='robot_map_loader',
        output='screen',
    )

    # ===== 9. 모드 발행 (LCD에 NAV2 모드 표시) =====
    mode_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-r', '0.1', '/robot_mode', 
             'std_msgs/msg/String', '{data: "NAV2"}'],
        output='screen',
    )

    # ===== 종료 시 모터 정지 (안전장치) =====
    motor_stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/cmd_vel', 
             'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen',
        shell=False,
    )

    # nav2_goal_node 종료 시 모터 정지
    motor_stop_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=nav2_goal_node,
            on_exit=[motor_stop_cmd],
        )
    )

    return LaunchDescription([
        # ========== 0. 기존 프로세스 정리 ==========
        OpaqueFunction(function=cleanup_existing_processes),
        
        # ========== Launch Arguments ==========
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/pinky/saved_maps/renew/nav2_map.yaml',
            description='Path to map yaml file (downloaded by robot_map_loader)'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start Nav2 lifecycle nodes'
        ),
        
        # ========== Launch Sequence ==========
        LogInfo(msg='🚀 Starting Nav2 Navigation Mode (pinky_navigation)...'),
        
        # 1. 모드 발행 시작 (LCD에 NAV2 표시)
        mode_publisher,
        
        # 2. Bringup 먼저 (LiDAR, TF, 모터)
        bringup_launch,
        
        # 3. Nav2 전체 스택 (3초 후 - TF 안정화 대기)
        #    pinky_navigation/bringup_launch.xml 사용
        TimerAction(period=3.0, actions=[
            LogInfo(msg='🗺️ Starting Nav2 Stack (pinky_navigation)...'),
            nav2_launch
        ]),
        
        # 4. 센서 + 상태 노드들 (4초 후)
        TimerAction(
            period=4.0,
            actions=[
                status_display_node,
                mqtt_bridge_node,
                camera_stream_node,
            ]
        ),
        
        # 5. Nav2 Goal Node + ArUco Dock Node + Map Loader (8초 후 - Nav2 준비 완료 대기)
        TimerAction(
            period=8.0, 
            actions=[
                LogInfo(msg='🎯 Starting Nav2 Goal Node (MQTT → Nav2)...'),
                nav2_goal_node,
                LogInfo(msg='🎯 Starting ArUco Dock Node (정밀 도킹)...'),
                aruco_dock_node,
                LogInfo(msg='📥 Starting Robot Map Loader (서버에서 맵 수신)...'),
                map_loader_node,
            ]
        ),
        
        # 종료 시 모터 정지
        motor_stop_on_exit,
    ])
