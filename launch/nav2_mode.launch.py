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


def download_map_before_nav2(context, *args, **kwargs):
    """Nav2 시작 전에 서버에서 맵 다운로드 (/status API 사용)"""
    import requests
    import os
    import re
    
    map_dir = "/home/pinky/saved_maps/renew"
    map_yaml = os.path.join(map_dir, "nav2_map.yaml")
    map_pgm = os.path.join(map_dir, "nav2_map.pgm")
    server_base = "http://192.168.0.3:5100"
    
    os.makedirs(map_dir, exist_ok=True)
    
    print("\n" + "="*50)
    print("  📥 Nav2 맵 다운로드 시작...")
    print("="*50)
    
    # 기존 맵이 있는지 확인
    if os.path.exists(map_yaml) and os.path.exists(map_pgm):
        print(f"  ✓ 기존 맵 발견: {map_yaml}")
        print("  ✓ 서버에서 최신 맵 확인 시도...")
    
    # 서버에서 최신 맵 파일명 찾기 (/status API 사용)
    try:
        # /status API에서 maps.latest.name 으로 최신 맵 파일명 가져오기
        status_url = f"{server_base}/status"
        resp = requests.get(status_url, timeout=5)
        
        yaml_filename = None
        pgm_filename = None
        
        if resp.status_code == 200:
            status_data = resp.json()
            # maps.latest.name 에서 파일명 추출
            if 'maps' in status_data and 'latest' in status_data['maps']:
                yaml_filename = status_data['maps']['latest'].get('name')
                if yaml_filename:
                    pgm_filename = yaml_filename.replace('.yaml', '.pgm')
                    print(f"  ✓ 최신 맵 발견: {yaml_filename}")
        
        if not yaml_filename:
            print("  ⚠️ 서버에서 맵 파일명을 찾을 수 없음")
            if os.path.exists(map_yaml):
                print(f"  ✓ 기존 맵 사용: {map_yaml}")
            return []
        
        # YAML 다운로드
        yaml_url = f"{server_base}/download/{yaml_filename}"
        resp = requests.get(yaml_url, timeout=10)
        if resp.status_code == 200:
            # YAML 내용에서 image 경로를 로컬 파일명으로 수정
            yaml_content = resp.text
            yaml_content = re.sub(r'image:\s*\S+', 'image: nav2_map.pgm', yaml_content)
            with open(map_yaml, 'w') as f:
                f.write(yaml_content)
            print(f"  ✓ YAML 다운로드 완료: {map_yaml}")
        else:
            print(f"  ⚠️ YAML 다운로드 실패: HTTP {resp.status_code}")
            return []
        
        # PGM 다운로드
        pgm_url = f"{server_base}/download/{pgm_filename}"
        resp = requests.get(pgm_url, timeout=10)
        if resp.status_code == 200:
            with open(map_pgm, 'wb') as f:
                f.write(resp.content)
            print(f"  ✓ PGM 다운로드 완료: {map_pgm}")
        else:
            print(f"  ⚠️ PGM 다운로드 실패: HTTP {resp.status_code}")
            
    except requests.exceptions.ConnectionError:
        print(f"  ⚠️ 서버 연결 실패 (192.168.0.3:5100)")
        if os.path.exists(map_yaml):
            print(f"  ✓ 기존 맵 사용: {map_yaml}")
        else:
            print(f"  ❌ 맵 파일 없음! Nav2가 실패할 수 있음")
    except Exception as e:
        print(f"  ⚠️ 맵 다운로드 오류: {e}")
    
    # 최종 확인
    if os.path.exists(map_yaml) and os.path.exists(map_pgm):
        print("  ✓ 맵 준비 완료!")
    else:
        print("  ⚠️ 맵 파일 없음 - Nav2 시작 시 오류 가능")
    
    print("="*50 + "\n")
    time.sleep(1)
    
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
        
        # ========== 1. 맵 다운로드 (Nav2 전에 먼저!) ==========
        OpaqueFunction(function=download_map_before_nav2),
        
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
        
        # 2. 모드 발행 시작 (LCD에 NAV2 표시)
        mode_publisher,
        
        # 3. Bringup 먼저 (LiDAR, TF, 모터)
        bringup_launch,
        
        # 4. Nav2 전체 스택 (3초 후 - TF 안정화 대기)
        #    pinky_navigation/bringup_launch.xml 사용
        TimerAction(period=3.0, actions=[
            LogInfo(msg='🗺️ Starting Nav2 Stack (pinky_navigation)...'),
            nav2_launch
        ]),
        
        # 5. 센서 + 상태 노드들 (4초 후)
        TimerAction(
            period=4.0,
            actions=[
                status_display_node,
                mqtt_bridge_node,
                camera_stream_node,
            ]
        ),
        
        # 6. Nav2 Goal Node + ArUco Dock Node (8초 후 - Nav2 준비 완료 대기)
        TimerAction(
            period=8.0, 
            actions=[
                LogInfo(msg='🎯 Starting Nav2 Goal Node (MQTT → Nav2)...'),
                nav2_goal_node,
                LogInfo(msg='🎯 Starting ArUco Dock Node (정밀 도킹)...'),
                aruco_dock_node,
            ]
        ),
        
        # 종료 시 모터 정지
        motor_stop_on_exit,
    ])
