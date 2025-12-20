from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_mqtt_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.xml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinky',
    maintainer_email='pinky@todo.todo',
    description='SLAM MQTT Project with motor and ultrasonic control',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ===== SLAM 모드 전용 (3개) =====
            'slam_auto_drive = slam_mqtt_project.auto_drive_node:main',
            'slam_map_saver = slam_mqtt_project.map_saver_node:main',
            'slam_collision_photo = slam_mqtt_project.collision_photo_node:main',
            
            # ===== NAV2 모드 전용 (3개) =====
            'nav2_goal = slam_mqtt_project.nav2_goal_node:main',
            'nav2_camera_stream = slam_mqtt_project.camera_stream_node:main',
            'nav2_aruco_dock = slam_mqtt_project.aruco_dock_node:main',
            
            # ===== 공통 (3개) =====
            'mqtt_bridge = slam_mqtt_project.mqtt_bridge_node:main',
            'status_display = slam_mqtt_project.status_display_node:main',
            'robot_map_loader = slam_mqtt_project.robot_map_loader:main',
            
            # ===== 설정 도구 (3개) - 필요시만 실행 =====
            'set_home_by_aruco = slam_mqtt_project.set_home_by_aruco:main',
            'set_home_pose = slam_mqtt_project.set_home_pose:main',
            'aruco_calibration = slam_mqtt_project.aruco_calibration:main',
        ],
    },
)
