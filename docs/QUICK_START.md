# 🚀 빠른 시작 가이드

> 설치, 빌드, 실행, 트러블슈팅

---

## 📋 목차

1. [의존성 설치](#의존성-설치)
2. [빌드](#빌드)
3. [실행 방법](#실행-방법)
4. [RViz 시각화](#rviz-시각화)
5. [트러블슈팅](#트러블슈팅)

---

## 의존성 설치

### ROS2 패키지

```bash
sudo apt install ros-jazzy-nav2-bringup \
                 ros-jazzy-slam-toolbox \
                 ros-jazzy-rplidar-ros \
                 ros-jazzy-tf2-ros \
                 ros-jazzy-robot-state-publisher
```

### Python 패키지

```bash
pip install paho-mqtt opencv-python numpy flask picamera2
```

### LED 제어 (rpi_ws281x)

```bash
cd ~/pinky_devices/rpi_ws281x
mkdir build && cd build
cmake ..
make
sudo make install

cd ~/pinky_devices/rpi-ws281x-python/library
sudo pip install .
```

---

## 빌드

```bash
cd ~/ros2_ws

# 의존성 확인 (선택)
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --packages-select slam_mqtt_project

# 환경 적용
source install/setup.bash
```

### 빌드 오류 시

```bash
# 클린 빌드
rm -rf build/slam_mqtt_project install/slam_mqtt_project
colcon build --packages-select slam_mqtt_project
```

---

## 실행 방법

### SLAM 모드 (맵 생성)

```bash
# 터미널 1: 전체 SLAM 시스템
ros2 launch slam_mqtt_project slam_exploration.launch.py

# 터미널 2: RViz (원격 PC에서)
ros2 launch pinky_navigation slam_view.launch.xml
```

### Nav2 모드 (네비게이션)

```bash
# 터미널 1: 전체 Nav2 시스템
ros2 launch slam_mqtt_project nav2_mode.launch.py map:=/path/to/map.yaml

# 터미널 2: RViz (원격 PC에서)
ros2 launch pinky_navigation nav2_view.launch.xml
```

### HOME 설정 (선택적)

```bash
# ArUco 마커로 HOME 위치 설정
ros2 launch slam_mqtt_project set_home.launch.py
```

### 개별 노드 실행 (테스트용)

```bash
# MQTT 브릿지
ros2 run slam_mqtt_project mqtt_bridge

# 카메라 스트리밍 (Nav2)
ros2 run slam_mqtt_project nav2_camera_stream

# 자율 주행 (SLAM)
ros2 run slam_mqtt_project slam_auto_drive

# 상태 표시 (LED + LCD)
sudo -E ros2 run slam_mqtt_project status_display
```

---

## RViz 시각화

### 원격 PC 설정

```bash
# ROS2 네트워크 설정 (원격 PC에서)
export ROS_DOMAIN_ID=0  # 로봇과 동일한 ID

# 멀티캐스트 허용 (방화벽)
sudo ufw allow from 192.168.0.0/24
```

### RViz 디스플레이 설정

| Display | Topic | 설명 |
|---------|-------|------|
| **Map** | `/map` | SLAM/정적 맵 |
| **LaserScan** | `/scan` | LiDAR 스캔 |
| **TF** | (all) | 좌표 변환 |
| **RobotModel** | (URDF) | 로봇 모델 |
| **Path** | `/plan` | 계획된 경로 |
| **ParticleCloud** | `/particle_cloud` | AMCL 파티클 |

### RViz 설정 저장

```bash
# RViz에서 File → Save Config As
# 권장 경로: ~/ros2_ws/src/slam_mqtt_project/rviz/slam_view.rviz
```

---

## 트러블슈팅

### 🔴 LiDAR 오류

**증상**: `/scan` 토픽 없음

```bash
# 해결책 1: 권한 확인
sudo chmod 666 /dev/ttyUSB0
ls -la /dev/ttyUSB*

# 해결책 2: udev 규칙
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/rplidar.rules
sudo udevadm control --reload-rules
```

### 🔴 MQTT 연결 실패

**증상**: `Connection refused`

```bash
# Mosquitto 상태 확인
systemctl status mosquitto

# 방화벽 확인
sudo ufw allow 1883

# 브로커 주소 확인 (코드 내)
grep -r "mqtt_host" ~/ros2_ws/src/slam_mqtt_project/
```

### 🔴 TF 오류

**증상**: `Could not transform from base_link to odom`

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# Static TF 확인
ros2 topic echo /tf_static

# 해결책: 필요한 TF 추가
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link laser_frame
```

### 🔴 Nav2 Goal 실패

**증상**: `Goal was rejected`

```bash
# Lifecycle 상태 확인
ros2 lifecycle list /bt_navigator
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server

# 활성화
ros2 lifecycle set /bt_navigator activate

# 맵이 로드되었는지 확인
ros2 topic echo /map --once
```

### 🔴 AMCL 위치 추정 실패

**증상**: 파티클이 수렴하지 않음

```bash
# 초기 위치 재설정 (RViz: 2D Pose Estimate)

# 파티클 분산 확인
ros2 topic echo /amcl_pose

# 해결책: 파티클 수 증가 (nav2_params.yaml)
# max_particles: 3000
```

### 🔴 카메라 오류

**증상**: `Camera in use` 또는 검은 화면

```bash
# 카메라 프로세스 확인
ps aux | grep libcamera

# 강제 종료
sudo killall libcamera-vid

# 카메라 테스트
libcamera-hello --list-cameras
```

### 🔴 LED 제어 실패

**증상**: `Can't open /dev/mem`

```bash
# root 권한으로 실행
sudo -E ros2 run slam_mqtt_project status_display

# 또는 그룹 권한 추가
sudo usermod -a -G gpio $USER
sudo reboot
```

---

## 🧪 테스트 명령어

### MQTT 테스트

```bash
# Subscribe
mosquitto_sub -h 192.168.0.3 -t "slam_mode" -v

# Publish (PLC → Robot)
mosquitto_pub -h 192.168.0.3 -t "plc/location" \
  -m '{"location_id": "A01"}'
```

### ROS2 토픽 테스트

```bash
# 토픽 목록
ros2 topic list

# 토픽 모니터링
ros2 topic echo /robot_mode
ros2 topic echo /nav2/status

# Goal 전송 (테스트)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}}}}"
```

### 서비스 테스트

```bash
# 맵 저장
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: '/map', map_url: '/tmp/test_map'}"

# AMCL 초기화
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

---

## 📊 성능 모니터링

```bash
# CPU/메모리
htop

# ROS2 노드 상태
ros2 node list
ros2 node info /auto_drive_node

# 토픽 주파수
ros2 topic hz /scan
ros2 topic hz /odom

# TF 지연
ros2 run tf2_ros tf2_monitor base_link map
```

---

## 🔗 유용한 링크

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
