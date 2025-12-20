# 🚀 새 기기 설치 가이드

## 1. ROS2 Jazzy 설치
```bash
# ROS2 Jazzy 설치 (Ubuntu 24.04)
sudo apt update && sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
```

## 2. 프로젝트 Clone
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ky51301130-jpg/slam_mqtt_project.git
```

## 3. Python 패키지 설치
```bash
cd ~/ros2_ws/src/slam_mqtt_project
pip install -r requirements.txt
```

## 4. 시스템 설정 파일 복사
```bash
# CycloneDDS 설정
cp config/system/cyclonedds.xml ~/cyclonedds.xml

# IP 주소 수정 (필요시)
nano ~/cyclonedds.xml
# <Peer address="YOUR_PC_IP"/> 로 변경

# Jazzy 설정 스크립트
cp config/system/jazzy_setting.sh ~/.jazzy_setting.sh

# .bashrc에 추가
cat config/system/bashrc_append.txt >> ~/.bashrc
source ~/.bashrc
```

## 5. 카메라 캘리브레이션
```bash
# 캘리브레이션 파일 복사 (같은 카메라 사용 시)
mkdir -p ~/pinky_test
cp config/camera_calibration.npz ~/pinky_test/

# 새 카메라면 캘리브레이션 다시 실행
# python3 -m slam_mqtt_project.aruco_calibration
```

## 6. ROS2 빌드
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 7. 실행
```bash
# SLAM 탐색 모드
ros2 launch slam_mqtt_project slam_exploration.launch.py

# Nav2 네비게이션 모드
ros2 launch slam_mqtt_project nav2_mode.launch.py
```

---

## 📋 추가 설치 (Raspberry Pi용)

### WS281x LED (옵션)
```bash
cd ~/pinky_devices
git clone https://github.com/jgarff/rpi_ws281x.git
cd rpi_ws281x
mkdir build && cd build
cmake .. && make
sudo make install

cd ../..
git clone https://github.com/rpi-ws281x/rpi-ws281x-python.git
cd rpi-ws281x-python/library
pip install .
```

### pinkylib (LCD, 센서)
```bash
# pinkylib가 별도 저장소에 있으면:
# git clone https://github.com/YOUR_USERNAME/pinkylib.git ~/pinkylib
# cd ~/pinkylib && pip install -e .
```

---

## ⚙️ CycloneDDS IP 설정
`~/cyclonedds.xml` 파일에서 Peer 주소를 자신의 네트워크에 맞게 수정:
```xml
<Peers>
    <Peer address="192.168.0.3"/>  <!-- PC IP -->
    <Peer address="192.168.0.5"/>  <!-- Robot IP -->
</Peers>
```
