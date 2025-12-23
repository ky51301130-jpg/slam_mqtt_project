# 🤖 SLAM MQTT Project - 완전 가이드

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.12+-yellow)
![License](https://img.shields.io/badge/License-MIT-green)
![Based on](https://img.shields.io/badge/Based_on-Pinky_Pro-ff69b4)

> Raspberry Pi 기반 자율주행 로봇의 SLAM/Nav2 통합 시스템 (로봇 측 코드)

> 🖥️ 서버 측 코드는 [slam_mqtt_server](https://github.com/ky51301130-jpg/slam_mqtt_server) 저장소를 참조하세요.

> 🤖 이 프로젝트는 [pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro) ROS2 패키지를 기반으로 개발되었습니다.

## 📋 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [시스템 아키텍처](#시스템-아키텍처)
3. [빠른 시작](#빠른-시작)
4. [노드별 상세 설명 + 핵심 코드](#노드별-상세-설명--핵심-코드)
5. [AMCL 상세 설명](#amcl-상세-설명)
6. [Behavior Tree 상세 설명](#behavior-tree-상세-설명)
7. [MQTT 통신 구조](#mqtt-통신-구조)
8. [RViz 시각화](#rviz-시각화)
9. [트러블슈팅](#트러블슈팅)

---

## 프로젝트 개요

### 🎯 목적

미지의 환경을 자율 탐색하여 맵을 생성하고, 생성된 맵을 기반으로 목표점 네비게이션을 수행하는 ROS2 패키지입니다.

### 🔧 주요 기능

| 모드 | 기능 | 카메라 | 포트 |
|------|------|--------|------|
| **SLAM** | 자율 탐색 + 맵 생성 | 충돌 사진 + AI 분석 | 5000 (사진), 5200 (스트리밍) |
| **Nav2** | 맵 기반 목표점 이동 | 1fps 스트리밍 | 5200 |

### 🖥️ 하드웨어 구성

```
┌─────────────────────────────────────────┐
│           Raspberry Pi 5                │
│         (192.168.0.5)                   │
├─────────────────────────────────────────┤
│ • LiDAR (SLAM/장애물 감지)              │
│ • 초음파 센서 (근거리 감지)             │
│ • DC 모터 + 드라이버 (차동 구동)        │
│ • Picamera2 (AI 분석용)                 │
│ • LCD 디스플레이 (상태 표시)            │
│ • WS281x LED (8개, 진행률/상태 표시)    │
│ • 버저 (알림음)                         │
└─────────────────────────────────────────┘
         │
         │ WiFi (192.168.0.x)
         ▼
┌─────────────────────────────────────────┐
│          MCU (ESP32/Arduino)            │
├─────────────────────────────────────────┤
│ • Lux 센서 (조도 측정)                  │
│ • 기타 환경 센서                        │
│      │                                  │
│      │ MQTT (mcu/sensors)               │
│      ▼                                  │
├─────────────────────────────────────────┤
│          서버 (192.168.0.3)             │
├─────────────────────────────────────────┤
│ • MQTT Broker (:1883)                   │
│ • Map Server Flask (:5000, :5200)       │
│ • YOLO 분석 서버                        │
│ • PLC 시스템                            │
└─────────────────────────────────────────┘
```

---

## 시스템 아키텍처

### 📊 노드 구성도

```
┌─────────────────────────────────────────────────────────────────┐
│                    SLAM 모드 (slam_exploration.launch.py)       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐           │
│  │ auto_drive  │──▶│   /scan     │──▶│ map_saver   │           │
│  │   _node     │   │  /odom      │   │   _node     │           │
│  └─────────────┘   └─────────────┘   └─────────────┘           │
│        │                                    │                   │
│        ▼                                    ▼                   │
│  ┌─────────────┐                     ┌─────────────┐           │
│  │  /cmd_vel   │                     │ 서버 업로드  │           │
│  └─────────────┘                     └─────────────┘           │
│                                                                 │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐           │
│  │   status    │   │ mqtt_bridge │   │ collision   │           │
│  │ _display    │◀──│   _node     │◀──│ _photo_node │           │
│  │ (LED+LCD)   │   │             │   │             │           │
│  └─────────────┘   └─────────────┘   └─────────────┘           │
│        │                  ▲                                     │
│        │ Lux 기반 LED     │ MCU (mcu/sensors)                   │
│        ▼                  │                                     │
│  ┌─────────────┐          │                                     │
│  │  WS281x LED │          │                                     │
│  │  + LCD 표시 │          │                                     │
│  └─────────────┘          │                                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    Nav2 모드 (nav2_mode.launch.py)              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Nav2 bringup_launch.py                      │   │
│  │  ┌────────┐ ┌─────────┐ ┌──────────┐ ┌────────────┐     │   │
│  │  │  AMCL  │ │ Planner │ │Controller│ │BT Navigator│     │   │
│  │  └────────┘ └─────────┘ └──────────┘ └────────────┘     │   │
│  └─────────────────────────────────────────────────────────┘   │
│        ▲                                                        │
│        │ navigate_to_pose                                       │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐           │
│  │ nav2_goal   │◀──│ MQTT/PLC    │   │ camera      │           │
│  │   _node     │   │   명령      │   │ _stream_node│           │
│  └─────────────┘   └─────────────┘   └─────────────┘           │
│                                                                 │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐           │
│  │   status    │◀──│ mqtt_bridge │   │ aruco_dock  │           │
│  │  _display   │   │   _node     │   │   _node     │           │
│  │ (LED+LCD)   │   └─────────────┘   └─────────────┘           │
│  └─────────────┘          ▲                  │                  │
│                           │                  │ 정밀 도킹        │
│                      MCU (Lux)               ▼                  │
│                                        ┌─────────────┐          │
│                                        │  ArUco 마커 │          │
│                                        │  위치 측정  │          │
│                                        └─────────────┘          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 빠른 시작

### 📦 의존성 설치

```bash
# ROS2 Jazzy + Nav2
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Python 패키지
pip install paho-mqtt requests flask pillow pyyaml

# 하드웨어 라이브러리 (라즈베리파이)
pip install pinkylib rpi-ws281x picamera2
```

### 🔨 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select slam_mqtt_project --symlink-install
source install/setup.bash
```

### 🚀 실행

```bash
# ===== 로봇 (192.168.0.5) =====

# SLAM 탐색 모드 (맵 생성)
ros2 launch slam_mqtt_project slam_exploration.launch.py

# Nav2 네비게이션 모드 (맵 기반 이동)
ros2 launch slam_mqtt_project nav2_mode.launch.py

# ===== 서버/PC (192.168.0.3) - RViz =====

# SLAM 시각화
ros2 launch pinky_navigation slam_view.launch.xml

# Nav2 시각화 (Goal 설정 가능)
ros2 launch pinky_navigation nav2_view.launch.xml
```

---

## 노드별 상세 설명 + 핵심 코드

### 1. 🚗 auto_drive_node.py (SLAM 자율 탐색)

**역할**: 미지의 환경을 자동으로 탐색하며 SLAM Toolbox가 맵을 생성하도록 주행

**핵심 알고리즘**:

```python
# ===== 1. LiDAR 데이터 처리 =====
def scan_callback(self, msg):
    ranges = np.array(msg.ranges)
    valid = np.where((ranges > 0.05) & (ranges < 10.0), ranges, 10.0)
    
    # 전방 거리 (345°~15°)
    self.front_dist = get_min(345, 15)
    
    # 대각선 (장애물 사전 감지)
    self.front_left = get_min(30, 60)   # 좌측 대각선
    self.front_right = get_min(300, 330)  # 우측 대각선
    
    # 탐색 방향 결정용 (열린 공간 감지)
    self.left_open = get_mean(60, 120)   # 좌측 평균 거리
    self.right_open = get_mean(240, 300)  # 우측 평균 거리

# ===== 2. 탐색 제어 로직 =====
def do_explore(self):
    front_clear = self.get_front_clear()  # 초음파 + LiDAR 융합
    
    # 정면 막힘 → 회전
    if front_clear < FRONT_STOP:
        # 더 열린 쪽으로 회전 방향 결정
        left_score = self.left_open + self.front_left
        right_score = self.right_open + self.front_right
        self.turn_direction = 1 if left_score >= right_score else -1
        self.start_turn()
        return
    
    # 통로 모드: 좌우 벽 사이에서 중앙 유지
    if self.in_corridor:
        center_error = self.left_dist - self.right_dist
        twist.angular.z = center_error * 1.2  # P 제어
        twist.linear.x = SPEED_CORRIDOR
        return
    
    # 일반 주행
    twist.linear.x = SPEED_FAST if front_clear > FRONT_SLOW else SPEED_EXPLORE
```

**학습 포인트**:
- `get_min()`: 각도 범위의 최소값 → 장애물 감지
- `get_mean()`: 각도 범위의 평균 → 열린 공간 감지
- P 제어: `error * gain` 형태의 비례 제어

---

### 2. 💾 map_saver_node.py (맵 저장 + 서버 업로드)

**역할**: 60초마다 맵 저장, 8개 완료 시 서버에 알림

**핵심 코드**:

```python
# ===== OccupancyGrid → PGM 이미지 변환 =====
def save_map(self):
    map_data = np.array(self.latest_map.data, dtype=np.int8)
    map_data = map_data.reshape((info.height, info.width))
    
    # ROS2 맵 값 → 그레이스케일 변환
    # -1: 미탐색(205), 0: 자유공간(254), 100: 장애물(0)
    img = np.full_like(map_data, 205, dtype=np.uint8)
    img[map_data == 0] = 254      # 자유 공간 = 흰색
    img[map_data == 100] = 0       # 장애물 = 검정
    
    # 중간값 (확률) 처리
    mask = (map_data > 0) & (map_data < 100)
    img[mask] = (254 - (map_data[mask] * 254 / 100)).astype(np.uint8)
    
    img = np.flipud(img)  # 좌표계 뒤집기
    cv2.imwrite(pgm_path, img)

# ===== YAML 파일 생성 =====
yaml_content = f"""image: {base}.pgm
mode: trinary
resolution: {info.resolution}
origin: [{origin.x}, {origin.y}, {yaw}]
initial_pose:
  x: {self.initial_pose['x']}
  y: {self.initial_pose['y']}
  yaw: {self.initial_pose['yaw']}
"""
```

**학습 포인트**:
- `OccupancyGrid.data`: 1차원 배열, reshape 필요
- 값 의미: -1(미탐색), 0(자유), 1-99(확률), 100(장애물)
- `np.flipud()`: ROS 좌표계 → 이미지 좌표계 변환

---

### 3. 🎯 nav2_goal_node.py (MQTT → Nav2 Goal)

**역할**: MQTT/PLC 명령을 Nav2 Action으로 변환

**핵심 코드**:

```python
# ===== Nav2 Action Client 설정 =====
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

# ===== Goal 전송 =====
def send_goal(self, x: float, y: float, yaw: float = 0.0):
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    
    # Yaw → Quaternion 변환
    goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    # 비동기 전송 + 콜백
    future = self._action_client.send_goal_async(
        goal_msg, feedback_callback=self.feedback_cb
    )
    future.add_done_callback(self.goal_response_cb)

# ===== AMCL 초기 위치 설정 =====
def set_initial_pose(self, x, y, yaw):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    # Covariance (위치 불확실성)
    msg.pose.covariance[0] = 0.25   # x 분산
    msg.pose.covariance[7] = 0.25   # y 분산
    msg.pose.covariance[35] = 0.06  # yaw 분산
    
    self.initial_pose_pub.publish(msg)
```

**학습 포인트**:
- `ActionClient`: 장시간 작업용 비동기 통신 (Goal → Feedback → Result)
- Quaternion: `(x,y,z,w)` = `(0, 0, sin(θ/2), cos(θ/2))` for 2D rotation
- Covariance: 대각선 요소만 사용 [0]=x, [7]=y, [35]=yaw

---

### 4. 📷 camera_stream_node.py (Nav2용 스트리밍)

**역할**: 1fps로 이미지 스트리밍, YOLO 서버에서 분석

**핵심 코드**:

```python
# ===== Picamera2 초기화 =====
from picamera2 import Picamera2

self.camera = Picamera2()
config = self.camera.create_preview_configuration(
    main={"format": "RGB888", "size": (320, 240)}
)
self.camera.configure(config)
self.camera.start()

# ===== 캡처 + JPEG 인코딩 =====
def _capture_loop(self):
    while self.running:
        frame = self.camera.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)  # 180도 회전
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        _, jpeg = cv2.imencode('.jpg', frame_bgr, 
            [cv2.IMWRITE_JPEG_QUALITY, 50])  # 50% 품질 (대역폭 절약)
        
        self.current_jpeg = jpeg.tobytes()
        time.sleep(1.0 / self.fps)  # 1fps

# ===== Flask HTTP 서버 =====
@app.route('/image.jpg')
def get_image():
    return Response(node.current_jpeg, mimetype='image/jpeg')
```

**학습 포인트**:
- `cv2.IMWRITE_JPEG_QUALITY`: 품질 vs 파일 크기 트레이드오프
- Flask: YOLO 서버가 HTTP로 이미지 가져감 (폴링 방식)
- `cv2.ROTATE_180`: 카메라 마운트 방향 보정

---

### 5. 🚨 collision_photo_node.py (SLAM용 충돌 사진)

**역할**: 충돌 감지 시에만 고해상도 사진 촬영

**핵심 코드**:

```python
# ===== 초음파 콜백 =====
def ultrasonic_cb(self, msg: Float32):
    distance = msg.data
    
    if distance < COLLISION_DISTANCE:  # 0.25m 미만
        self.capture_collision_photo(distance)

# ===== 쿨다운 방지 =====
def capture_collision_photo(self, distance):
    now = time.time()
    
    # 연속 촬영 방지 (3초 쿨다운)
    if now - self.last_photo_time < COOLDOWN_TIME:
        return
    
    # 고해상도 촬영 (640x480)
    frame = self.camera.capture_array()
    filename = f"collision_{timestamp}_{distance:.2f}m.jpg"
    cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    
    self.last_photo_time = now
    
    # MQTT로 서버에 알림
    self.mqtt.publish("collision/photo_ready", json.dumps({
        "filename": filename,
        "url": f"http://192.168.0.5:5001/photos/{filename}"
    }))
```

**학습 포인트**:
- 쿨다운: 동일 장애물에 대한 중복 촬영 방지
- 이벤트 기반: 상시 스트리밍 대비 전력/대역폭 절약

---

### 6. 📡 mqtt_bridge_node.py (ROS2 ↔ MQTT)

**역할**: 외부 시스템(PLC, 서버)과 ROS2 연결

**핵심 코드**:

```python
# ===== MQTT → ROS2 =====
def on_message(self, client, userdata, msg):
    payload = msg.payload.decode("utf-8")
    ros_msg = String(data=payload)
    
    if msg.topic.startswith("mcu/"):
        self.pub_mcu.publish(ros_msg)
    elif msg.topic.startswith("plc/"):
        self.pub_plc.publish(ros_msg)

# ===== ROS2 → MQTT =====
def cycle_cb(self, msg):
    self.mqtt.publish("ros/map_cycle_complete", msg.data, qos=1)

# ===== 재연결 로직 =====
def start_mqtt(self):
    while rclpy.ok():
        try:
            self.mqtt.connect(MQTT_HOST, MQTT_PORT, 60)
            self.mqtt.loop_forever()
        except Exception as e:
            self.get_logger().warn(f"MQTT fail: {e}, retry 10s")
            time.sleep(10)
```

**학습 포인트**:
- `loop_forever()`: 블로킹 루프, 별도 스레드 필요
- QoS 1: 최소 1회 전달 보장 (중요 메시지용)

---

### 7. 🔔 status_display_node.py (LED + LCD 통합 표시)

**역할**: WS281x LED + LCD를 통합 관리하여 로봇 상태 시각화
- 기존 `led_controller_node.py` + `lcd_status_node.py` + `ultrasonic_node.py` 통합
- 저전력 최적화: 변화 있을 때만 업데이트

**데이터 흐름**:

```
MCU (ESP32)                  MQTT Broker              ROS2
    │                            │                      │
    │  {"Lux": 150.5}            │                      │
    ├──────mcu/sensors──────────▶│                      │
    │                            │                      │
    │                            │  /mqtt/mcu_sensors   │
    │                            ├─────────────────────▶│ mqtt_bridge_node
    │                            │                      │       │
    │                            │                      │       ▼
    │                            │                      │ status_display_node
    │                            │                      │   (LED + LCD 통합)
```

**핵심 코드**:

```python
# ===== LED + LCD 통합 노드 =====
class StatusDisplayNode(Node):
    def __init__(self):
        # LED 초기화 (WS281x)
        self.leds = LED()
        self.leds.__enter__()
        
        # LCD 초기화 (SPI)
        self.lcd = LCD()
        
        # 상태 변수
        self.robot_mode = "IDLE"      # SLAM/NAV2/IDLE
        self.current_lux = 0           # MCU에서 수신
        self.map_save_count = 0        # 맵 저장 진행률
        self.battery_percent = 50.0    # 배터리 잔량
        
        # 구독: 모드, Lux, 맵 저장, 배터리
        self.create_subscription(String, ROS.ROBOT_MODE, self.mode_cb, 1)
        self.create_subscription(String, ROS.MQTT_MCU_SENSORS, self.sensor_cb, 1)
        self.create_subscription(Int32, ROS.MAP_SAVER_SAVED, self.map_saved_cb, 1)
        self.create_subscription(Float32, ROS.BATTERY_PRESENT, self.battery_cb, 1)

# ===== MCU Lux 센서 처리 =====
LUX_THRESHOLD = 100  # 밝음/어두움 기준

def sensor_cb(self, msg):
    data = json.loads(msg.data)  # {"Lux": 150.5}
    if "Lux" in data:
        self.current_lux = float(data["Lux"])
        # Lux 기반 LED 색상 결정
        if self.current_lux >= LUX_THRESHOLD:
            self.current_led_mode = "bright"   # GREEN
        else:
            self.current_led_mode = "dark"     # BLUE

# ===== 맵 저장 진행률 LED =====
def _set_led_progress(self, count, total=8):
    """맵 저장 진행률: 저장된 만큼 ORANGE, 나머지 RED"""
    for i in range(NUM_LEDS):
        color = ORANGE if i < count else RED
        self.leds.set_pixel(i, color)
    self.leds.show()

# ===== 상태별 LED 색상 =====
colors = {
    "driving": RED,      # SLAM 주행 중
    "map_saving": None,  # 진행률 표시 (_set_led_progress)
    "bright": GREEN,     # Lux >= 100 (밝은 환경)
    "dark": BLUE,        # Lux < 100 (어두운 환경)
    "idle": OFF          # 대기
}

# ===== LCD 배터리/모드 표시 =====
def update_lcd(self):
    img = Image.new('RGB', (320, 240), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    
    # 모드 표시 (SLAM: 파랑, NAV2: 보라)
    mode_color = MODE_COLORS.get(self.robot_mode)
    draw.rectangle([(0, 0), (320, 50)], fill=mode_color)
    draw.text((160, 25), self.robot_mode, font=self.font_large, anchor="mm")
    
    # 배터리 바
    bar_width = int(280 * self.battery_percent / 100)
    draw.rectangle([(20, 80), (300, 130)], outline=(100, 100, 100))
    draw.rectangle([(22, 82), (22 + bar_width, 128)], fill=self._get_battery_color())
    
    self.lcd.img_show(img)
```

**학습 포인트**:
- MCU → MQTT → mqtt_bridge → status_display 데이터 흐름
- Lux 센서: 환경 밝기에 따른 LED 색상 변경
- 모드별 동작: SLAM(진행률), Nav2(Lux 기반)
- 통합 노드: LED + LCD + 배터리를 한 노드에서 관리 (리소스 절약)

---

### 8. 🎯 aruco_dock_node.py (ArUco 정밀 도킹) - NEW!

**역할**: Nav2 도착 후 ArUco 마커로 정밀 위치 조정

**흐름**:
```
1. Nav2로 PORT_A 근처 도착
2. nav2_goal_node가 dock_enable = True 발행
3. aruco_dock_node가 카메라로 ArUco 마커 감지
4. 마커 위치/각도 기반 정밀 접근
5. 도킹 완료 시 위치 저장 + 알림
```

**핵심 코드**:

```python
# ===== ArUco 마커 ID → 포트 매핑 =====
MARKER_PORT_MAP = {
    0: "HOME",       # ID 0 = HOME (충전/기준점)
    1: "PORT_A",     # ID 1 = 작업위치 A
    2: "PORT_B",     # ID 2 = 작업위치 B
    # ...
}

# ===== 도킹 제어 루프 =====
def dock_control_loop(self):
    if not self.docking_enabled:
        return
    
    # ArUco 마커 감지
    corners, ids, _ = self.aruco_detector.detectMarkers(frame)
    
    if self.target_marker_id in ids:
        # 마커 위치 계산 (x, y, z, yaw)
        rvec, tvec = cv2.solvePnP(...)
        distance = np.linalg.norm(tvec)
        
        # 정밀 접근
        if distance > self.DOCK_DISTANCE:
            # 전진 + 각도 보정
            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = -center_error * self.ANGULAR_SPEED
        else:
            # 도킹 완료!
            self.save_port_position()
            self.publish_arrival()
```
        if i < self.map_save_count:
---

### 9. 📝 topics.py (토픽 중앙 관리) - NEW!

**역할**: 모든 ROS2/MQTT 토픽을 한 파일에서 관리

**사용법**:

```python
from slam_mqtt_project.topics import ROS, MQTT, NET, ARUCO

# ROS2 토픽 사용
self.create_subscription(String, ROS.ROBOT_MODE, self.cb, 10)
self.create_publisher(Twist, ROS.CMD_VEL, 10)

# MQTT 토픽 사용
self.mqtt.subscribe(MQTT.SUB_MCU_SENSORS)  # "/mcu/sensors"
self.mqtt.publish(MQTT.PUB_NAV_STATUS, payload)

# 네트워크 설정
print(f"Server: {NET.SERVER_IP}:{NET.MQTT_PORT}")
print(f"Map Upload: {NET.map_upload_url()}")

# ArUco 마커 설정
marker_map = ARUCO.PORT_MAP  # {0: "HOME", 1: "PORT_A", ...}
```

**장점**:
- 토픽 수정 시 한 파일만 수정
- 타이핑/자동완성 지원
- IP/포트 중앙 관리

---

## AMCL 상세 설명

### 🎯 AMCL이란? (Adaptive Monte Carlo Localization)

**"로봇이 맵 안에서 자신이 어디 있는지 알아내는 알고리즘"**입니다.

SLAM 모드에서는 맵을 생성하면서 위치를 추정하지만, Nav2 모드에서는 **이미 만들어진 맵**을 사용합니다. 이때 AMCL이 LiDAR 데이터와 맵을 비교하여 로봇의 현재 위치를 추정합니다.

### 📊 AMCL 동작 원리

```
┌─────────────────────────────────────────────────────────────┐
│                    AMCL 동작 원리                            │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   1. 파티클 생성 (초기화)                                    │
│   ┌────────────────────────┐                                │
│   │    * * * * * * *       │  ← 수백~수천 개의 "파티클"      │
│   │   * * * 🤖 * * *       │    (가능한 위치 후보들)         │
│   │    * * * * * * *       │                                │
│   └────────────────────────┘                                │
│                                                              │
│   2. LiDAR 매칭 (센서 업데이트)                              │
│   ┌────────────────────────┐                                │
│   │         🧱              │                                │
│   │    * *  🤖  * *        │  ← 실제 LiDAR 스캔과           │
│   │         🧱              │    맵이 일치하는 파티클에      │
│   └────────────────────────┘    높은 가중치 부여             │
│                                                              │
│   3. 리샘플링 (수렴)                                         │
│   ┌────────────────────────┐                                │
│   │         🧱              │                                │
│   │       * 🤖 *           │  ← 가중치 높은 파티클만 생존    │
│   │         🧱              │    → 위치가 정확해짐!          │
│   └────────────────────────┘                                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 🔧 핵심 파라미터 설명 (`nav2_params.yaml`)

```yaml
amcl:
  ros__parameters:
    # ===== 파티클 수 (정확도 vs 성능) =====
    min_particles: 500      # 최소 파티클 수
    max_particles: 2000     # 최대 파티클 수
    # 👆 라즈베리파이에서는 500~2000 권장
    #    너무 높으면 CPU 부하 증가
    #    너무 낮으면 위치 추정 부정확
    
    # ===== 업데이트 조건 =====
    update_min_d: 0.2       # 0.2m 이동해야 업데이트
    update_min_a: 0.5       # 0.5rad(~29°) 회전해야 업데이트
    # 👆 너무 작으면 CPU 과부하
    #    너무 크면 위치 추정 지연
    
    # ===== 초기 위치 =====
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
    # 👆 맵 저장 시 저장된 initial_pose와 일치해야 함!
    #    map_saver_node가 YAML에 저장한 값 사용
    
    # ===== 센서 모델 =====
    laser_model_type: "likelihood_field"  # 권장 (빠르고 정확)
    # 다른 옵션: "beam" (더 정확하지만 느림)
    
    laser_max_range: 12.0      # LiDAR 최대 측정 거리 (m)
    laser_min_range: 0.1       # LiDAR 최소 측정 거리 (m)
    max_beams: 180             # 사용할 LiDAR 빔 수 (360개 중)
    # 👆 빔 수가 많을수록 정확하지만 느림
    
    # ===== 오도메트리 모델 =====
    odom_model_type: "diff"    # 차동 구동 로봇
    # 다른 옵션: "omni" (전방향 이동 로봇)
    
    # 오도메트리 노이즈 파라미터
    alpha1: 0.2   # 회전→회전 노이즈
    alpha2: 0.2   # 이동→회전 노이즈
    alpha3: 0.2   # 이동→이동 노이즈
    alpha4: 0.2   # 회전→이동 노이즈
    # 👆 값이 클수록 오도메트리를 덜 신뢰
    #    바퀴 미끄러짐이 심하면 값을 높임
    
    # ===== 복구 동작 =====
    recovery_alpha_slow: 0.001  # 느린 복구율
    recovery_alpha_fast: 0.1    # 빠른 복구율
    # 👆 위치를 잃었을 때 파티클 재분배 속도
```

### 💻 우리 프로젝트에서 AMCL 사용 코드

```python
# nav2_goal_node.py에서 초기 위치 설정

def set_initial_pose(self, x, y, yaw):
    """
    AMCL에게 "로봇이 여기 있어"라고 알려줌
    
    사용 시점:
    1. Nav2 모드 시작 시
    2. 맵에서 로드한 initial_pose 적용
    3. MQTT로 외부에서 위치 리셋 요청 시
    """
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'           # 맵 좌표계 기준
    msg.header.stamp = self.get_clock().now().to_msg()
    
    # 위치 설정
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    
    # Yaw(회전각) → Quaternion 변환
    # 2D 로봇이므로 z, w만 사용 (x, y는 0)
    # 공식: z = sin(θ/2), w = cos(θ/2)
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    # ===== Covariance (불확실성) =====
    # 6x6 행렬을 1차원 배열로 표현 (36개 요소)
    # 대각선 요소만 중요: [0]=x, [7]=y, [35]=yaw
    msg.pose.covariance[0] = 0.25   # x 분산 (0.5m × 0.5m)
    msg.pose.covariance[7] = 0.25   # y 분산 (0.5m × 0.5m)
    msg.pose.covariance[35] = 0.06  # yaw 분산 (~14° × 14°)
    # 👆 값이 클수록 "위치가 불확실해"라고 AMCL에 전달
    #    AMCL이 파티클을 더 넓게 분포시킴
    
    self.initial_pose_pub.publish(msg)
    self.get_logger().info(f"Initial pose set: x={x}, y={y}, yaw={yaw}")
```

### 📍 RViz에서 AMCL 확인하기

1. **파티클 클라우드 표시**:
   - Add → By topic → `/particle_cloud` → PoseArray
   - 빨간 화살표들이 파티클 (위치 후보)

2. **초기 위치 수동 설정**:
   - 상단 툴바에서 `2D Pose Estimate` 클릭 (단축키: P)
   - 맵에서 로봇 위치를 클릭 + 드래그 (방향 지정)

3. **수렴 확인**:
   - 파티클들이 한 점으로 모이면 위치 추정 완료!
   - 분산되어 있으면 위치 불확실 → 로봇을 조금 움직여봄

### ⚠️ AMCL 관련 주의사항

| 문제 | 원인 | 해결 방법 |
|------|------|-----------|
| 파티클이 수렴 안됨 | 초기 위치가 맞지 않음 | RViz에서 2D Pose Estimate로 재설정 |
| 위치가 갑자기 점프 | 유사한 환경 (긴 복도 등) | LiDAR 특징이 많은 곳으로 이동 |
| CPU 사용량 높음 | 파티클 수 과다 | max_particles 줄이기 |
| 위치 추정 느림 | update_min_d/a 너무 큼 | 값을 줄이기 (0.1, 0.3 등) |

---

## Behavior Tree 상세 설명

### 🌳 Behavior Tree(BT)란?

**"로봇이 목표까지 가는 과정에서 어떤 순서로 무엇을 할지 결정하는 의사결정 나무"**입니다.

기존 상태 머신(State Machine)보다 **모듈화**가 잘 되어 있고, **복잡한 행동**을 쉽게 조합할 수 있습니다.

### 📊 Nav2 기본 Behavior Tree 구조

```
┌─────────────────────────────────────────────────────────────────────────┐
│                   NavigateToPose Behavior Tree                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│                        [Root: NavigateToPose]                            │
│                               │                                          │
│                    ┌──────────┴──────────┐                              │
│                    ▼                     ▼                               │
│             [PipelineSequence]    [RecoveryNode]                        │
│                    │                     │                               │
│         ┌─────────┼─────────┐     ┌─────┴─────┐                         │
│         ▼         ▼         ▼     ▼           ▼                         │
│   [RateController] [ComputePath] [FollowPath] [Spin] [BackUp] [Wait]   │
│         │              │             │                                   │
│         ▼              ▼             ▼                                   │
│   "1Hz로 실행"   "경로 계산"   "경로 추종"                               │
│                                                                          │
│  실행 흐름:                                                              │
│  1. ComputePathToPose: 목표까지 전역 경로 계산 (A* 알고리즘)             │
│  2. FollowPath: 계산된 경로를 따라 이동 (DWB Controller)                │
│  3. (실패 시) Recovery 실행: Spin → BackUp → Wait 순서로 시도           │
│  4. Recovery 후 다시 1번부터 재시도                                      │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 🧩 BT 노드 타입 상세 설명

| 노드 타입 | 기호 | 설명 | 예시 |
|-----------|------|------|------|
| **Sequence** | → | 자식을 순서대로 실행, **하나라도 실패하면 중단** | 경로계산 → 경로추종 |
| **Fallback** | ? | 자식을 순서대로 실행, **하나가 성공하면 중단** | Spin 실패 → BackUp 시도 |
| **Parallel** | ⇉ | 자식을 동시에 실행, N개 성공 시 성공 | 이동 + 장애물 감지 |
| **Decorator** | ◇ | 자식 결과를 변형 | RateController, Retry |
| **Action** | ▢ | 실제 동작 수행 | ComputePath, FollowPath |
| **Condition** | ○ | 조건 확인 (True/False) | GoalReached, IsBatteryLow |

### 🔧 핵심 파라미터 설명 (`nav2_params.yaml`)

```yaml
bt_navigator:
  ros__parameters:
    # ===== 기본 BT 파일 =====
    default_bt_xml_filename: "navigate_to_pose_w_replanning_and_recovery.xml"
    # 👆 Nav2 기본 제공 BT 사용
    # 위치: /opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/
    #
    # 다른 옵션:
    # - navigate_to_pose.xml: 기본 (복구 없음)
    # - navigate_through_poses.xml: 여러 웨이포인트 통과
    # - navigate_to_pose_w_replanning_and_recovery.xml: 재계획 + 복구 (권장)
    
    # ===== Goal 관련 =====
    goal_blackboard_id: "goal"           # Goal 저장 변수명
    goals_blackboard_id: "goals"         # 다중 Goal 저장 변수명
    path_blackboard_id: "path"           # 경로 저장 변수명
    
    # ===== 플러그인 (사용 가능한 BT 노드들) =====
    plugin_lib_names:
      # 경로 계획
      - nav2_compute_path_to_pose_action_bt_node    # 목표점까지 경로 계산
      - nav2_compute_path_through_poses_action_bt_node  # 웨이포인트 경로
      
      # 경로 추종
      - nav2_follow_path_action_bt_node             # 경로 따라가기
      - nav2_smooth_path_action_bt_node             # 경로 부드럽게
      
      # 복구 동작
      - nav2_spin_action_bt_node                    # 제자리 회전
      - nav2_back_up_action_bt_node                 # 후진
      - nav2_wait_action_bt_node                    # 대기
      - nav2_clear_costmap_service_bt_node          # Costmap 초기화
      
      # 조건
      - nav2_goal_reached_condition_bt_node         # 목표 도달 확인
      - nav2_is_stuck_condition_bt_node             # 막힘 확인
      - nav2_is_battery_low_condition_bt_node       # 배터리 확인
      
      # 데코레이터
      - nav2_rate_controller_bt_node                # 실행 주기 제어
      - nav2_distance_controller_bt_node            # 거리 기반 제어
      - nav2_speed_controller_bt_node               # 속도 기반 제어

# ===== 복구 동작 파라미터 =====

# 제자리 회전 (Spin)
spin:
  ros__parameters:
    simulate_ahead_time: 2.0      # 충돌 시뮬레이션 시간
    max_rotational_vel: 1.0       # 최대 회전 속도 (rad/s)
    min_rotational_vel: 0.4       # 최소 회전 속도
    rotational_acc_lim: 3.2       # 회전 가속도 제한
    # 👆 막혔을 때 360° 회전하며 장애물 재스캔

# 후진 (BackUp)
backup:
  ros__parameters:
    simulate_ahead_time: 2.0      # 충돌 시뮬레이션 시간
    # 👆 막혔을 때 뒤로 물러나서 탈출

# 대기 (Wait)
wait:
  ros__parameters:
    plugin: "nav2_behaviors/Wait"
    # 👆 동적 장애물이 지나가길 기다림
```

### 🚗 Planner Server (전역 경로 계획)

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0  # 계획 주기 (Hz)
    
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      # 👆 NavFn: Dijkstra/A* 기반 전역 경로 계획기
      
      tolerance: 0.5              # 목표 허용 오차 (m)
      # 👆 목표점 근처 0.5m 이내면 성공으로 간주
      
      use_astar: true             # A* 알고리즘 사용
      # 👆 true: A* (빠름, 휴리스틱 사용)
      #    false: Dijkstra (느림, 최적 경로 보장)
      
      allow_unknown: true         # 미탐색 영역 통과 허용
      # 👆 true: 회색 영역(미탐색) 통과 가능
      #    false: 흰색 영역(자유 공간)만 통과
```

### 🎮 Controller Server (로컬 경로 추종)

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0    # 제어 주기 (Hz)
    min_x_velocity_threshold: 0.001  # 최소 속도 임계값
    min_theta_velocity_threshold: 0.001
    
    progress_checker_plugins: ["progress_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5   # 이 거리 이상 이동해야 진행 중
      movement_time_allowance: 10.0   # 이 시간 동안 진행 없으면 실패
    
    goal_checker_plugins: ["goal_checker"]
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25         # 위치 허용 오차 (m)
      yaw_goal_tolerance: 0.25        # 방향 허용 오차 (rad, ~14°)
      stateful: True                  # 한번 도달하면 유지
    
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # 👆 DWB: Dynamic Window Approach 기반 로컬 플래너
      
      # 속도 제한 (라즈베리파이 로봇용 보수적 설정)
      max_vel_x: 0.3              # 최대 전진 속도 (m/s)
      min_vel_x: 0.0              # 최소 전진 속도 (후진 불가)
      max_vel_y: 0.0              # 최대 횡방향 속도 (차동구동=0)
      max_vel_theta: 1.0          # 최대 회전 속도 (rad/s)
      min_speed_theta: 0.4        # 최소 회전 속도
      
      # 가속도 제한
      acc_lim_x: 2.5              # 전진 가속도 (m/s²)
      acc_lim_y: 0.0              # 횡방향 가속도 (차동구동=0)
      acc_lim_theta: 3.2          # 회전 가속도 (rad/s²)
      decel_lim_x: -2.5           # 감속도
      decel_lim_theta: -3.2
      
      # 시뮬레이션 파라미터
      sim_time: 1.7               # 궤적 시뮬레이션 시간 (초)
      vx_samples: 20              # 전진 속도 샘플 수
      vy_samples: 1               # 횡방향 속도 샘플 수
      vtheta_samples: 20          # 회전 속도 샘플 수
```

### 💻 우리 프로젝트에서 BT 사용 코드

```python
# nav2_goal_node.py에서 NavigateToPose Action 사용

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class Nav2GoalNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_node')
        
        # NavigateToPose Action Client 생성
        # 이 Action을 호출하면 BT Navigator가 실행됨
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'  # Action 서버 이름
        )
    
    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """
        BT Navigator에게 목표점 전달
        
        BT가 자동으로 수행하는 작업:
        1. ComputePathToPose: A* 알고리즘으로 전역 경로 계산
        2. FollowPath: DWB Controller로 경로 추종
        3. (장애물 만나면) Recovery 실행: Spin → BackUp → Wait
        4. GoalReached: 목표 도달 확인
        """
        # Action 서버 대기
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available!")
            return False
        
        # Goal 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 위치 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # 방향 설정 (Yaw → Quaternion)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 비동기 Goal 전송
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Goal 수락/거절 응답 처리"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2!")
            return
        
        self.get_logger().info("Goal accepted, BT is running...")
        
        # 결과 대기
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """BT 실행 중 피드백 수신"""
        feedback = feedback_msg.feedback
        
        # 현재 위치
        current_x = feedback.current_pose.pose.position.x
        current_y = feedback.current_pose.pose.position.y
        
        # 남은 거리
        distance = feedback.distance_remaining
        
        # 예상 도착 시간
        eta = feedback.estimated_time_remaining.sec
        
        # 현재 복구 동작 수
        recoveries = feedback.number_of_recoveries
        
        self.get_logger().info(
            f"Progress: dist={distance:.2f}m, ETA={eta}s, recoveries={recoveries}"
        )
        
        # MQTT로 상태 전송 (선택)
        self.mqtt.publish("robot/nav_status", json.dumps({
            "distance": distance,
            "eta": eta,
            "recoveries": recoveries
        }))
    
    def result_callback(self, future):
        """BT 완료 결과 처리"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info("Goal reached successfully!")
            self.mqtt.publish("robot/arrived", "success")
        elif status == 5:  # CANCELED
            self.get_logger().warn("Goal was canceled")
            self.mqtt.publish("robot/arrived", "canceled")
        elif status == 6:  # ABORTED
            self.get_logger().error("Goal aborted (could not reach)")
            self.mqtt.publish("robot/arrived", "failed")
    
    def cancel_goal(self):
        """현재 Goal 취소"""
        if hasattr(self, '_goal_handle'):
            self._goal_handle.cancel_goal_async()
            self.get_logger().info("Goal cancel requested")
```

### 🔄 SLAM vs Nav2 모드 비교

| 항목 | SLAM 모드 | Nav2 모드 |
|------|-----------|-----------|
| **목적** | 미지의 환경 탐색 + 맵 생성 | 알려진 맵에서 목표점 이동 |
| **위치 추정** | SLAM Toolbox (맵 생성 중 추정) | **AMCL** (맵 기반 추정) |
| **맵** | 실시간 생성 (OccupancyGrid) | 저장된 맵 로드 (.pgm + .yaml) |
| **경로 계획** | 없음 (탐색 알고리즘) | **Planner** (A* 알고리즘) |
| **주행 제어** | auto_drive_node (직접 구현) | **BT Navigator + Controller** |
| **복구 동작** | 직접 구현 (회전, 후진) | **BT Recovery** (Spin, BackUp, Wait) |
| **외부 명령** | MQTT로 시작/중지만 | MQTT로 **목표점 지정** 가능 |

### 📍 RViz에서 BT/Nav2 확인하기

1. **Global Plan (전역 경로)**:
   - Add → By topic → `/plan` → Path
   - 녹색 선으로 표시

2. **Local Plan (로컬 경로)**:
   - Add → By topic → `/local_plan` → Path
   - 파란색 선으로 표시

3. **Costmap (장애물 맵)**:
   - Add → By topic → `/global_costmap/costmap` → Map
   - Add → By topic → `/local_costmap/costmap` → Map

4. **Goal 수동 설정**:
   - 상단 툴바에서 `2D Goal Pose` 클릭 (단축키: G)
   - 맵에서 목표 위치 클릭 + 드래그 (방향 지정)

### ⚠️ BT/Nav2 관련 트러블슈팅

| 문제 | 원인 | 해결 방법 |
|------|------|-----------|
| "Action server not available" | Nav2가 아직 시작 안됨 | launch 순서 확인, 대기 시간 늘리기 |
| Goal이 즉시 실패 | 목표가 장애물 안에 있음 | Costmap 확인, tolerance 늘리기 |
| 경로를 못 찾음 | 목표까지 길이 없음 | allow_unknown: true 설정 |
| 로봇이 회전만 함 | 최소 속도가 너무 높음 | min_vel_x, min_speed_theta 줄이기 |
| Recovery 반복 | 막다른 길 | 맵 확인, 목표 위치 변경 |
| 목표 근처에서 흔들림 | tolerance가 너무 작음 | xy_goal_tolerance 늘리기 |

---

## MQTT 통신 구조

> 📡 MQTT Broker는 서버(192.168.0.3)에서 운영됩니다. 자세한 서버 설정은 [slam_mqtt_server](https://github.com/ky51301130-jpg/slam_mqtt_server) 저장소를 참조하세요.

### 📬 필수 모니터링 토픽 (10개)

프로젝트 전체를 모니터링하려면 아래 토픽들이 필수입니다.

#### 📌 (A) Nav2 / 이동 관련

| MQTT Topic | 방향 | 설명 |
|------------|------|------|
| `robot/navigate_to_pose` | 로봇→서버 | Nav2 Goal 요청 값 (PLC 명령 수신 시 자동 발행) |
| `robot/nav_result` | 로봇→서버 | Nav2 완료/실패 이유 (SUCCEEDED/ABORTED/CANCELED) |
| `plc/location` | PLC→로봇 | PLC가 요청한 목적지 이름 ("station1") |
| `plc/goal` | PLC→로봇 | PLC 좌표 기반 Goal ({"x":1.0, "y":2.0, "yaw":0}) |

**흐름**: `PLC → plc/location → server_mqtt_bridge(서버) → mqtt_bridge(로봇) → nav2_goal_node → Nav2 → robot/nav_result`

#### 📌 (B) SLAM / Map 생성 파이프라인

| MQTT Topic | 방향 | 설명 |
|------------|------|------|
| `ros/map_cycle_complete` | 로봇→서버 | SLAM 맵 저장 완료 (8사이클 완료 시) → 서버에서 맵 병합 |
| `collision/photo_ready` | 로봇→서버 | 충돌 사진 URL → 서버가 다운로드하여 저장 (YOLO 학습용) |
| `qr_detected` | 로봇→서버 | QR 코드 감지 이벤트 |

#### 📌 (C) AI Vision 분석 (서버 발행)

| MQTT Topic | 방향 | 설명 |
|------------|------|------|
| `mqtt/pinky/detection` | 서버→구독자 | YOLO 감지 결과 (person, box 등) |
| `mqtt/pinky/obstacle_type` | 서버→구독자 | 감지된 장애물 유형 |

#### 📌 (D) 상태 모니터링

| MQTT Topic | 방향 | 설명 |
|------------|------|------|
| `slam_mode` | 로봇→서버 | 현재 로봇 모드 (SLAM/NAV2/IDLE) |
| `battery/status` | 로봇→서버 | 배터리 전압 (옵션) |

---

### 📬 전체 토픽 맵

```
MQTT Broker (192.168.0.3:1883)
│
├── 📥 외부 → 로봇 (수신)
│   │
│   ├── mcu/
│   │   └── sensors               # {"Lux": 150.5} - MCU 센서
│   │
│   ├── plc/
│   │   ├── location              # "station1" - 위치 이름 이동
│   │   └── goal                  # {"x":1.0, "y":2.0} - 좌표 이동
│   │
│   ├── server/
│   │   └── locations             # 위치 프리셋 업데이트
│   │
│   └── nav2/
│       └── initial_pose          # AMCL 초기 위치 설정
│
├── 📤 로봇 → 외부 (발행)
│   │
│   ├── robot/
│   │   ├── navigate_to_pose      # Goal 요청 (모니터링용)
│   │   ├── nav_status            # 진행 상태
│   │   ├── nav_result            # 완료/실패 결과
│   │   └── arrived               # 목표 도착
│   │
│   ├── slam_mode                 # SLAM/NAV2/IDLE
│   │
│   ├── battery/
│   │   └── status                # 배터리 전압 (옵션)
│   │
│   ├── collision/
│   │   └── photo_ready           # 충돌 사진 알림
│   │
│   ├── qr_detected               # QR 코드 감지
│   │
│   └── ros/
│       └── map_cycle_complete    # 맵 사이클 완료
│
└── 📊 AI 서버 발행
    └── mqtt/pinky/
        ├── detection             # YOLO 감지 결과
        └── obstacle_type         # 장애물 유형
```

---

### 📝 ROS2 ↔ MQTT 매핑 테이블

| ROS2 Topic | MQTT Topic | 방향 |
|------------|------------|------|
| `/robot_mode` | `slam_mode` | ROS2 → MQTT |
| `/nav2/status` | `robot/nav_status` | ROS2 → MQTT |
| `/nav2/status` (결과) | `robot/nav_result` | ROS2 → MQTT |
| `/nav2/arrived` | `robot/arrived` | ROS2 → MQTT |
| `/collision/photo_ready` | `collision/photo_ready` | ROS2 → MQTT |
| `/qr_detected` | `qr_detected` | ROS2 → MQTT |
| `/battery/voltage` | `battery/status` | ROS2 → MQTT |
| `/map_saver/cycle_complete` | `ros/map_cycle_complete` | ROS2 → MQTT |
| `mcu/sensors` | `/mqtt/mcu_sensors` | MQTT → ROS2 |
| `plc/location` | `/mqtt/plc_location` | MQTT → ROS2 |
| `plc/goal` | `/mqtt/plc_goal` | MQTT → ROS2 |

---

### 📝 메시지 예시

```python
# ===== MCU에서 센서 데이터 전송 =====
mqtt.publish("mcu/sensors", '{"Lux": 150.5}')

# 여러 센서 데이터 함께 전송 가능
mqtt.publish("mcu/sensors", json.dumps({
    "Lux": 150.5,
    "Temperature": 25.3,
    "Humidity": 45.0
}))

# ===== PLC에서 로봇에 이동 명령 =====
mqtt.publish("plc/location", "station1")
mqtt.publish("plc/goal", '{"x":2.0, "y":1.5, "yaw":1.57}')

# ===== 서버에서 위치 프리셋 업데이트 =====
mqtt.publish("server/locations", json.dumps({
    "station1": {"x": 1.0, "y": 0.5, "yaw": 0},
    "charging": {"x": -0.5, "y": 0, "yaw": 3.14}
}))

# ===== Nav2 초기 위치 설정 =====
mqtt.publish("nav2/initial_pose", '{"x":0, "y":0, "yaw":0}')
```

### 📝 모니터링 메시지 형식

```python
# ===== robot/navigate_to_pose (Goal 요청 모니터링) =====
{
    "type": "location",        # 또는 "coordinate"
    "value": "station1",       # 또는 {"x":1.0, "y":2.0, "yaw":0}
    "timestamp": 1733567890.123
}

# ===== robot/nav_result (결과) =====
{
    "result": "SUCCEEDED",     # SUCCEEDED/ABORTED/CANCELED/REJECTED
    "message": "Goal reached!",
    "goal": {"x": 1.0, "y": 0.5, "yaw": 0},
    "timestamp": 1733567900.456
}

# ===== slam_mode (로봇 모드) =====
{
    "mode": "SLAM",            # SLAM/NAV2/IDLE
    "timestamp": 1733567800.000
}

# ===== battery/status (배터리, 옵션) =====
{
    "voltage": 11.8,
    "timestamp": 1733567860.000
}
```

---

## RViz 시각화

### 🖥️ 실행 방법

```bash
# SLAM 시각화 (맵 생성 과정)
ros2 launch pinky_navigation slam_view.launch.xml

# Nav2 시각화 (Goal 설정 + 경로 확인)
ros2 launch pinky_navigation nav2_view.launch.xml
```

### 🎮 RViz 조작

| 도구 | 단축키 | 용도 |
|------|--------|------|
| **2D Pose Estimate** | P | AMCL 초기 위치 설정 |
| **2D Goal Pose** | G | 마우스로 목표점 설정 |
| **Interact** | I | 마커 상호작용 |

### 📊 표시 항목

- **Map**: 맵 이미지 (PGM → 그레이스케일)
- **LaserScan**: LiDAR 포인트 클라우드
- **Robot Model**: URDF 기반 로봇 형상
- **TF**: 좌표 프레임 관계
- **Global Costmap**: 전역 장애물 맵
- **Local Costmap**: 로컬 장애물 맵
- **Global Plan**: 전역 경로 (녹색)
- **Local Plan**: 로컬 경로 (파란색)

---

## 트러블슈팅

### ❌ 자주 발생하는 문제

#### 1. "Nav2 action server not available"

```bash
# Nav2 실행 확인
ros2 node list | grep -E 'amcl|planner|controller'

# 해결: Nav2 먼저 실행
ros2 launch nav2_bringup bringup_launch.py map:=/path/to/map.yaml
```

#### 2. MQTT 연결 실패

```bash
# 브로커 확인
ping 192.168.0.3
mosquitto_sub -h 192.168.0.3 -t '#' -v

# 해결: 브로커 서비스 확인
sudo systemctl status mosquitto
```

#### 3. 카메라 오류

```bash
# 카메라 테스트
libcamera-hello --list-cameras

# 권한 확인
sudo usermod -aG video $USER
```

#### 4. TF 오류 "map → base_footprint"

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# SLAM 또는 AMCL이 map 프레임 발행하는지 확인
ros2 topic echo /tf --filter "frame_id=='map'"
```

---

## 🔗 관련 저장소

| 저장소 | 역할 | 위치 |
|----------|------|------|
| 🤖 [slam_mqtt_project](https://github.com/ky51301130-jpg/slam_mqtt_project) | 로봇 측 코드 (현재) | Raspberry Pi (192.168.0.5) |
| 🖥️ [slam_mqtt_server](https://github.com/ky51301130-jpg/slam_mqtt_server) | 서버 측 코드 | PC (192.168.0.3) |

### 서버에서 제공하는 기능

- **MQTT Broker** - Mosquitto (:1883)
- **맵 업로드 서버** - Flask (:5100)
- **맵 병합** - 8장 ICP 정렬 + 과반수 투표
- **AI 비전** - ArUco + YOLO 감지
- **모니터링** - Grafana + InfluxDB
- **충돌 사진 저장** - YOLO 학습 데이터 수집

### 데이터 흐름

```
로봇 (192.168.0.5)                  서버 (192.168.0.3)
     │                                  │
     │ ─── SLAM 맵 업로드 ────────▶ │ nav2_map_builder (맵 병합)
     │ ─── 충돌 사진 URL ─────────▶ │ unified_server (사진 다운로드)
     │ ◀── PLC/MCU 명령 ─────────── │ server_mqtt_bridge
     │ ◀── 병합된 맵 (Nav2용) ───── │
     │                                  │
```

---

## 📁 파일 구조

```
slam_mqtt_project/
├── slam_mqtt_project/
│   │
│   │  ===== SLAM 모드 전용 (3개) =====
│   ├── auto_drive_node.py      # SLAM 자율 탐색 + ArUco HOME 도킹 (1065줄)
│   ├── map_saver_node.py       # 맵 저장 + 서버 업로드
│   ├── collision_photo_node.py # SLAM용 충돌 사진
│   │
│   │  ===== NAV2 모드 전용 (3개) =====
│   ├── nav2_goal_node.py       # MQTT/PLC → Nav2 Goal + ArUco 연동 (438줄)
│   ├── camera_stream_node.py   # Nav2용 스트리밍 (Flask)
│   ├── aruco_dock_node.py      # ArUco 정밀 도킹 (443줄) ← NEW!
│   │
│   │  ===== 공통 (3개) =====
│   ├── mqtt_bridge_node.py     # ROS2 ↔ MQTT 브릿지
│   ├── status_display_node.py  # LED + LCD 통합 표시 (287줄) ← 통합!
│   ├── robot_map_loader.py     # 로봇에서 맵 로드 (Nav2 시작용)
│   │
│   │  ===== 설정 도구 (3개) =====
│   ├── set_home_by_aruco.py    # ArUco 마커로 HOME 설정 ← NEW!
│   ├── set_home_pose.py        # 수동 HOME 위치 설정
│   ├── aruco_calibration.py    # 카메라 캘리브레이션
│   │
│   │  ===== 토픽 관리 =====
│   └── topics.py               # ROS2/MQTT 토픽 중앙 관리 (322줄) ← NEW!
│
├── launch/
│   ├── slam_exploration.launch.py  # SLAM 모드 런치
│   ├── nav2_mode.launch.py         # Nav2 모드 런치
│   └── set_home.launch.py          # HOME 설정 런치 ← NEW!
│
├── config/
│   └── nav2_params.yaml            # Nav2 파라미터
│
├── docs/
│   ├── QUICK_START.md              # 빠른 시작 가이드
│   ├── CODE_ANALYSIS.md            # 코드 분석
│   ├── SYSTEM_ARCHITECTURE.md      # 시스템 구조
│   └── README.md                   # docs용 README
│
└── README.md                       # 이 문서
```

---

## 📜 라이선스

MIT License

---

## 🙏 Special Thanks & 기반 프로젝트

### 🤖 Pinky Pro

이 프로젝트는 **[pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro)** ROS2 패키지를 기반으로 개발되었습니다.

| 항목 | 내용 |
|------|------|
| **원본 저장소** | [github.com/pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro) |
| **라이선스** | Apache-2.0 |
| **환경** | Ubuntu 24.04 + ROS2 Jazzy |

#### Pinky Pro 기여자

- [@kyunghwan51](https://github.com/kyunghwan51) - 민경환 (메인 개발자)
- [@byeongkyu](https://github.com/byeongkyu) - Byeong-Kyu Ahn (ROS2 패키지 개발, [pinky_robot](https://github.com/byeongkyu/pinky_robot) 참고)
- [@deMerui](https://github.com/deMerui) - nomaefg

#### Pinky Pro 참고 자료

- 📚 [Pinky Pro 수업 자료 (Google Drive)](https://drive.google.com/drive/folders/1MeBp7xXAmHrNdEJYtsQ25DtCSrlwyAwC)
- 💡 [Pinky Pro Assistants (ChatGPT)](https://chatgpt.com/g/g-69141c60b0908191975d16ce2421b768-pinky-pro-assistants)

### 사용된 패키지 (Pinky Pro 기반)

| 패키지 | 역할 |
|--------|------|
| `pinky_bringup` | 로봇 하드웨어 초기화 |
| `pinky_navigation` | Nav2 런치 및 설정 |
| `pinky_description` | URDF/로봇 모델 |
| `pinky_led` | WS281x LED 제어 |
| `pinky_emotion` | LCD 감정 표시 |
| `pinky_interfaces` | 커스텀 서비스/메시지 |

---

### 🛠️ 오픈소스 라이브러리

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/) - 로봇 운영체제
- [Nav2 Navigation Stack](https://nav2.org/) - 자율 네비게이션
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) - SLAM 알고리즘
- [paho-mqtt](https://www.eclipse.org/paho/) - MQTT 클라이언트
- [Flask](https://flask.palletsprojects.com/) - 웹 서버
- [Picamera2](https://github.com/raspberrypi/picamera2) - 라즈베리파이 카메라
- [rpi-ws281x](https://github.com/jgarff/rpi_ws281x) - WS281x LED 드라이버
- [OpenCV](https://opencv.org/) - 컴퓨터 비전 (ArUco 마커)
