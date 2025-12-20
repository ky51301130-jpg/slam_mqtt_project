# 💻 코드 분석

> 노드별 핵심 코드, AMCL/Behavior Tree 상세 설명

---

## 📋 목차

1. [노드별 핵심 코드](#노드별-핵심-코드)
2. [AMCL 상세 설명](#amcl-상세-설명)
3. [Behavior Tree 상세 설명](#behavior-tree-상세-설명)
4. [Nav2 파라미터 설명](#nav2-파라미터-설명)

---

## 노드별 핵심 코드

### 1. 🚗 auto_drive_node.py (SLAM 자율 탐색)

**역할**: 미지의 환경을 자동으로 탐색하며 SLAM Toolbox가 맵을 생성하도록 주행

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

---

### 5. 🚨 collision_photo_node.py (SLAM용 충돌 사진)

**역할**: 충돌 감지 시에만 고해상도 사진 촬영

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

---

### 6. 📡 mqtt_bridge_node.py (ROS2 ↔ MQTT)

**역할**: 외부 시스템(PLC, 서버)과 ROS2 연결

```python
# ===== MQTT → ROS2 =====
def on_message(self, client, userdata, msg):
    payload = msg.payload.decode("utf-8")
    
    if msg.topic == "plc/location":
        self.pub_plc_location.publish(String(data=payload))
        # 모니터링용 재발행
        self.mqtt_publish("robot/navigate_to_pose", json.dumps({
            "type": "location",
            "value": payload,
            "timestamp": time.time()
        }))

# ===== ROS2 → MQTT =====
def robot_mode_cb(self, msg: String):
    mode = msg.data.upper()
    if mode != self.last_mode:
        self.last_mode = mode
        self.mqtt_publish("slam_mode", json.dumps({
            "mode": mode,
            "timestamp": time.time()
        }))

def nav_status_cb(self, msg: String):
    self.mqtt_publish("robot/nav_status", msg.data)
    
    # 결과 전용 발행
    try:
        data = json.loads(msg.data)
        status = data.get("status", "")
        if status in ["SUCCEEDED", "ABORTED", "CANCELED", "REJECTED"]:
            self.mqtt_publish("robot/nav_result", json.dumps({
                "result": status,
                "message": data.get("message", ""),
                "goal": data.get("goal", {}),
                "timestamp": time.time()
            }))
    except:
        pass
```

---

### 7. 🔔 status_display_node.py (LED + LCD 통합 표시)

**역할**: WS281x LED + LCD를 통합 관리하여 로봇 상태 시각화
- 기존 `led_controller_node.py` + `lcd_status_node.py` + `ultrasonic_node.py` 통합
- 저전력 최적화: 변화 있을 때만 업데이트

```python
# ===== MCU 센서 데이터 수신 =====
LUX_THRESHOLD = 100  # 밝음/어두움 기준값 (lux)

def sensor_cb(self, msg):
    try:
        data = json.loads(msg.data)  # {"Lux": 150.5, ...}
        
        if "Lux" in data:
            self.current_lux = float(data["Lux"])
            
            # Nav2 모드: 항상 Lux 기반 LED
            # SLAM 모드: 주행 중이 아닐 때만 Lux 적용
            if self.robot_mode == "NAV2" or not self.is_driving:
                if self.current_lux >= LUX_THRESHOLD:
                    self.current_led_mode = "bright"   # GREEN
                else:
                    self.current_led_mode = "dark"     # BLUE
    except:
        pass

# ===== 맵 저장 진행률 표시 (SLAM 모드) =====
def _set_led_progress(self, count, total=8):
    for i in range(NUM_LEDS):  # 8개 LED
        if i < count:
            self.leds.set_pixel(i, ORANGE)  # 완료
        else:
            self.leds.set_pixel(i, RED)      # 대기
    self.leds.show()

# ===== 상태별 색상 =====
colors = {
    "driving": RED,       # SLAM 주행 중
    "map_saving": None,   # 진행률 표시
    "bright": GREEN,      # Lux >= 100 (밝은 환경)
    "dark": BLUE,         # Lux < 100 (어두운 환경)
    "idle": OFF           # 대기
}

# ===== LCD 배터리/모드 표시 =====
def update_lcd(self):
    img = Image.new('RGB', (320, 240), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    
    # 모드 표시 (SLAM: 파랑, NAV2: 보라)
    mode_color = MODE_COLORS.get(self.robot_mode)
    draw.rectangle([(0, 0), (320, 50)], fill=mode_color)
    
    # 배터리 바
    bar_width = int(280 * self.battery_percent / 100)
    draw.rectangle([(20, 80), (300, 130)], outline=(100, 100, 100))
    draw.rectangle([(22, 82), (22 + bar_width, 128)], fill=self._get_battery_color())
    
    self.lcd.img_show(img)
```

---

### 8. 🎯 aruco_dock_node.py (ArUco 정밀 도킹) - NEW!

**역할**: Nav2 도착 후 ArUco 마커로 정밀 위치 조정

```python
# ===== ArUco 마커 ID → 포트 매핑 =====
MARKER_PORT_MAP = {
    0: "HOME",       # ID 0 = HOME (충전/기준점)
    1: "PORT_A",     # ID 1 = 작업위치 A
    2: "PORT_B",     # ID 2 = 작업위치 B
}

# ===== 도킹 제어 루프 =====
def dock_control_loop(self):
    if not self.docking_enabled:
        return
    
    # ArUco 마커 감지
    corners, ids, _ = self.aruco_detector.detectMarkers(frame)
    
    if self.target_marker_id in ids:
        # 마커 위치 계산
        rvec, tvec = cv2.solvePnP(...)
        distance = np.linalg.norm(tvec)
        
        # 정밀 접근
        if distance > self.DOCK_DISTANCE:
            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = -center_error * self.ANGULAR_SPEED
        else:
            # 도킹 완료!
            self.save_port_position()
            self.publish_arrival()
```
```

---

## AMCL 상세 설명

### 🎯 AMCL이란? (Adaptive Monte Carlo Localization)

**"로봇이 맵 안에서 자신이 어디 있는지 알아내는 알고리즘"**

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

### 핵심 파라미터 (`nav2_params.yaml`)

```yaml
amcl:
  ros__parameters:
    # 파티클 수 (정확도 vs 성능)
    min_particles: 500      # 최소 파티클 수
    max_particles: 2000     # 최대 파티클 수
    
    # 업데이트 조건
    update_min_d: 0.2       # 0.2m 이동해야 업데이트
    update_min_a: 0.5       # 0.5rad(~29°) 회전해야 업데이트
    
    # 센서 모델
    laser_model_type: "likelihood_field"
    laser_max_range: 12.0
    laser_min_range: 0.1
    max_beams: 180
    
    # 오도메트리 모델
    odom_model_type: "diff"    # 차동 구동 로봇
    alpha1: 0.2   # 회전→회전 노이즈
    alpha2: 0.2   # 이동→회전 노이즈
    alpha3: 0.2   # 이동→이동 노이즈
    alpha4: 0.2   # 회전→이동 노이즈
```

### 초기 위치 설정 코드

```python
def set_initial_pose(self, x, y, yaw):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    
    # Yaw → Quaternion
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    # Covariance (불확실성)
    msg.pose.covariance[0] = 0.25   # x 분산
    msg.pose.covariance[7] = 0.25   # y 분산
    msg.pose.covariance[35] = 0.06  # yaw 분산
    
    self.initial_pose_pub.publish(msg)
```

---

## Behavior Tree 상세 설명

### 🌳 Behavior Tree란?

**"로봇이 목표까지 가는 과정에서 어떤 순서로 무엇을 할지 결정하는 의사결정 나무"**

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
│                                                                          │
│  실행 흐름:                                                              │
│  1. ComputePathToPose: 목표까지 전역 경로 계산 (A* 알고리즘)             │
│  2. FollowPath: 계산된 경로를 따라 이동 (DWB Controller)                │
│  3. (실패 시) Recovery 실행: Spin → BackUp → Wait 순서로 시도           │
│  4. Recovery 후 다시 1번부터 재시도                                      │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### BT 노드 타입

| 노드 타입 | 기호 | 설명 |
|-----------|------|------|
| **Sequence** | → | 자식을 순서대로 실행, 하나라도 실패하면 중단 |
| **Fallback** | ? | 자식을 순서대로 실행, 하나가 성공하면 중단 |
| **Parallel** | ⇉ | 자식을 동시에 실행 |
| **Decorator** | ◇ | 자식 결과를 변형 |
| **Action** | ▢ | 실제 동작 수행 |
| **Condition** | ○ | 조건 확인 (True/False) |

### BT Navigator 사용 코드

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class Nav2GoalNode(Node):
    def __init__(self):
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
    
    def send_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        distance = fb.distance_remaining
        recoveries = fb.number_of_recoveries
        print(f"Distance: {distance:.2f}m, Recoveries: {recoveries}")
    
    def result_callback(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            print("Goal reached!")
        elif status == 6:  # ABORTED
            print("Goal aborted")
```

---

## Nav2 파라미터 설명

### Planner Server (전역 경로 계획)

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5              # 목표 허용 오차 (m)
      use_astar: true             # A* 알고리즘 사용
      allow_unknown: true         # 미탐색 영역 통과 허용
```

### Controller Server (로컬 경로 추종)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # 속도 제한
      max_vel_x: 0.3              # 최대 전진 속도 (m/s)
      max_vel_theta: 1.0          # 최대 회전 속도 (rad/s)
      
      # 가속도 제한
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      
      # 목표 허용 오차
      xy_goal_tolerance: 0.25     # 위치 (m)
      yaw_goal_tolerance: 0.25    # 방향 (rad)
```

### 복구 동작

```yaml
spin:
  ros__parameters:
    max_rotational_vel: 1.0       # 최대 회전 속도
    min_rotational_vel: 0.4       # 최소 회전 속도

backup:
  ros__parameters:
    simulate_ahead_time: 2.0      # 충돌 시뮬레이션 시간
```

---

## 🔄 SLAM vs Nav2 모드 비교

| 항목 | SLAM 모드 | Nav2 모드 |
|------|-----------|-----------|
| **목적** | 미지의 환경 탐색 + 맵 생성 | 알려진 맵에서 목표점 이동 |
| **위치 추정** | SLAM Toolbox | **AMCL** |
| **맵** | 실시간 생성 | 저장된 맵 로드 |
| **경로 계획** | 없음 (탐색 알고리즘) | **Planner** (A*) |
| **주행 제어** | auto_drive_node | **BT Navigator** |
| **복구 동작** | 직접 구현 | **BT Recovery** |
