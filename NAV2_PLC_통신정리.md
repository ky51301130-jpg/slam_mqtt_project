# PLC ↔ Nav2 통신 정리

## 📊 전체 흐름도

```
┌─────────┐      MQTT       ┌──────────────────┐      MQTT       ┌────────────────┐
│   PLC   │ ──────────────► │  Server Bridge   │ ──────────────► │  nav2_goal_node│
│(MCU/ESP)│                 │  (192.168.0.3)   │                 │    (로봇)       │
└─────────┘                 └──────────────────┘                 └────────────────┘
     │                              │                                    │
     │  /plc/location               │  robot/navigate_to_pose            │  NavigateToPose
     │  {"A":1,"B":0}               │  (JSON 전달)                       │  (Nav2 Action)
     │                              │                                    │
     └──────────────────────────────┴────────────────────────────────────┘
```

---

## 🔌 MQTT 토픽 (PLC → 로봇)

### 1. 위치 명령 (PORT로 이동)
| 토픽 | 값 | 설명 |
|------|-----|------|
| `/plc/location` | `{"A":1,"B":0}` | PORT_A로 이동 |
| `/plc/location` | `{"A":0,"B":1}` | PORT_B로 이동 |
| `/plc/location` | `"cancel"` or `"stop"` | 이동 취소 |

**레거시 포맷 (호환)**:
```
/plc/location → "PORT_A"
/plc/location → "HOME"
```

### 2. 좌표 명령 (직접 좌표 이동)
| 토픽 | 값 | 설명 |
|------|-----|------|
| `plc/goal` | `{"x":1.5, "y":2.0, "yaw":0}` | 절대 좌표로 이동 |

### 3. PORT 상태 신호
| 토픽 | 값 | 설명 |
|------|-----|------|
| `plc/port_status` | `{"A":1,"B":0}` | A 작업 가능, B 불가 |

---

## 📡 MQTT 토픽 (로봇 → 외부)

### 네비게이션 상태
| 토픽 | 값 예시 | 설명 |
|------|---------|------|
| `robot/nav_status` | `{"status":"NAVIGATING","message":"(1.5,2.0)","goal":{...}}` | 진행 상태 |
| `robot/nav_result` | `{"result":"SUCCEEDED","message":"Goal reached!"}` | 최종 결과 |
| `robot/arrived` | `{"goal":{...},"port":"PORT_A","status":"arrived"}` | 도착 알림 |

### 상태 값 종류
| status | 의미 |
|--------|------|
| `NAVIGATING` | 이동 중 |
| `SUCCEEDED` | 도착 완료 |
| `ABORTED` | 실패 (경로 막힘) |
| `CANCELED` | 취소됨 |
| `REJECTED` | Nav2가 거부 |
| `ERROR` | 오류 |

---

## 🤖 ROS2 토픽 (내부)

### Nav2 관련
| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `ros/nav2/goal` | PoseStamped | 수신 | Goal 목표 (RViz에서 클릭) |
| `ros/nav2/cancel` | Bool | 수신 | Goal 취소 (True) |
| `ros/nav2/status` | String/JSON | 발행 | 진행 상태 |
| `ros/nav2/arrived` | Bool | 발행 | 도착 여부 |
| `/initialpose` | PoseWithCovarianceStamped | 발행 | AMCL 초기 위치 |

### ArUco 도킹 연계
| 토픽 | 타입 | 설명 |
|------|------|------|
| `ros/aruco/target_port` | String | 목표 포트 (PORT_A, PORT_B) |
| `ros/aruco/dock_enable` | Bool | 도킹 모드 활성화 |
| `ros/aruco/port_goals` | String/JSON | 저장된 포트 좌표 업데이트 |

---

## 📂 PORT 좌표 저장소

### 파일 위치
```
/home/pinky/saved_maps/port_goals.json
```

### 파일 형식
```json
{
  "PORT_A": {"x": 1.5, "y": 2.0, "yaw": 0.0},
  "PORT_B": {"x": -1.0, "y": 3.5, "yaw": 1.57},
  "HOME": {"x": 0.0, "y": 0.0, "yaw": 0.0}
}
```

### 좌표 등록 방법
1. **수동 편집**: `port_goals.json` 직접 수정
2. **ArUco 학습**: 로봇이 ArUco 마커 앞에서 위치 저장
3. **MQTT 명령**: `ros/aruco/port_goals` 토픽으로 업데이트

---

## 💡 사용 예시

### 1. PLC에서 PORT_A로 이동 명령
```
MQTT Publish:
  Topic: /plc/location
  Payload: {"A":1,"B":0}
```

### 2. 직접 좌표로 이동
```
MQTT Publish:
  Topic: plc/goal
  Payload: {"x": 2.5, "y": 1.0, "yaw": 0.0}
```

### 3. 이동 취소
```
MQTT Publish:
  Topic: /plc/location
  Payload: cancel
```

### 4. AMCL 초기 위치 설정
```
MQTT Publish:
  Topic: nav2/initial_pose
  Payload: {"x": 0.0, "y": 0.0, "yaw": 0.0}
```

---

## 🔄 도착 후 동작 순서

```
1. PLC: /plc/location → {"A":1,"B":0}
   ↓
2. nav2_goal_node: PORT_A 좌표 조회 (port_goals.json)
   ↓
3. nav2_goal_node → Nav2: NavigateToPose 액션 호출
   ↓
4. Nav2: 경로 계획 + 이동
   ↓
5. nav2_goal_node: robot/nav_status → {"status":"NAVIGATING",...}
   ↓
6. Nav2: 도착 완료
   ↓
7. nav2_goal_node: 
   - ros/nav2/arrived → True
   - ros/aruco/target_port → "PORT_A"
   - ros/aruco/dock_enable → True (정밀 도킹 시작)
   - robot/arrived → {"port":"PORT_A","status":"arrived"}
   ↓
8. aruco_dock_node: ArUco 마커 정밀 도킹 수행
   ↓
9. 도킹 완료 → ros/aruco/dock_complete → True
```

---

## ⚠️ 주의사항

1. **토픽 슬래시**: `/plc/location`은 앞에 `/` 있음 (실제 PLC 통신용)
2. **JSON 형식**: PLC에서 ASCII로 보내므로 `{"A":1,"B":0}` 그대로 사용
3. **좌표계**: map 프레임 기준 (meters, radians)
4. **yaw 단위**: 라디안 (0 = 동쪽, π/2 = 북쪽, π = 서쪽)

---

## 🛠️ 관련 노드

| 노드 | 파일 | 역할 |
|------|------|------|
| `nav2_goal_node` | nav2_goal_node.py | MQTT→Nav2 변환 |
| `server_mqtt_bridge` | server_mqtt_bridge.py | 서버 MQTT 브릿지 |
| `aruco_dock_node` | aruco_dock_node.py | ArUco 정밀 도킹 |
| `auto_drive_node` | auto_drive_node.py | SLAM 자율주행 + HOME 복귀 |
