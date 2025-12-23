# 🤖 SLAM MQTT Project

> Raspberry Pi 기반 자율주행 로봇의 SLAM/Nav2 통합 시스템 (로봇 측 코드)

> 🖥️ 서버 측 코드는 [slam_mqtt_server](https://github.com/ky51301130-jpg/slam_mqtt_server) 저장소를 참조하세요.

> 🤖 이 프로젝트는 [pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro) ROS2 패키지를 기반으로 개발되었습니다.

## 📋 문서 구조

| 문서 | 설명 |
|------|------|
| [📖 README.md](README.md) | 프로젝트 개요 및 문서 안내 (현재 문서) |
| [🏗️ SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) | 시스템 구조, 노드 구성, MQTT 토픽 |
| [💻 CODE_ANALYSIS.md](CODE_ANALYSIS.md) | 노드별 코드 분석, AMCL/BT 상세 설명 |
| [🚀 QUICK_START.md](QUICK_START.md) | 빠른 시작, 설치, 실행, 트러블슈팅 |

---

## 🎯 프로젝트 개요

미지의 환경을 자율 탐색하여 맵을 생성하고, 생성된 맵을 기반으로 목표점 네비게이션을 수행하는 ROS2 패키지입니다.

### 주요 기능

| 모드 | 기능 | 카메라 | 포트 |
|------|------|--------|------|
| **SLAM** | 자율 탐색 + 맵 생성 | 충돌 사진 + AI 분석 | 5000 (사진), 5200 (스트리밍) |
| **Nav2** | 맵 기반 목표점 이동 | 1fps 스트리밍 | 5200 |

### 하드웨어 구성

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
└─────────────────────────────────────────┘
         │
         │ MQTT (mcu/sensors)
         ▼
┌─────────────────────────────────────────┐
│          서버 (192.168.0.3)             │
├─────────────────────────────────────────┤
│ • MQTT Broker (:1883)                   │
│ • Map Server Flask (:5000, :5200)       │
│ • YOLO 분석 서버                        │
│ • PLC 시스템                            │
└─────────────────────────────────────────┘
```

---

## 📁 파일 구조

```
slam_mqtt_project/
├── slam_mqtt_project/
│   │
│   │  ===== SLAM 모드 전용 =====
│   ├── auto_drive_node.py      # SLAM 자율 탐색 + ArUco HOME 도킹
│   ├── map_saver_node.py       # 맵 저장 + 서버 업로드
│   ├── collision_photo_node.py # SLAM용 충돌 사진
│   │
│   │  ===== NAV2 모드 전용 =====
│   ├── nav2_goal_node.py       # MQTT/PLC → Nav2 Goal + ArUco 연동
│   ├── camera_stream_node.py   # Nav2용 스트리밍 (Flask)
│   ├── aruco_dock_node.py      # ArUco 정밀 도킹 ← NEW!
│   │
│   │  ===== 공통 =====
│   ├── mqtt_bridge_node.py     # ROS2 ↔ MQTT 브릿지
│   ├── status_display_node.py  # LED + LCD 통합 표시 ← 통합!
│   ├── robot_map_loader.py     # 로봇에서 맵 로드
│   │
│   │  ===== 설정 도구 =====
│   ├── set_home_by_aruco.py    # ArUco 마커로 HOME 설정
│   ├── set_home_pose.py        # 수동 HOME 위치 설정
│   ├── aruco_calibration.py    # 카메라 캘리브레이션
│   │
│   │  ===== 토픽 관리 =====
│   └── topics.py               # ROS2/MQTT 토픽 중앙 관리 ← NEW!
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
│   ├── README.md                   # 프로젝트 개요
│   ├── SYSTEM_ARCHITECTURE.md      # 시스템 구조
│   ├── CODE_ANALYSIS.md            # 코드 분석
│   └── QUICK_START.md              # 빠른 시작
│
└── README.md                       # 메인 README
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
- [@byeongkyu](https://github.com/byeongkyu) - Byeong-Kyu Ahn (ROS2 패키지 개발)
- [@deMerui](https://github.com/deMerui) - nomaefg

#### Pinky Pro 참고 자료

- 📚 [Pinky Pro 수업 자료 (Google Drive)](https://drive.google.com/drive/folders/1MeBp7xXAmHrNdEJYtsQ25DtCSrlwyAwC)
- 💡 [Pinky Pro Assistants (ChatGPT)](https://chatgpt.com/g/g-69141c60b0908191975d16ce2421b768-pinky-pro-assistants)

### 🛠️ 오픈소스 라이브러리

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/) - 로봇 운영체제
- [Nav2 Navigation Stack](https://nav2.org/) - 자율 네비게이션
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) - SLAM 알고리즘
- [paho-mqtt](https://www.eclipse.org/paho/) - MQTT 클라이언트
- [Flask](https://flask.palletsprojects.com/) - 웹 서버
- [Picamera2](https://github.com/raspberrypi/picamera2) - 라즈베리파이 카메라
