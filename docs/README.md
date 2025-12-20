# 🤖 SLAM MQTT Project

> Raspberry Pi 기반 자율주행 로봇의 SLAM/Nav2 통합 시스템

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
| **SLAM** | 자율 탐색 + 맵 생성 | 충돌 시 사진 | 5001 |
| **Nav2** | 맵 기반 목표점 이동 | 1fps 스트리밍 | 5000 |

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
│   ├── auto_drive_node.py      # SLAM 자율 탐색
│   ├── map_saver_node.py       # 맵 저장 + 서버 업로드
│   ├── nav2_goal_node.py       # MQTT → Nav2 Goal
│   ├── camera_stream_node.py   # Nav2용 스트리밍
│   ├── collision_photo_node.py # SLAM용 충돌 사진
│   ├── mqtt_bridge_node.py     # ROS2 ↔ MQTT
│   ├── ultrasonic_node.py      # 초음파 센서
│   ├── led_controller_node.py  # LED 상태 표시
│   └── lcd_status_node.py      # LCD 배터리 표시
├── launch/
│   ├── slam_exploration.launch.py  # SLAM 모드 런치
│   └── nav2_mode.launch.py         # Nav2 모드 런치
├── config/
│   └── nav2_params.yaml            # Nav2 파라미터
├── docs/
│   ├── README.md                   # 프로젝트 개요
│   ├── SYSTEM_ARCHITECTURE.md      # 시스템 구조
│   ├── CODE_ANALYSIS.md            # 코드 분석
│   └── QUICK_START.md              # 빠른 시작
└── README.md                       # 메인 README
```

---

## 📜 라이선스

MIT License

## 🙏 감사의 말

- ROS2 Jazzy
- Nav2 Navigation Stack
- SLAM Toolbox
- paho-mqtt
- Flask
- Picamera2
