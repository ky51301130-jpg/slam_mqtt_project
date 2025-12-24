# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.3.0] - 2025-12-24

### Added
- 🎯 **PLC 단일 문자 명령 지원** - `A`, `B` 단일 문자로 PORT_A, PORT_B 이동 가능
- 🔗 **ArUco 포트 좌표 수신** - `robot/navigate_to_pose` 토픽으로 정밀 좌표 수신
- 📡 **MQTT 구독 토픽 확장** - `/plc/#`, `robot/navigate_to_pose` 추가

### Changed
- 🔧 **MQTT 토픽 슬래시 통일**
  - `plc/#` → `/plc/#` (슬래시 포함)
  - `plc/location` → `/plc/location`
  - `plc/goal` → `/plc/goal`
- 📝 **PLC 명령 처리 개선** (`nav2_goal_node.py`)
  - 단일 문자: `A` → `PORT_A`
  - JSON (PLC): `{"A":1,"B":0}` → `PORT_A`
  - JSON (ArUco): `{"type":"aruco_port", "port":"A", ...}` → 좌표로 직접 이동
- 🗺️ **맵 다운로드 시점 변경** - Nav2 시작 전에 서버에서 맵 다운로드 (launch 시)

### Fixed
- 🐛 **PLC 신호 수신 불가 버그** - MQTT 토픽 매칭 오류 수정 (`/plc/location` ≠ `plc/location`)
- 🐛 **ArUco 감지 무한 루프** - 8사이클 완료 시 ArUco 감지 중지

### Documentation
- 📚 README.md PLC 통신 섹션 전면 개편
- 📚 MQTT 토픽 및 메시지 형식 예시 업데이트

---

## [1.2.0] - 2025-12-23

### Added
- 🤖 **Pinky Pro 기반 명시** - 모든 문서에 [pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro) 기반 프로젝트임을 명시
- 📷 **SLAM 모드 카메라 스트리밍** - 충돌 사진뿐만 아니라 AI 분석용 1fps 스트리밍 추가 (포트 5200)
- 🗺️ **맵 완성도 표시** - map_saver에서 맵 완성도(%) 실시간 로깅
- 🌐 **collision_photo 웹서버 개선** - 맵/사진 통합 웹 뷰어 (`/`, `/maps`, `/photos`)

### Changed
- 🔄 **도킹 순서 개선** - PORT_B 도킹 완료 후 180도 회전 → PORT_A로 이동
- 🚀 **포트 정리**
  - SLAM 모드: 5000 (충돌 사진), 5200 (AI 스트리밍)
  - Nav2 모드: 5200 (AI 스트리밍)
- ⚡ **도킹 속도 조정**
  - 전진 속도: 0.08 → 0.12 m/s
  - 후진 속도: 0.05 → 0.06 m/s
  - 회전 P 게인: 0.6 → 0.4 (더 부드러운 접근)
- 📝 **코드 최적화** - 모든 노드 코드 간소화 및 가독성 향상
  - `auto_drive_node.py`: 1035줄 → 620줄 (40% 감소)
  - `camera_stream_node.py`: 455줄 → 158줄 (65% 감소)
  - `collision_photo_node.py`: 362줄 → 224줄 (38% 감소)
  - `map_saver_node.py`: 422줄 → 265줄 (37% 감소)

### Fixed
- 🐛 **도킹 중 회피 로직** - 장애물 회피 후 마커 재탐색 개선
- 🐛 **초음파 센서 노이즈** - 버퍼 크기 5 → 10으로 증가하여 노이즈 필터링 강화

### Documentation
- 📚 모든 README에 Pinky Pro 기여자 및 참고 자료 추가
- 📚 포트 정보 업데이트 (SLAM/Nav2 모드별 정리)
- 📚 Special Thanks 섹션 추가 (원본 저장소, 라이선스, 기여자)

---

## [1.1.0] - 2025-12-17

### Added
- ArUco 마커 기반 정밀 도킹 시스템
- PORT_A/PORT_B 자동 좌표 저장
- LED 진행률 표시 (8단계)
- LCD 상태 표시 통합

### Changed
- 도킹 존 방식으로 변경 (size 기반)
- YAML에 PORT 좌표 저장

---

## [1.0.0] - 2025-12-10

### Added
- 초기 릴리스
- SLAM 자율 탐색 모드
- Nav2 네비게이션 모드
- MQTT 브릿지
- 8-cycle 맵 저장 시스템
