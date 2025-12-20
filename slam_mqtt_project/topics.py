#!/usr/bin/env python3
"""
=============================================================================
                        ROS2 & MQTT 토픽 정의
=============================================================================
모든 토픽을 한 곳에서 관리하여 수정과 유지보수를 쉽게 합니다.
토픽 변경 시 이 파일만 수정하면 됩니다.

사용법:
    from slam_mqtt_project.topics import ROS, MQTT
    self.create_subscription(String, ROS.ROBOT_MODE, self.cb, 10)
    self.mqtt.publish(MQTT.SLAM_MODE, payload)
=============================================================================
"""


class ROS:
    """ROS2 토픽 정의"""
    
    # ======================== 센서 ========================
    SCAN = "/scan"                          # LiDAR 스캔 (LaserScan)
    ODOM = "/odom"                          # 오도메트리 (Odometry)
    ULTRASONIC = "/ultrasonic"              # 초음파 거리 (Float32)
    
    # ======================== 모터/주행 ========================
    CMD_VEL = "cmd_vel"                     # 속도 명령 (Twist)
    
    # ======================== 로봇 상태 ========================
    ROBOT_MODE = "/robot_mode"              # SLAM/NAV2/IDLE (String)
    ROBOT_STATUS = "ros/robot_status"       # 상태 메시지 (String)
    
    # ======================== 자율 주행 (SLAM) ========================
    AUTO_DRIVE_ENABLE = "ros/auto_drive/enable"   # 자율 주행 시작/정지 (Bool)
    AUTO_DRIVE_ACTIVE = "ros/auto_drive/active"   # 주행 중 여부 (Bool)
    CAMERA_TRIGGER = "ros/camera_trigger"         # 카메라 캡처 트리거 (Bool)
    
    # ======================== 맵 저장 ========================
    MAP = "/map"                            # OccupancyGrid 맵 (시스템)
    MAP_SAVER_SAVED = "ros/map_saver/saved"       # 저장된 맵 번호 (Int32)
    MAP_SAVER_CYCLE = "ros/map_saver/cycle_complete"   # 사이클 완료 (String)
    MAP_SAVER_COMPLETE = "ros/map_saver/map_complete"  # 맵 완성 (Bool)
    
    # ======================== Odom 리셋 ========================
    ODOM_RESET = "ros/odom/reset"           # Odom 리셋 명령 (Bool)
    
    # ======================== 귀환 ========================
    ROBOT_AT_HOME = "ros/robot/at_home"     # 홈 위치 도착 (Bool)
    ROBOT_RETURN_HOME = "ros/robot/return_home"   # 귀환 명령 (Bool)
    
    # ======================== Nav2 ========================
    NAV2_MODE = "ros/nav2/mode"             # Nav2 모드 (String)
    NAV2_MAP_READY = "ros/nav2/map_ready"   # 맵 로드 완료 (Bool)
    NAV2_GOAL = "ros/nav2/goal"             # Goal 목표 (PoseStamped)
    NAV2_CANCEL = "ros/nav2/cancel"         # Goal 취소 (Bool)
    NAV2_STATUS = "ros/nav2/status"         # 진행 상태 (String/JSON)
    NAV2_ARRIVED = "ros/nav2/arrived"       # 도착 여부 (Bool)
    INITIAL_POSE = "/initialpose"           # AMCL 초기 위치 (시스템)
    
    # ======================== 충돌 감지 ========================
    COLLISION_TRIGGER = "ros/collision/trigger"       # 충돌 트리거 (Bool)
    COLLISION_PHOTO_READY = "ros/collision/photo_ready"  # 사진 준비 (String/JSON)
    COLLISION_IMAGE = "ros/collision/image"           # 압축 이미지 (CompressedImage)
    
    # ======================== MQTT 브릿지 (외부→ROS2) ========================
    MQTT_MCU_SENSORS = "mqtt/mcu_sensors"       # MCU 센서 데이터 (String/JSON)
    MQTT_PLC_LOCATION = "mqtt/plc_location"     # PLC 위치 명령 (String)
    MQTT_PLC_GOAL = "mqtt/plc_goal"             # PLC 좌표 명령 (String/JSON)
    
    # ======================== 배터리 ========================
    BATTERY_VOLTAGE = "ros/battery/voltage"     # 전압 (Float32)
    BATTERY_PRESENT = "ros/battery/present"     # 배터리 잔량 (Float32)
    
    # ======================== QR 코드 (장애물 마커) ========================
    # YOLO 서버에서 QR 감지 → 장애물 유형 인식 (Nav2와 무관)
    QR_OBSTACLE = "ros/qr/obstacle"             # QR 장애물 감지 (String: 장애물 유형)
    
    # ======================== ArUco 마커 (HOME + PORT) ========================
    # ID 0 = HOME (기준점, 충전 위치)
    # ID 1~N = PORT_A, PORT_B, ... (작업 위치)
    ARUCO_HOME_DETECTED = "ros/aruco/home_detected"   # HOME 마커 감지 (String/JSON)
    ARUCO_PORT_DETECTED = "ros/aruco/port_detected"   # PORT 마커 감지 (String/JSON)
    
    # ======================== 카메라/AI ========================
    CAMERA_IMAGE = "ros/camera/image"           # 카메라 이미지 (CompressedImage)
    OBSTACLE_ACTION = "ros/obstacle/action"     # 장애물 동작 (String)
    STOP_ROBOT = "ros/stop_robot"               # 긴급 정지 (Bool)
    
    # ======================== AI 서버 응답 (외부→ROS2) ========================
    AI_DETECTION = "mqtt/pinky/detection"       # 감지 결과 (String/JSON)
    AI_OBSTACLE_TYPE = "mqtt/pinky/obstacle_type"   # 장애물 유형 (String)


# ======================== 네트워크 설정 (IP/PORT) ========================
class NET:
    """네트워크 설정 - IP 주소 및 포트 중앙 관리"""
    
    # IP 주소
    SERVER_IP = "192.168.0.3"       # PC 서버 (MQTT, Flask, InfluxDB)
    ROBOT_IP = "192.168.0.5"        # 로봇 (라즈베리파이)
    MCU_IP = "192.168.0.4"          # MCU (ESP32 등)
    PLC_IP = "192.168.0.155"        # PLC
    
    # MQTT
    MQTT_PORT = 1883
    
    # Flask 서버 포트 (로봇 → PC로 업로드)
    COLLISION_PORT = 5000           # 충돌 사진 (로봇에서 제공)
    MAP_UPLOAD_PORT = 5100          # 맵 업로드 (PC 서버)
    STREAMING_PORT = 5200           # 카메라 스트리밍 (로봇에서 제공)
    
    # 업로드 URL
    @classmethod
    def map_upload_url(cls):
        return f"http://{cls.SERVER_IP}:{cls.MAP_UPLOAD_PORT}/upload"
    
    @classmethod
    def collision_photo_url(cls, filename=""):
        base = f"http://{cls.ROBOT_IP}:{cls.COLLISION_PORT}/photos"
        return f"{base}/{filename}" if filename else base
    
    @classmethod
    def stream_url(cls):
        return f"http://{cls.ROBOT_IP}:{cls.STREAMING_PORT}/image.jpg"


class MQTT:
    """MQTT 토픽 정의"""
    
    # ======================== 연결 설정 (NET에서 가져옴) ========================
    HOST = NET.SERVER_IP
    PORT = NET.MQTT_PORT
    
    # ======================== 구독 (외부 → 로봇) ========================
    SUB_MCU_SENSORS = "/mcu/sensors"         # MCU 센서 (Lux 등) 이거 실제니까 수정하지마 절대로
    SUB_PLC = "plc/#"                       # PLC 명령 (location, goal)
    SUB_PLC_LOCATION = "/plc/location"       # PLC 위치 명령: {"A":1,"B":0} 또는 {"A":0,"B":1} 이것도 절대 수정하지마 ASCII채로 옴
    SUB_PLC_GOAL = "plc/goal"               # PLC 좌표 명령: {"x":1.0,"y":2.0,"yaw":0}
    SUB_SERVER = "server/#"                 # 서버 명령
    SUB_NAV2 = "nav2/#"                     # Nav2 명령
    
    # 구독 토픽 리스트
    SUBSCRIBE_LIST = [
        SUB_MCU_SENSORS,
        SUB_PLC,
        SUB_SERVER,
        SUB_NAV2,
    ]
    
    # ======================== 발행 (로봇 → 외부) ========================
    # 로봇 상태
    SLAM_MODE = "slam_mode"                     # 로봇 모드
    
    # Nav2 상태
    NAV_STATUS = "robot/nav_status"             # 진행 상태
    NAV_RESULT = "robot/nav_result"             # 최종 결과
    NAV_ARRIVED = "robot/arrived"               # 도착 알림
    NAVIGATE_TO_POSE = "robot/navigate_to_pose" # Goal 요청 모니터링
    
    # SLAM/맵
    MAP_CYCLE_COMPLETE = "ros/map_cycle_complete"   # 맵 저장 사이클
    COLLISION_PHOTO = "collision/photo_ready"       # 충돌 사진
    
    # 맵 다운로드 (PC → 로봇)
    MAP_READY = "robot/map_ready"                   # PC에서 병합 맵 준비 완료
    MAP_LOADED = "robot/map_loaded"                 # 로봇에서 맵 로드 완료
    
    # 센서
    BATTERY_STATUS = "battery/status"           # 배터리
    
    # 장애물 (QR 마커로 인식)
    QR_OBSTACLE = "obstacle/qr_detected"        # QR 장애물 감지
    
    # ArUco (HOME + PORT)
    ARUCO_HOME = "aruco/home_detected"          # HOME 마커 감지
    ARUCO_PORT = "aruco/port_detected"          # PORT 마커 감지
    
    # AI Vision (서버 발행)
    AI_DETECTION = "mqtt/pinky/detection"       # 객체 감지
    AI_OBSTACLE_TYPE = "mqtt/pinky/obstacle_type"  # 장애물 유형


class ARUCO:
    """
    ArUco 마커 관련 토픽 정의
    
    마커 ID 규칙:
    - ID 0: PORT_A (SLAM 복귀 위치, HOME과 동일)
    - ID 1: PORT_B
    - ID 2: PORT_C
    - ID 3: PORT_D
    - ID 4: PORT_E
    
    용도:
    - SLAM 모드: PORT_A 마커로 출발/복귀 위치 인식 (HOME 역할)
    - Nav2 모드: PORT 마커로 도킹 위치 인식
    """
    
    # ======================== 마커 ID ========================
    PORT_A_ID = 0  # SLAM HOME 역할
    PORT_B_ID = 1
    PORT_C_ID = 2
    PORT_D_ID = 3
    PORT_E_ID = 4
    
    # HOME은 PORT_A와 동일 (SLAM 복귀용)
    HOME_ID = PORT_A_ID
    
    # 도킹용 마커 ID 세트 (PORT_A, PORT_B만)
    DOCK_MARKER_IDS = {PORT_A_ID, PORT_B_ID}
    
    # 마커 ID → 이름 매핑
    PORT_MAP = {
        0: "PORT_A",
        1: "PORT_B",
        2: "PORT_C",
        3: "PORT_D",
        4: "PORT_E",
    }
    
    # ======================== 도킹 파라미터 (ZONE 방식) ========================
    # size가 이 범위 안에 들어오면 도킹 완료
    DOCK_SIZE_MIN = 180       # 최소 (너무 멀면 전진)
    DOCK_SIZE_MAX = 350       # 최대 (너무 가까우면 후진)
    DOCK_SIZE_TARGET = 250    # 목표 (이 근처면 OK)
    DOCK_CENTER_TOLERANCE = 0.10  # 중앙 정렬 허용 오차 (normalized: -1~1)
    
    DOCK_SPEED_FORWARD = 0.08  # 전진 속도 (m/s)
    DOCK_SPEED_BACK = 0.05     # 후진 속도 (m/s)
    DOCK_ANGULAR_P = 0.6       # 회전 P 게인
    DOCK_STABLE_COUNT = 5      # N프레임 연속이면 완료
    
    # ======================== 도킹 토픽 ========================
    DOCK_ENABLE = "ros/aruco/dock_enable"       # 도킹 모드 활성화 (Bool)
    DOCK_STATUS = "ros/aruco/dock_status"       # 도킹 상태 (String/JSON)
    DOCK_COMPLETE = "ros/aruco/dock_complete"   # 도킹 완료 (Bool)
    LEARNING_MODE = "ros/aruco/learning_mode"   # 학습 모드 (Bool)
    
    # ======================== 포트 관리 ========================
    PORT_GOALS = "ros/aruco/port_goals"         # 저장된 포트 목록 (String/JSON)
    PORT_ARRIVAL = "ros/aruco/port_arrival"     # 포트 도착 (String/JSON)
    TARGET_PORT = "ros/aruco/target_port"       # 목표 포트 (String: PORT_A, PORT_B, ...)
    PORT_ODOM = "ros/aruco/port_odom"           # PORT 좌표 (odom 기준, String/JSON)


class DRIVE:
    """자율주행 관련 상수"""
    
    # ======================== 거리 임계값 (m) ========================
    FRONT_STOP = 0.20         # 정면 정지 거리
    FRONT_SLOW = 0.32         # 정면 감속 거리
    DIAGONAL_STOP = 0.15      # 사선 정지 거리
    DIAGONAL_SLOW = 0.26      # 사선 감속 거리
    SIDE_TOUCH = 0.11         # 측면 접촉 거리
    REAR_STOP = 0.15          # 후방 정지 거리
    US_EMERGENCY = 0.10       # 초음파 긴급 정지
    US_CRITICAL = 0.15        # 초음파 위험 거리
    
    # ======================== 속도 (m/s, rad/s) ========================
    SPEED_FAST = 0.13         # 고속 탐색
    SPEED_EXPLORE = 0.10      # 일반 탐색
    SPEED_SLOW = 0.07         # 저속
    SPEED_CORRIDOR = 0.08     # 통로 모드
    BACKUP_SPEED = -0.08      # 후진
    TURN_SPEED = 0.45         # 회전 속도
    STEER_SPEED = 0.20        # 조향 속도
    
    # ======================== 타이밍/기타 ========================
    STUCK_TIME = 3.0          # 갇힘 판정 시간 (초)
    STUCK_DIST = 0.10         # 갇힘 판정 거리 (m)
    CORRIDOR_WIDTH = 0.40     # 통로 판정 폭 (m)
    LOG_INTERVAL = 5.0        # 로그 간격 (초)
    CONTROL_PERIOD = 0.2      # 제어 주기 (초)
    
    # ======================== 연속 회전 방지 ========================
    MAX_CONSECUTIVE_TURNS = 3  # N번 연속 회전하면 후진


# ======================== 유틸리티 함수 ========================
def get_all_ros_topics() -> list:
    """모든 ROS2 토픽 리스트 반환"""
    return [v for k, v in vars(ROS).items() 
            if not k.startswith('_') and isinstance(v, str)]


def get_all_mqtt_pub_topics() -> list:
    """모든 MQTT 발행 토픽 리스트 반환"""
    return [
        MQTT.SLAM_MODE,
        MQTT.NAV_STATUS,
        MQTT.NAV_RESULT,
        MQTT.NAV_ARRIVED,
        MQTT.NAVIGATE_TO_POSE,
        MQTT.MAP_CYCLE_COMPLETE,
        MQTT.COLLISION_PHOTO,
        MQTT.QR_OBSTACLE,
        MQTT.ARUCO_HOME,
        MQTT.ARUCO_PORT,
        MQTT.BATTERY_STATUS,
    ]


def print_topic_summary():
    """토픽 요약 출력 (디버깅용)"""
    print("=" * 60)
    print("ROS2 Topics:")
    for k, v in vars(ROS).items():
        if not k.startswith('_') and isinstance(v, str):
            print(f"  {k:30} = {v}")
    
    print("\n" + "=" * 60)
    print("MQTT Topics (Publish):")
    for topic in get_all_mqtt_pub_topics():
        print(f"  {topic}")
    
    print("\nMQTT Topics (Subscribe):")
    for topic in MQTT.SUBSCRIBE_LIST:
        print(f"  {topic}")


if __name__ == "__main__":
    print_topic_summary()
