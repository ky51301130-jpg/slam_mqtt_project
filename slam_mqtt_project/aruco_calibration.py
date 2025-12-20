#!/usr/bin/env python3
"""
=============================================================================
                    ArUco 캘리브레이션 & 도킹 테스트 도구
=============================================================================
카메라 캘리브레이션 + 도킹 거리 조정을 위한 대화형 도구

기능:
1. 카메라 캘리브레이션 (체커보드 or ArUco 보드)
2. ArUco 마커 거리 측정 테스트
3. 도킹 거리/속도 파라미터 조정
4. 설정 저장 (JSON)

사용법:
    ros2 run slam_mqtt_project aruco_calibration
    
    또는 직접 실행:
    python3 aruco_calibration.py
=============================================================================
"""

import os
# Qt/OpenCV GUI 비활성화 (헤드리스 환경)
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import time
from datetime import datetime
from typing import Optional, Tuple, Dict

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    print("⚠️ Picamera2 not available, using USB camera fallback")

# ============== 설정 파일 경로 ==============
CONFIG_DIR = "/home/pinky/saved_maps"
CALIBRATION_FILE = f"{CONFIG_DIR}/aruco_calibration.json"
DEFAULT_MARKER_SIZE = 0.10  # 기본 마커 크기 10cm


class ArucoCalibration:
    """ArUco 캘리브레이션 및 테스트 도구"""
    
    def __init__(self):
        self.camera = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = DEFAULT_MARKER_SIZE
        
        # 도킹 파라미터
        self.dock_distance = 0.30  # 목표 도킹 거리 (m)
        self.dock_tolerance = 0.05  # 허용 오차 (m)
        self.center_tolerance = 0.05  # 중앙 정렬 허용 오차 (m)
        self.angle_tolerance = 5.0  # 각도 허용 오차 (도)
        
        # ArUco 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # 기존 설정 로드
        self._load_config()
        
        # 카메라 초기화
        self._init_camera()
    
    def _init_camera(self):
        """카메라 초기화"""
        if PICAMERA_AVAILABLE:
            try:
                self.camera = Picamera2()
                config = self.camera.create_preview_configuration(
                    main={"size": (640, 480), "format": "RGB888"}
                )
                self.camera.configure(config)
                self.camera.start()
                print("📷 Picamera2 initialized (640x480)")
                time.sleep(1)  # 카메라 안정화
            except Exception as e:
                print(f"❌ Picamera2 failed: {e}")
                self.camera = None
        else:
            # USB 카메라 fallback
            self.camera = cv2.VideoCapture(0)
            if self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                print("📷 USB camera initialized (640x480)")
            else:
                print("❌ No camera available")
                self.camera = None
    
    def _load_config(self):
        """저장된 설정 로드"""
        try:
            if os.path.exists(CALIBRATION_FILE):
                with open(CALIBRATION_FILE, 'r') as f:
                    config = json.load(f)
                
                # 카메라 캘리브레이션
                if 'camera_matrix' in config:
                    self.camera_matrix = np.array(config['camera_matrix'], dtype=np.float32)
                if 'dist_coeffs' in config:
                    self.dist_coeffs = np.array(config['dist_coeffs'], dtype=np.float32)
                
                # 마커/도킹 파라미터
                self.marker_size = config.get('marker_size', DEFAULT_MARKER_SIZE)
                self.dock_distance = config.get('dock_distance', 0.30)
                self.dock_tolerance = config.get('dock_tolerance', 0.05)
                self.center_tolerance = config.get('center_tolerance', 0.05)
                self.angle_tolerance = config.get('angle_tolerance', 5.0)
                
                print(f"📂 Loaded config from {CALIBRATION_FILE}")
                return True
        except Exception as e:
            print(f"⚠️ Config load failed: {e}")
        
        # 기본 캘리브레이션 사용
        self._set_default_calibration()
        return False
    
    def _set_default_calibration(self):
        """기본 캘리브레이션 값 설정"""
        # Raspberry Pi Camera v2 640x480 기본값
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5,), dtype=np.float32)
        print("📐 Using default calibration (640x480)")
    
    def save_config(self):
        """설정 저장"""
        try:
            os.makedirs(CONFIG_DIR, exist_ok=True)
            
            config = {
                'camera_matrix': self.camera_matrix.tolist() if self.camera_matrix is not None else None,
                'dist_coeffs': self.dist_coeffs.tolist() if self.dist_coeffs is not None else None,
                'marker_size': self.marker_size,
                'dock_distance': self.dock_distance,
                'dock_tolerance': self.dock_tolerance,
                'center_tolerance': self.center_tolerance,
                'angle_tolerance': self.angle_tolerance,
                'saved_at': datetime.now().isoformat()
            }
            
            with open(CALIBRATION_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            
            print(f"💾 Config saved to {CALIBRATION_FILE}")
            return True
        except Exception as e:
            print(f"❌ Save failed: {e}")
            return False
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """프레임 캡처"""
        if self.camera is None:
            return None
        
        if PICAMERA_AVAILABLE and isinstance(self.camera, Picamera2):
            try:
                return self.camera.capture_array()
            except:
                return None
        else:
            ret, frame = self.camera.read()
            if ret:
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return None
    
    def detect_marker(self, frame: np.ndarray) -> Optional[Dict]:
        """
        ArUco 마커 감지 및 거리/각도 계산
        
        Returns:
            {
                'id': marker_id,
                'distance': float (m),
                'x_offset': float (m),
                'y_offset': float (m),
                'yaw': float (degrees),
                'corners': array
            }
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if ids is None or len(ids) == 0:
            return None
        
        # 첫 번째 마커 사용
        marker_id = ids[0][0]
        marker_corners = corners[0]
        
        # 3D 위치 추정
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            [marker_corners], self.marker_size,
            self.camera_matrix, self.dist_coeffs
        )
        
        x_offset = tvecs[0][0][0]  # 좌우 (양수=오른쪽)
        y_offset = tvecs[0][0][1]  # 상하
        distance = tvecs[0][0][2]  # 정면 거리
        
        # 각도 계산
        rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
        yaw_rad = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        yaw_deg = np.degrees(yaw_rad)
        
        return {
            'id': int(marker_id),
            'distance': float(distance),
            'x_offset': float(x_offset),
            'y_offset': float(y_offset),
            'yaw': float(yaw_deg),
            'corners': marker_corners
        }
    
    def run_distance_test(self):
        """거리 측정 테스트 (터미널 기반)"""
        print("\n" + "="*60)
        print("        ArUco 거리 측정 테스트 (터미널)")
        print("="*60)
        print(f"마커 크기: {self.marker_size*100:.1f}cm")
        print(f"목표 도킹 거리: {self.dock_distance*100:.1f}cm")
        print(f"허용 오차: 거리 ±{self.dock_tolerance*100:.1f}cm, 중앙 ±{self.center_tolerance*100:.1f}cm, 각도 ±{self.angle_tolerance:.1f}°")
        print("-"*60)
        print("마커를 카메라 앞에 두고 거리를 확인하세요.")
        print("Ctrl+C = 종료")
        print("="*60 + "\n")
        
        last_save_dist = None
        try:
            while True:
                frame = self.capture_frame()
                if frame is None:
                    print("\r❌ 카메라 프레임 없음", end='')
                    time.sleep(0.5)
                    continue
                
                # 마커 감지
                result = self.detect_marker(frame)
                
                if result:
                    dist = result['distance']
                    x_off = result['x_offset']
                    yaw = result['yaw']
                    
                    # 도킹 상태 판정
                    dist_ok = abs(dist - self.dock_distance) < self.dock_tolerance
                    center_ok = abs(x_off) < self.center_tolerance
                    angle_ok = abs(yaw) < self.angle_tolerance
                    
                    # 상태 표시
                    d_sym = "✅" if dist_ok else "❌"
                    c_sym = "✅" if center_ok else "❌"
                    a_sym = "✅" if angle_ok else "❌"
                    dock_status = "🎯 DOCK OK!" if (dist_ok and center_ok and angle_ok) else ""
                    
                    print(f"\rID:{result['id']:2d} | 거리:{dist*100:5.1f}cm {d_sym} | X:{x_off*100:+5.1f}cm {c_sym} | 각도:{yaw:+5.1f}° {a_sym} {dock_status}   ", end='', flush=True)
                    last_save_dist = dist
                else:
                    print("\r⏳ 마커 감지 중...                                                           ", end='', flush=True)
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\n중단됨")
            if last_save_dist:
                save = input(f"마지막 거리 {last_save_dist*100:.1f}cm를 도킹 거리로 저장? (y/n): ").strip().lower()
                if save == 'y':
                    self.dock_distance = last_save_dist
                    self.save_config()
                    print(f"💾 도킹 거리 저장: {self.dock_distance*100:.1f}cm")
    
    def run_parameter_setup(self):
        """파라미터 설정 메뉴"""
        while True:
            print("\n" + "="*60)
            print("        도킹 파라미터 설정")
            print("="*60)
            print(f"  1. 마커 크기:        {self.marker_size*100:.1f} cm")
            print(f"  2. 도킹 거리:        {self.dock_distance*100:.1f} cm")
            print(f"  3. 거리 허용오차:    ±{self.dock_tolerance*100:.1f} cm")
            print(f"  4. 중앙 허용오차:    ±{self.center_tolerance*100:.1f} cm")
            print(f"  5. 각도 허용오차:    ±{self.angle_tolerance:.1f}°")
            print("-"*60)
            print("  s. 저장")
            print("  q. 뒤로")
            print("="*60)
            
            choice = input("선택: ").strip().lower()
            
            if choice == '1':
                try:
                    val = float(input(f"마커 크기 (cm) [{self.marker_size*100:.1f}]: ") or self.marker_size*100)
                    self.marker_size = val / 100
                    print(f"✅ 마커 크기: {self.marker_size*100:.1f}cm")
                except ValueError:
                    print("❌ 숫자를 입력하세요")
            
            elif choice == '2':
                try:
                    val = float(input(f"도킹 거리 (cm) [{self.dock_distance*100:.1f}]: ") or self.dock_distance*100)
                    self.dock_distance = val / 100
                    print(f"✅ 도킹 거리: {self.dock_distance*100:.1f}cm")
                except ValueError:
                    print("❌ 숫자를 입력하세요")
            
            elif choice == '3':
                try:
                    val = float(input(f"거리 허용오차 (cm) [{self.dock_tolerance*100:.1f}]: ") or self.dock_tolerance*100)
                    self.dock_tolerance = val / 100
                    print(f"✅ 거리 허용오차: ±{self.dock_tolerance*100:.1f}cm")
                except ValueError:
                    print("❌ 숫자를 입력하세요")
            
            elif choice == '4':
                try:
                    val = float(input(f"중앙 허용오차 (cm) [{self.center_tolerance*100:.1f}]: ") or self.center_tolerance*100)
                    self.center_tolerance = val / 100
                    print(f"✅ 중앙 허용오차: ±{self.center_tolerance*100:.1f}cm")
                except ValueError:
                    print("❌ 숫자를 입력하세요")
            
            elif choice == '5':
                try:
                    val = float(input(f"각도 허용오차 (°) [{self.angle_tolerance:.1f}]: ") or self.angle_tolerance)
                    self.angle_tolerance = val
                    print(f"✅ 각도 허용오차: ±{self.angle_tolerance:.1f}°")
                except ValueError:
                    print("❌ 숫자를 입력하세요")
            
            elif choice == 's':
                self.save_config()
            
            elif choice == 'q':
                break
    
    def run_camera_calibration(self):
        """카메라 캘리브레이션 (체커보드) - 터미널 기반"""
        print("\n" + "="*60)
        print("        카메라 캘리브레이션 (터미널)")
        print("="*60)
        print("9x6 체커보드를 다양한 각도로 촬영합니다.")
        print("최소 10장 이상 촬영 권장")
        print("-"*60)
        print("Enter = 캡처, c = 캘리브레이션 실행, q = 취소")
        print("="*60 + "\n")
        
        # 체커보드 설정
        CHECKERBOARD = (9, 6)  # 내부 코너 수
        SQUARE_SIZE = 0.025  # 25mm 정사각형
        
        # 3D 포인트 준비
        objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        objp *= SQUARE_SIZE
        
        objpoints = []  # 3D 포인트
        imgpoints = []  # 2D 포인트
        gray_shape = None
        
        while True:
            cmd = input(f"\n[{len(objpoints)}장 캡처됨] Enter=캡처, c=실행, q=취소: ").strip().lower()
            
            if cmd == 'q':
                break
            
            elif cmd == 'c':
                if len(objpoints) < 5:
                    print("❌ 최소 5장 필요합니다")
                    continue
                
                print("\n⏳ 캘리브레이션 계산 중...")
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                    objpoints, imgpoints, gray_shape, None, None
                )
                
                if ret:
                    self.camera_matrix = mtx
                    self.dist_coeffs = dist.flatten()
                    
                    # 재투영 오차 계산
                    total_error = 0
                    for i in range(len(objpoints)):
                        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                        total_error += error
                    mean_error = total_error / len(objpoints)
                    
                    print(f"\n✅ 캘리브레이션 완료!")
                    print(f"   재투영 오차: {mean_error:.4f}")
                    print(f"   Camera Matrix:\n{mtx}")
                    
                    save = input("\n저장하시겠습니까? (y/n): ").strip().lower()
                    if save == 'y':
                        self.save_config()
                    break
                else:
                    print("❌ 캘리브레이션 실패")
            
            else:  # Enter - 캡처
                frame = self.capture_frame()
                if frame is None:
                    print("❌ 카메라 프레임 없음")
                    continue
                
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                gray_shape = gray.shape[::-1]
                
                # 체커보드 찾기
                ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
                
                if ret:
                    # 코너 정밀화
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    objpoints.append(objp)
                    imgpoints.append(corners2)
                    print(f"📸 캡처 성공! ({len(objpoints)}장)")
                else:
                    print("❌ 체커보드를 찾을 수 없습니다. 다시 시도하세요.")
    
    def print_aruco_markers(self):
        """ArUco 마커 생성 및 출력"""
        print("\n" + "="*60)
        print("        ArUco 마커 생성")
        print("="*60)
        
        marker_ids = [0, 1]  # PORT_A, PORT_B
        marker_names = {0: "PORT_A", 1: "PORT_B"}
        
        for marker_id in marker_ids:
            marker_img = aruco.generateImageMarker(self.aruco_dict, marker_id, 200)
            filename = f"{CONFIG_DIR}/aruco_marker_{marker_id}_{marker_names[marker_id]}.png"
            cv2.imwrite(filename, marker_img)
            print(f"✅ 저장: {filename}")
        
        print(f"\n마커를 {self.marker_size*100:.0f}cm 크기로 인쇄하세요!")
        print("="*60 + "\n")
    
    def run_menu(self):
        """메인 메뉴"""
        while True:
            print("\n" + "="*60)
            print("     🎯 ArUco 캘리브레이션 & 도킹 테스트 도구")
            print("="*60)
            print("  1. 거리 측정 테스트 (실시간)")
            print("  2. 도킹 파라미터 설정")
            print("  3. 카메라 캘리브레이션 (체커보드)")
            print("  4. ArUco 마커 이미지 생성")
            print("  5. 현재 설정 보기")
            print("-"*60)
            print("  q. 종료")
            print("="*60)
            
            choice = input("선택: ").strip().lower()
            
            if choice == '1':
                self.run_distance_test()
            elif choice == '2':
                self.run_parameter_setup()
            elif choice == '3':
                self.run_camera_calibration()
            elif choice == '4':
                self.print_aruco_markers()
            elif choice == '5':
                self._print_current_config()
            elif choice == 'q':
                break
        
        self.cleanup()
    
    def _print_current_config(self):
        """현재 설정 출력"""
        print("\n" + "="*60)
        print("        현재 설정")
        print("="*60)
        print(f"마커 크기:        {self.marker_size*100:.1f} cm")
        print(f"도킹 거리:        {self.dock_distance*100:.1f} cm")
        print(f"거리 허용오차:    ±{self.dock_tolerance*100:.1f} cm")
        print(f"중앙 허용오차:    ±{self.center_tolerance*100:.1f} cm")
        print(f"각도 허용오차:    ±{self.angle_tolerance:.1f}°")
        print("-"*60)
        if self.camera_matrix is not None:
            print("Camera Matrix:")
            print(self.camera_matrix)
        print("="*60)
    
    def cleanup(self):
        """정리"""
        if self.camera is not None:
            if PICAMERA_AVAILABLE and isinstance(self.camera, Picamera2):
                self.camera.stop()
            else:
                self.camera.release()
        cv2.destroyAllWindows()
        print("👋 종료")


def main():
    """ROS2 엔트리포인트"""
    try:
        tool = ArucoCalibration()
        tool.run_menu()
    except KeyboardInterrupt:
        print("\n중단됨")
    except Exception as e:
        print(f"오류: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
