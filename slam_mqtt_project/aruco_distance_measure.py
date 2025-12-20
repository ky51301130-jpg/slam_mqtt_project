#!/usr/bin/env python3
"""ArUco 마커 3D 거리 측정 도구

마커까지의 x, y, z 좌표를 측정합니다.
- x: 좌우 (오른쪽 +, 왼쪽 -)
- y: 상하 (아래 +, 위 -)
- z: 깊이/거리 (전방 +)
"""
import cv2
import numpy as np
import time
from picamera2 import Picamera2

# 캘리브레이션 로드
CALIB_FILE = "/home/pinky/pinky_test/camera_calibration.npz"
MARKER_SIZE = 0.10  # 10cm 마커

ARUCO_PORT_MAP = {0: "PORT_A", 1: "PORT_B", 2: "PORT_C", 3: "PORT_D", 4: "PORT_E"}


def main():
    print("=" * 60)
    print("🎯 ArUco 마커 3D 거리 측정기")
    print("=" * 60)
    
    # 카메라 캘리브레이션 로드
    try:
        calib = np.load(CALIB_FILE)
        camera_matrix = calib['camera_matrix']
        dist_coeffs = calib['distortion_coefficients']
        print(f"✅ 캘리브레이션 로드: {CALIB_FILE}")
    except Exception as e:
        print(f"❌ 캘리브레이션 로드 실패: {e}")
        print("기본값 사용...")
        camera_matrix = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1))
    
    # 카메라 초기화
    print("📷 카메라 초기화...")
    cam = Picamera2()
    config = cam.create_still_configuration(main={"format": "RGB888", "size": (640, 480)})
    cam.configure(config)
    cam.start()
    time.sleep(1)
    
    # ArUco 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    print(f"📐 마커 크기: {MARKER_SIZE*100:.1f}cm")
    print("=" * 60)
    print("마커를 카메라 앞에 두세요. Ctrl+C로 종료합니다.")
    print()
    
    last_print = 0
    measurements = []
    
    try:
        while True:
            frame = cam.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            corners, ids, _ = detector.detectMarkers(gray)
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    corner = corners[i][0]
                    
                    # 마커 크기 (픽셀)
                    width = np.linalg.norm(corner[0] - corner[1])
                    height = np.linalg.norm(corner[1] - corner[2])
                    marker_size_px = (width + height) / 2
                    
                    # 중심 좌표 (normalized)
                    center_x = np.mean(corner[:, 0])
                    center_y = np.mean(corner[:, 1])
                    normalized_x = (center_x - 320) / 320
                    normalized_y = (center_y - 240) / 240
                    
                    # 3D 좌표 계산 (solvePnP)
                    obj_points = np.array([
                        [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                        [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                        [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                        [-MARKER_SIZE/2,  MARKER_SIZE/2, 0]
                    ], dtype=np.float32)
                    
                    img_points = corner.astype(np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points, camera_matrix, dist_coeffs
                    )
                    
                    if success:
                        x, y, z = tvec.flatten()
                        name = ARUCO_PORT_MAP.get(marker_id, f"ID:{marker_id}")
                        
                        now = time.time()
                        if now - last_print > 0.3:  # 0.3초마다 출력
                            print(f"🎯 {name:8s} │ X={x*100:+7.2f}cm  Y={y*100:+7.2f}cm  Z={z*100:7.2f}cm │ Size={marker_size_px:5.1f}px │ Center=({normalized_x:+.2f}, {normalized_y:+.2f})")
                            last_print = now
                            
                            # 측정값 저장 (정렬되었을 때 - 필터 완화)
                            if abs(normalized_x) < 0.15:  # 중앙 근처면 저장
                                measurements.append({
                                    'name': name,
                                    'x': x, 'y': y, 'z': z,
                                    'size_px': marker_size_px
                                })
                                if len(measurements) >= 10:
                                    # 평균 계산
                                    avg_x = np.mean([m['x'] for m in measurements]) * 100
                                    avg_y = np.mean([m['y'] for m in measurements]) * 100
                                    avg_z = np.mean([m['z'] for m in measurements]) * 100
                                    avg_size = np.mean([m['size_px'] for m in measurements])
                                    print()
                                    print("=" * 60)
                                    print(f"📊 평균 측정값 ({len(measurements)} samples):")
                                    print(f"   X = {avg_x:+.2f} cm")
                                    print(f"   Y = {avg_y:+.2f} cm")
                                    print(f"   Z = {avg_z:.2f} cm (거리)")
                                    print(f"   Size = {avg_size:.1f} px")
                                    print("=" * 60)
                                    print()
                                    measurements.clear()
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print()
        print("종료됨.")
    finally:
        cam.stop()


if __name__ == '__main__':
    main()
