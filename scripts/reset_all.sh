#!/bin/bash
#
# 🔄 Pinky 로봇 전체 초기화 스크립트
#
# 초기화 항목:
# - HOME 설정 파일들
# - PORT 좌표 파일
# - 저장된 맵 파일들
# - 충돌 사진들
# - ArUco 관련 파일들
#

echo "========================================"
echo "  🔄 Pinky Robot RESET ALL"
echo "========================================"
echo ""

# 프로세스 종료
echo "1. 실행 중인 노드 종료..."
killall -9 python3 component_container_isolated ros2 2>/dev/null
sleep 1

# HOME 설정 파일 삭제
echo "2. HOME 설정 초기화..."
rm -f ~/.ros_home_config.json
rm -f ~/.pinky_home_pose.json
rm -f ~/.aruco_home.json
echo "   ✓ HOME 파일 삭제됨"

# PORT 좌표 파일 삭제
echo "3. PORT 좌표 초기화..."
rm -f /home/pinky/saved_maps/port_goals.json
echo "   ✓ port_goals.json 삭제됨"

# 맵 파일 삭제 (선택적)
echo ""
read -p "4. 저장된 맵 파일도 삭제할까요? (y/N): " del_maps
if [[ "$del_maps" == "y" || "$del_maps" == "Y" ]]; then
    rm -f /home/pinky/saved_maps/map_*.pgm
    rm -f /home/pinky/saved_maps/map_*.yaml
    rm -f /home/pinky/saved_maps/map_*.png
    echo "   ✓ 맵 파일 삭제됨"
else
    echo "   - 맵 파일 유지"
fi

# 충돌 사진 삭제 (선택적)
echo ""
read -p "5. 충돌 사진도 삭제할까요? (y/N): " del_photos
if [[ "$del_photos" == "y" || "$del_photos" == "Y" ]]; then
    rm -f /home/pinky/collision_photos/*.jpg
    echo "   ✓ 충돌 사진 삭제됨"
else
    echo "   - 충돌 사진 유지"
fi

# ROS2 로그 삭제 (선택적)
echo ""
read -p "6. ROS2 로그도 삭제할까요? (y/N): " del_logs
if [[ "$del_logs" == "y" || "$del_logs" == "Y" ]]; then
    rm -rf ~/.ros/log/*
    echo "   ✓ ROS2 로그 삭제됨"
else
    echo "   - ROS2 로그 유지"
fi

echo ""
echo "========================================"
echo "  ✅ 초기화 완료!"
echo "========================================"
echo ""
echo "다음 단계:"
echo "  1. SLAM 모드 시작: ros2 launch slam_mqtt_project slam_exploration.launch.py"
echo "  2. ArUco 마커 앞에서 HOME/PORT 좌표 자동 저장"
echo "  3. Nav2 모드 시작: ros2 launch slam_mqtt_project nav2_mode.launch.py"
echo ""
