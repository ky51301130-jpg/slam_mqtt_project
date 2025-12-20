#!/bin/bash
#
# 🛑 Pinky 로봇 프로세스 종료 스크립트
#
# 종료 대상:
# - ROS2 노드 (python3, component_container_isolated)
# - SLAM / Nav2 관련 프로세스
# - Flask 서버
#

echo "========================================"
echo "  🛑 Pinky Robot STOP ALL"
echo "========================================"

# 1. ROS2 노드 종료
echo "1. ROS2 노드 종료..."
killall -9 python3 2>/dev/null
killall -9 component_container_isolated 2>/dev/null
killall -9 ros2 2>/dev/null

# 2. Nav2 관련
echo "2. Nav2 프로세스 종료..."
killall -9 bt_navigator 2>/dev/null
killall -9 controller_server 2>/dev/null
killall -9 planner_server 2>/dev/null
killall -9 behavior_server 2>/dev/null
killall -9 amcl 2>/dev/null
killall -9 lifecycle_manager 2>/dev/null
killall -9 map_server 2>/dev/null

# 3. SLAM 관련
echo "3. SLAM 프로세스 종료..."
killall -9 sync_slam_toolbox_node 2>/dev/null
killall -9 async_slam_toolbox_node 2>/dev/null

# 4. 기타 ROS2 관련
echo "4. 기타 프로세스 종료..."
killall -9 robot_state_publisher 2>/dev/null
killall -9 joint_state_publisher 2>/dev/null
killall -9 sllidar_node 2>/dev/null
killall -9 rviz2 2>/dev/null

sleep 1

# 확인
remaining=$(pgrep -c -f "ros2|slam|nav2|component_container" 2>/dev/null || echo "0")
if [ "$remaining" = "0" ]; then
    echo ""
    echo "✅ 모든 프로세스 종료 완료!"
else
    echo ""
    echo "⚠️ 일부 프로세스가 남아있을 수 있음 (${remaining}개)"
    echo "   강제 종료: pkill -9 -f ros2"
fi

echo "========================================"
