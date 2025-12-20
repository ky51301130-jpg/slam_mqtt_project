#!/bin/bash
#==============================================================================
# SLAM 완전 초기화 스크립트
# 모든 ROS2/SLAM 관련 프로세스를 종료하고 깨끗하게 재시작
#==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}     SLAM 완전 초기화 스크립트${NC}"
echo -e "${BLUE}========================================${NC}"

#------------------------------------------------------------------------------
# 1. ROS2 관련 프로세스 종료
#------------------------------------------------------------------------------
echo -e "\n${YELLOW}[1/5] ROS2 노드 종료 중...${NC}"

# slam_toolbox 관련
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "async_slam_toolbox" 2>/dev/null || true
pkill -9 -f "sync_slam_toolbox" 2>/dev/null || true

# Nav2 관련
pkill -9 -f "nav2" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "amcl" 2>/dev/null || true
pkill -9 -f "map_server" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true

# 로봇 드라이버
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "ld19" 2>/dev/null || true
pkill -9 -f "ldlidar" 2>/dev/null || true
pkill -9 -f "rplidar" 2>/dev/null || true
pkill -9 -f "sllidar" 2>/dev/null || true

# slam_mqtt_project 노드들
pkill -9 -f "mqtt_bridge" 2>/dev/null || true
pkill -9 -f "auto_drive" 2>/dev/null || true
pkill -9 -f "ultrasonic" 2>/dev/null || true
pkill -9 -f "map_saver" 2>/dev/null || true
pkill -9 -f "led_controller" 2>/dev/null || true
pkill -9 -f "collision_photo" 2>/dev/null || true
pkill -9 -f "nav2_goal" 2>/dev/null || true
pkill -9 -f "lcd_status" 2>/dev/null || true

echo -e "${GREEN}  ✓ 프로세스 종료 완료${NC}"

#------------------------------------------------------------------------------
# 2. ROS2 Daemon 재시작
#------------------------------------------------------------------------------
echo -e "\n${YELLOW}[2/5] ROS2 Daemon 재시작...${NC}"
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true
echo -e "${GREEN}  ✓ Daemon 재시작 완료${NC}"

#------------------------------------------------------------------------------
# 3. 공유 메모리 정리
#------------------------------------------------------------------------------
echo -e "\n${YELLOW}[3/5] 공유 메모리 정리...${NC}"
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
rm -rf /dev/shm/*ros* 2>/dev/null || true
echo -e "${GREEN}  ✓ 공유 메모리 정리 완료${NC}"

#------------------------------------------------------------------------------
# 4. 환경 확인
#------------------------------------------------------------------------------
echo -e "\n${YELLOW}[4/5] ROS2 환경 확인...${NC}"
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || source ~/pinky_pro/install/setup.bash 2>/dev/null || true
echo -e "${GREEN}  ✓ 환경 로드 완료${NC}"

#------------------------------------------------------------------------------
# 5. 상태 확인
#------------------------------------------------------------------------------
echo -e "\n${YELLOW}[5/5] 상태 확인...${NC}"
sleep 2

REMAINING=$(ps aux | grep -E "(slam|nav2|mqtt_bridge|auto_drive)" | grep -v grep | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo -e "${GREEN}  ✓ 모든 프로세스 정리 완료${NC}"
else
    echo -e "${RED}  ! 일부 프로세스가 남아있음 (수동 확인 필요)${NC}"
    ps aux | grep -E "(slam|nav2|mqtt_bridge|auto_drive)" | grep -v grep
fi

echo -e "\n${BLUE}========================================${NC}"
echo -e "${GREEN}초기화 완료! 이제 SLAM을 시작할 수 있습니다.${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "실행 명령어:"
echo -e "  ${YELLOW}ros2 launch slam_mqtt_project slam_exploration.launch.py${NC}"
echo ""
