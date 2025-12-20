#!/bin/bash
# ROS2 Jazzy 설정 스크립트
# 사용법: source ~/.jazzy_setting.sh

ID=5

# alias setting
alias sb="source ~/.bashrc; echo \"Bashrc is reloaded!\""
alias ros_domain="export ROS_DOMAIN_ID=\$ID"
alias jazzy="ros_domain; source /opt/ros/jazzy/setup.bash; echo \"Jazzy (ROS_DOMAIN_ID=\$ID) is activated!\""
alias ros2pkg_enable="jazzy; source ~/ros2_ws/install/local_setup.bash; \
echo \"Jazzy (ROS_DOMAIN_ID=\$ID) and Workspace(~/ros2_ws) is activated!!\""
