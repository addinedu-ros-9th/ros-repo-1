#!/bin/bash
#chmod +x setup_env.sh
# ROS2 Jazzy 환경 소스
source /opt/ros/jazzy/setup.bash

# 워크스페이스 환경 소스
source install/setup.bash

echo "✅ ROS2 Jazzy 환경 설정 완료"
echo "✅ 워크스페이스 환경 설정 완료"
echo "🚀 kiosk 애플리케이션을 실행하려면: python3 src/kiosk/main_window.py" 