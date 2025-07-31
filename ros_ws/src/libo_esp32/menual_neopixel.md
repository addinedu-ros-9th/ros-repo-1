# micro-ROS 네오픽셀 LED 제어 시스템

이 시스템은 ESP32와 micro-ROS를 사용하여 네오픽셀 LED를 ROS2 토픽으로 제어하는 시스템입니다.

## 시스템 구성

- **ESP32**: 네오픽셀 LED 제어 및 micro-ROS 구독자
- **ROS2 Jazzy**: LED 상태 발행자
- **micro-ROS Agent**: ESP32와 ROS2 간 통신 중계

## 상태별 LED 색상

- **기쁨**: 초록색 (0, 255, 0)
- **슬픔**: 초록색 (0, 255, 0)  
- **화남**: 빨간색 (255, 0, 0)

## 실행 방법

### 1. 환경 설정
```bash
# ROS2 Jazzy 환경 설정 (도메인 ID 26 자동 설정)
jazzy
```

### 2. main_server 빌드
```bash
cd ros_ws
colcon build --packages-select main_server
source install/setup.bash
```

### 3. micro-ROS Agent 생성 및 빌드
```bash
# agent 작업공간 생성
ros2 run micro_ros_setup create_agent_ws.sh

# agent 빌드
ros2 run micro_ros_setup build_agent.sh

# 환경 소스 (중요!)
source install/local_setup.bash
```

### 4. ESP32 코드 업로드
1. Arduino IDE에서 `neopixel_microros.ino` 파일을 열기
2. ESP32 보드 설정 확인
3. micro_ros_arduino 라이브러리 설치
4. FastLED 라이브러리 설치
5. ESP32에 코드 업로드

### 5. Agent 실행
```bash
# UDP 연결 (ESP32 IP 주소 확인 필요)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### 6. LED 상태 발행자 실행
```bash
# 새 터미널에서
jazzy
cd ros_ws
source install/setup.bash
ros2 run main_server led_status_publisher
```

### 7. 토픽 확인
```bash
# 새 터미널에서
jazzy
ros2 topic list
ros2 topic echo /led_status
```

## 문제 해결

### ESP32 연결 문제
- ESP32 IP 주소 확인
- 네트워크 연결 상태 확인
- Agent 포트 설정 확인

### LED가 반응하지 않는 경우
- ESP32 시리얼 모니터로 메시지 수신 확인
- 토픽 발행 상태 확인
- LED 핀 연결 확인

## 파일 구조

```
ros_ws/
├── src/
│   ├── main_server/
│   │   └── main_server/
│   │       └── led_status_publisher.py
│   └── libo_esp32/
│       └── neopixel_microros/
│           └── neopixel_microros.ino
```

## 참고사항

- ESP32의 IP 주소는 네트워크 설정에 따라 다를 수 있습니다
- LED 개수는 실제 하드웨어에 맞게 조정하세요
- 도메인 ID는 26으로 설정되어 있습니다 