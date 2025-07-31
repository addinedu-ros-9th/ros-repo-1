# ESP32 무게 센서 micro-ROS 사용법

## 개요
ESP32와 HX711 로드셀을 사용하여 무게를 측정하고, micro-ROS를 통해 ROS2 토픽으로 데이터를 퍼블리시하는 시스템입니다.

## 하드웨어 구성
- ESP32 개발보드
- HX711 로드셀 앰프
- 로드셀 (스테인리스 스틸)
- 연결:
  - ESP32 GPIO4 → HX711 DOUT
  - ESP32 GPIO18 → HX711 SCK
  - ESP32 3.3V → HX711 VCC
  - ESP32 GND → HX711 GND

## 소프트웨어 요구사항

### Arduino IDE 설정
1. **Arduino IDE 설치** (최신 버전)
2. **ESP32 보드 매니저 추가**:
   - Arduino IDE → 파일 → 환경설정
   - 추가 보드 매니저 URL에 추가:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
3. **ESP32 보드 설치**:
   - 도구 → 보드 → ESP32 Arduino → ESP32 Dev Module
4. **필요한 라이브러리 설치**:
   - HX711_ADC 라이브러리
   - micro_ros_arduino 라이브러리

### micro-ROS 설정
1. **micro-ROS Agent 설치**:
   ```bash
   # Ubuntu 24.04에서
   sudo apt update
   sudo apt install ros-jazzy-micro-ros-agent
   ```

2. **micro-ROS Agent 실행**:
   ```bash
   # ROS Domain ID 26으로 실행
   export ROS_DOMAIN_ID=26
   micro_ros_agent udp4 --port 8888
   ```

## 사용법

### 1. ESP32 코드 업로드
1. Arduino IDE에서 `weight_sensor_microros.ino` 파일 열기
2. 보드 설정:
   - 보드: ESP32 Dev Module
   - 업로드 속도: 115200
   - 포트: ESP32 연결된 포트 선택
3. 코드 업로드

### 2. 캘리브레이션 (처음 사용 시)
1. 캘리브레이션 코드 업로드 (기존 `calibration_esp32.ino` 사용)
2. 시리얼 모니터에서 캘리브레이션 진행
3. 캘리브레이션 완료 후 micro-ROS 코드로 다시 업로드

### 3. ROS2 환경 설정
```bash
# 터미널에서 jazzy 명령어로 ROS2 환경 설정
jazzy

# 또는 수동으로 설정
export ROS_DOMAIN_ID=26
source /opt/ros/jazzy/setup.bash
```

### 4. 무게 센서 수신 노드 실행
```bash
# 워크스페이스 빌드
cd ros_ws
colcon build --packages-select main_server

# 소스 설정
source install/setup.bash

# 무게 센서 수신 노드 실행
ros2 launch main_server weight_sensor.launch.py
```

### 5. 토픽 확인
```bash
# 토픽 리스트 확인
ros2 topic list

# 무게 데이터 확인
ros2 topic echo /weight_data

# 토픽 정보 확인
ros2 topic info /weight_data
```

## 시리얼 명령어 (ESP32)
- `t`: 영점 조정 (Tare)
- `r`: 캘리브레이션 모드 전환 안내
- `c`: 캘리브레이션 값 수동 변경

## 네트워크 설정
- ESP32와 컴퓨터가 같은 WiFi 네트워크에 연결되어야 함
- ESP32의 IP 주소를 확인하고 micro-ROS Agent에서 올바른 주소로 연결

## 문제 해결

### 1. 연결 문제
- ESP32와 컴퓨터가 같은 네트워크에 있는지 확인
- 방화벽 설정 확인
- micro-ROS Agent가 실행 중인지 확인

### 2. 데이터 수신 안됨
- ROS Domain ID가 26으로 설정되었는지 확인
- 토픽 이름이 `/weight_data`인지 확인
- ESP32 시리얼 모니터에서 연결 상태 확인

### 3. 무게 측정 오류
- HX711 연결 확인
- 캘리브레이션 값 확인
- 로드셀 상태 확인

## 토픽 정보
- **토픽 이름**: `/weight_data`
- **메시지 타입**: `std_msgs/msg/Float32`
- **단위**: 그램 (g)
- **퍼블리시 주기**: 1초

## 코드 구조
- `weight_sensor_microros.ino`: ESP32 micro-ROS 코드
- `weight_subscriber.py`: ROS2 수신 노드
- `weight_sensor.launch.py`: Launch 파일

## 라이센스
이 프로젝트는 MIT 라이센스 하에 배포됩니다. 