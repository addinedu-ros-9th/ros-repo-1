/*
   ESP32 + HX711 로드셀 + micro-ROS 무게 측정 코드 (WiFi 버전 - 수정)
   무게 측정 값을 ROS2 토픽으로 퍼블리시
*/

#include <micro_ros_arduino.h>
#include <HX711_ADC.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <EEPROM.h>
#include <WiFi.h>

// WiFi 설정
char ssid[] = "AIE_509_2.4G";
char password[] = "addinedu_class1";

// PC IP 주소 (실제 PC IP로 변경 필요)
char agent_ip[] = "192.168.0.76";  // WiFi IP 사용
size_t agent_port = 8888;

// 핀 설정
const int HX711_dout = 4;
const int HX711_sck = 18;

#ifdef LED_BUILTIN
const int LED_PIN = LED_BUILTIN;
#else
const int LED_PIN = 2;
#endif

// HX711 객체 생성
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// micro-ROS 관련 변수
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Float32 weight_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;

// EEPROM 캘리브레이션 값 주소
const int calVal_eepromAdress = 0;

// 무게 측정 관련 변수
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 1000;

// 연결 상태
bool wifi_connected = false;
bool microros_connected = false;

// micro-ROS 에러 핸들러
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// WiFi 연결 함수
void connectWiFi() {
  Serial.print("📡 WiFi 연결 중... SSID: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println();
    Serial.println("✅ WiFi 연결 성공!");
    Serial.print("📶 IP 주소: ");
    Serial.println(WiFi.localIP());
    Serial.print("🎯 Agent IP: ");
    Serial.println(agent_ip);
    Serial.print("🔌 Agent Port: ");
    Serial.println(agent_port);
  } else {
    wifi_connected = false;
    Serial.println();
    Serial.println("❌ WiFi 연결 실패!");
  }
}

// micro-ROS 연결 시도
bool connectMicroROS() {
  Serial.println("🔧 micro-ROS 연결 시도 중...");
  
  // micro-ROS WiFi transport 설정
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  
  allocator = rcl_get_default_allocator();
  
  // init options 설정 (도메인 ID 명시)
  init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ init options 초기화 실패");
    return false;
  }
  
  // 도메인 ID 설정
  ret = rcl_init_options_set_domain_id(&init_options, 26);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ 도메인 ID 설정 실패");
    return false;
  }
  
  // micro-ROS 지원 초기화 (init_options 사용)
  ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS support 초기화 실패");
    return false;
  }
  
  // 노드 생성
  ret = rclc_node_init_default(&node, "esp32_weight_sensor", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 노드 생성 실패");
    return false;
  }
  
  // 퍼블리셔 생성
  ret = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "weight_data");
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 퍼블리셔 생성 실패");
    return false;
  }

  // 실행자 생성
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 실행자 생성 실패");
    return false;
  }
  
  Serial.println("✅ micro-ROS 연결 성공!");
  Serial.println("🌐 도메인 ID: 26 설정됨");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 무게 센서 micro-ROS (WiFi) 시작 ===");

  // LED 핀 설정
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // WiFi 연결
  connectWiFi();
  
  if (!wifi_connected) {
    Serial.println("❌ WiFi 연결 실패로 인해 종료합니다.");
    while(1);
  }

  // HX711 초기화
  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("❌ 오류: HX711 연결을 확인하세요!");
    while (1);
  }

  // EEPROM에서 캘리브레이션 값 불러오기
  EEPROM.begin(512);
  float calibrationValue;
  EEPROM.get(calVal_eepromAdress, calibrationValue);
  
  if (calibrationValue == 0 || isnan(calibrationValue)) {
    Serial.println("⚠️  경고: EEPROM에 캘리브레이션 값이 없습니다!");
    calibrationValue = 1.0;
  } else {
    Serial.print("✅ EEPROM에서 캘리브레이션 값 로드: ");
    Serial.println(calibrationValue);
  }
  
  LoadCell.setCalFactor(calibrationValue);
  Serial.println("✅ 로드셀 준비 완료!");

  // micro-ROS 연결 시도
  microros_connected = connectMicroROS();
  
  if (microros_connected) {
    Serial.println("📡 무게 데이터를 /weight_data 토픽으로 퍼블리시합니다.");
    Serial.println("🌐 ROS Domain ID: 26");
    Serial.println("📶 WiFi IP: " + WiFi.localIP().toString());
    Serial.println("=========================");
  } else {
    Serial.println("⚠️  micro-ROS 연결 실패, 재시도합니다...");
  }
  
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  static boolean newDataReady = 0;
  static unsigned long last_heartbeat = 0;
  static unsigned long last_reconnect_attempt = 0;
  static int consecutive_failures = 0;
  
  // WiFi 연결 상태 확인 (더 자주)
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi 연결 끊김. 재연결 시도...");
    wifi_connected = false;
    microros_connected = false;
    
    // WiFi 재연결
    WiFi.disconnect();
    delay(1000);
    connectWiFi();
    
    if (wifi_connected) {
      delay(2000);  // WiFi 안정화 대기
      microros_connected = connectMicroROS();
    }
    
    consecutive_failures = 0;  // 재연결 성공 시 실패 카운터 초기화
    return;
  }
  
  // micro-ROS 연결 상태 확인 및 재시도 (더 적극적)
  if (!microros_connected) {
    if (millis() - last_reconnect_attempt > 3000) {  // 3초마다 재시도
      Serial.println("🔄 micro-ROS 재연결 시도...");
      microros_connected = connectMicroROS();
      last_reconnect_attempt = millis();
      
      if (!microros_connected) {
        consecutive_failures++;
        Serial.printf("❌ micro-ROS 재연결 실패 (%d/5)\n", consecutive_failures);
        
        // 5번 연속 실패 시 WiFi도 재연결
        if (consecutive_failures >= 5) {
          Serial.println("🔄 WiFi 재연결 시도...");
          WiFi.disconnect();
          delay(1000);
          connectWiFi();
          consecutive_failures = 0;
        }
      } else {
        consecutive_failures = 0;
        Serial.println("✅ micro-ROS 재연결 성공!");
      }
    }
    return;
  }
  
  // 새로운 무게 데이터 확인
  if (LoadCell.update()) {
    newDataReady = true;
  }

  // 무게 측정 및 퍼블리시
  if (newDataReady && (millis() - lastPublishTime > publishInterval)) {
    float weight = LoadCell.getData();
    
    // 무게 값 설정
    weight_msg.data = weight;
    
    // ROS2 토픽으로 퍼블리시
    rcl_ret_t ret = rcl_publish(&publisher, &weight_msg, NULL);
    
    // 시리얼 출력
    Serial.print("📊 무게: ");
    if (abs(weight) < 0.1) {
      Serial.print("0.0 g");
    } else {
      Serial.print(weight, 1);
      Serial.print(" g");
    }
    
    if (ret == RCL_RET_OK) {
      Serial.println(" -> ✅ 퍼블리시 성공");
      consecutive_failures = 0;  // 성공 시 실패 카운터 초기화
      last_heartbeat = millis();
    } else {
      Serial.println(" -> ❌ 퍼블리시 실패");
      consecutive_failures++;
      
      // 3번 연속 퍼블리시 실패 시 재연결
      if (consecutive_failures >= 3) {
        Serial.println("🔄 연속 퍼블리시 실패로 인한 재연결");
        microros_connected = false;
        consecutive_failures = 0;
      }
    }
    
    newDataReady = 0;
    lastPublishTime = millis();
    
    // LED 깜빡임으로 퍼블리시 상태 표시
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }

  // micro-ROS 실행자 실행 (에러 체크 추가)
  if (microros_connected) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
      Serial.println("⚠️  Executor 에러 발생");
      microros_connected = false;  // 재연결 트리거
    }
  }

  // 하트비트 체크 (30초 동안 퍼블리시가 성공하지 않으면 재연결)
  if (microros_connected && (millis() - last_heartbeat > 30000)) {
    Serial.println("💔 하트비트 타임아웃 - 재연결 시도");
    microros_connected = false;
    last_heartbeat = millis();
  }

  // 시리얼 명령어 처리
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte == 't') {
      Serial.println("🔄 영점 조정 중...");
      LoadCell.tareNoDelay();
    }
    else if (inByte == 'r') {
      Serial.println("🔧 캘리브레이션 모드로 전환합니다.");
    }
    else if (inByte == 'c') {
      changeCalFactor();
    }
    else if (inByte == 'w') {
      Serial.print("📶 WiFi 상태: ");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("연결됨 - " + WiFi.localIP().toString());
      } else {
        Serial.println("연결 끊김");
      }
      Serial.print("🤖 micro-ROS 상태: ");
      Serial.println(microros_connected ? "연결됨" : "연결 끊김");
      Serial.printf("❌ 연속 실패 횟수: %d\n", consecutive_failures);
    }
    else if (inByte == 'm') {
      // 수동 micro-ROS 재연결
      Serial.println("🔄 micro-ROS 수동 재연결 시도...");
      microros_connected = connectMicroROS();
      consecutive_failures = 0;
    }
    else if (inByte == 'R') {
      // 완전 재시작
      Serial.println("🔄 완전 재시작...");
      ESP.restart();
    }
  }

  // 영점 조정 완료 확인
  if (LoadCell.getTareStatus() == true) {
    Serial.println("✅ 영점 조정 완료!");
  }
}

// 캘리브레이션 값 수동 변경 함수
void changeCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  
  Serial.println("=========================");
  Serial.print("현재 캘리브레이션 값: ");
  Serial.println(oldCalibrationValue);
  Serial.println("새로운 값을 입력하세요 (예: 696.0):");
  
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("새로운 캘리브레이션 값: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  
  _resume = false;
  Serial.print("EEPROM에 저장하시겠습니까? (y/n): ");
  
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
        EEPROM.begin(512);
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
        EEPROM.commit();
        Serial.println("✅ 값이 EEPROM에 저장되었습니다!");
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("❌ 값이 저장되지 않았습니다.");
        _resume = true;
      }
    }
  }
  Serial.println("=========================");
}