/*
   ESP32 + HX711 로드셀 + micro-ROS 무게 측정 코드
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

// 핀 설정
const int HX711_dout = 4; // mcu > HX711 dout pin
const int HX711_sck = 18;  // mcu > HX711 sck pin

// ESP32 내장 LED 핀 (보드에 따라 다를 수 있음)
#ifdef LED_BUILTIN
const int LED_PIN = LED_BUILTIN;
#else
const int LED_PIN = 2;  // ESP32 Dev Module 기본 LED 핀
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
const unsigned long publishInterval = 1000; // 1초마다 퍼블리시

// micro-ROS 에러 핸들러
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 무게 센서 micro-ROS 시작 ===");

  // LED 핀 설정
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

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
    Serial.println("   먼저 캘리브레이션을 진행하세요.");
    calibrationValue = 1.0; // 기본값
  } else {
    Serial.print("✅ EEPROM에서 캘리브레이션 값 로드: ");
    Serial.println(calibrationValue);
  }
  
  LoadCell.setCalFactor(calibrationValue);
  Serial.println("✅ 로드셀 준비 완료!");

  // micro-ROS 초기화
  Serial.println("🔧 micro-ROS 초기화 중...");
  
  // micro-ROS 설정
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  
  // micro-ROS 지원 초기화
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // 노드 생성
  RCCHECK(rclc_node_init_default(&node, "esp32_weight_sensor", "", &support));
  
  // 퍼블리셔 생성
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "weight_data"));

  // 실행자 생성
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  Serial.println("✅ micro-ROS 초기화 완료!");
  Serial.println("📡 무게 데이터를 /weight_data 토픽으로 퍼블리시합니다.");
  Serial.println("🌐 ROS Domain ID: 26");
  Serial.println("=========================");
  
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  static boolean newDataReady = 0;
  
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
    RCSOFTCHECK(rcl_publish(&publisher, &weight_msg, NULL));
    
    // 시리얼 출력
    Serial.print("📊 무게: ");
    if (abs(weight) < 0.1) {
      Serial.print("0.0 g");
    } else {
      Serial.print(weight, 1);
      Serial.print(" g");
    }
    Serial.println(" -> /weight_data 토픽으로 퍼블리시됨");
    
    newDataReady = 0;
    lastPublishTime = millis();
    
    // LED 깜빡임으로 퍼블리시 상태 표시
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }

  // micro-ROS 실행자 실행
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // 시리얼 명령어 처리
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte == 't') {
      Serial.println("🔄 영점 조정 중...");
      LoadCell.tareNoDelay();
    }
    else if (inByte == 'r') {
      Serial.println("🔧 캘리브레이션 모드로 전환합니다.");
      Serial.println("   캘리브레이션 코드를 업로드하세요.");
    }
    else if (inByte == 'c') {
      changeCalFactor();
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