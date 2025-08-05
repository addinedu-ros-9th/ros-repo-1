/*
   ESP32 통합 센서 + LED 제어 시스템 (USB Serial + micro-ROS)
   - HX711 로드셀로 무게 측정 → /weight_data 토픽으로 퍼블리시
   - /led_status 토픽 구독 → Neopixel LED 색상 제어
   - 도메인 ID: 77
   - 통신: USB Serial (115200 baud)
*/

#include <Adafruit_NeoPixel.h>
#include <micro_ros_arduino.h>
#include <HX711_ADC.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <stdio.h>
#include <EEPROM.h>

// ===== 핀 설정 =====
// HX711 로드셀 핀
const int HX711_dout = 4;
const int HX711_sck = 18;

// Neopixel LED 스트립 핀
#define LED_STRIP_PIN   2        // ESP32의 GPIO 2번 핀
#define NUM_LEDS        30       // LED 개수
#define BRIGHTNESS      64       // 밝기 (0-255)

// 상태 LED 핀 (내장 LED 또는 별도)
#ifdef LED_BUILTIN
const int STATUS_LED_PIN = LED_BUILTIN;
#else
const int STATUS_LED_PIN = 13;
#endif

// ===== 객체 생성 =====
// HX711 객체
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// Adafruit_NeoPixel 객체
Adafruit_NeoPixel strip(NUM_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// ===== micro-ROS 설정 =====
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// 퍼블리셔 (무게 데이터)
rcl_publisher_t weight_publisher;
std_msgs__msg__Float32 weight_msg;

// 구독자 (LED 상태)
rcl_subscription_t led_subscription;
std_msgs__msg__String led_msg;

// 메시지 버퍼 (정적 할당)
char led_msg_buffer[50];  // LED 메시지용 정적 버퍼

// ===== 상태 변수 =====
// 연결 상태
bool microros_connected = false;
unsigned long last_heartbeat = 0;

// 무게 센서 관련
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 1000;  // 1초마다 퍼블리시
const int calVal_eepromAddress = 0;

// LED 제어 관련
volatile bool new_led_message_received = false;
volatile int led_status = 0;  // 0: 꺼짐, 1: 기쁨(초록), 2: 슬픔(파랑), 3: 화남(빨강)

// 에러 카운터
int consecutive_failures = 0;

// ===== micro-ROS 에러 핸들러 =====
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  Serial.println("💥 micro-ROS 치명적 오류 발생!");
  while(1){
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    delay(100);
  }
}

// ===== LED 구독자 콜백 함수 =====
void led_subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * received_msg = (const std_msgs__msg__String *)msgin;
  
  // 최소한의 검증만
  if (received_msg == NULL || received_msg->data.data == NULL) {
    return;
  }
  
  // 빠른 문자 비교 (첫 글자만 확인)
  char first_char = received_msg->data.data[0];
  if (first_char == 0xEA) {  // "기쁨"의 첫 바이트 (UTF-8)
    led_status = 1;
  } else if (first_char == 0xEC) {  // "슬픔"의 첫 바이트 (UTF-8)
    led_status = 2;
  } else if (first_char == 0xED) {  // "화남"의 첫 바이트 (UTF-8)
    led_status = 3;
  } else {
    led_status = 0;
  }
  
  new_led_message_received = true;
  last_heartbeat = millis();
}

// ===== micro-ROS 연결 함수 =====
bool connectMicroROS() {
  Serial.println("🔧 micro-ROS Serial 연결 시도 중...");
  
  // Serial 통신 설정 (USB Serial 사용)
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  
  // init options 설정 (도메인 ID 77)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ init options 초기화 실패: %d\n", ret);
    return false;
  }
  
  // 도메인 ID 77 설정
  ret = rcl_init_options_set_domain_id(&init_options, 77);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ 도메인 ID 설정 실패: %d\n", ret);
    return false;
  }
  
  // micro-ROS 지원 초기화 (도메인 ID 포함)
  ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ micro-ROS support 초기화 실패: %d\n", ret);
    return false;
  }
  
  // 노드 생성
  ret = rclc_node_init_default(&node, "combined_sensor_led", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ 노드 생성 실패: %d\n", ret);
    return false;
  }
  
  // 무게 데이터 퍼블리셔 생성
  ret = rclc_publisher_init_default(
    &weight_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "weight_data");
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ 무게 퍼블리셔 생성 실패: %d\n", ret);
    return false;
  }
  
  // LED 상태 구독자 생성
  ret = rclc_subscription_init_default(
    &led_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "led_status");
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ LED 구독자 생성 실패: %d\n", ret);
    return false;
  }
  
  // executor 생성 (퍼블리셔 1개 + 구독자 1개 = 2개)
  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ executor 생성 실패: %d\n", ret);
    return false;
  }
  
  // LED 메시지 버퍼 초기화 (정적 할당)
  led_msg.data.data = led_msg_buffer;
  led_msg.data.size = 0;
  led_msg.data.capacity = sizeof(led_msg_buffer);
  
  // 구독자를 executor에 추가
  ret = rclc_executor_add_subscription(&executor, &led_subscription, &led_msg, &led_subscription_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ LED 구독자 추가 실패: %d\n", ret);
    return false;
  }
  
  microros_connected = true;
  
  Serial.println("✅ micro-ROS Serial 연결 성공!");
  Serial.println("🌐 도메인 ID: 77 설정됨");
  Serial.println("📡 /weight_data 퍼블리셔 준비 완료");
  Serial.println("📡 /led_status 구독자 준비 완료");
  Serial.printf("💾 LED 메시지 버퍼: %d bytes 할당됨\n", sizeof(led_msg_buffer));
  return true;
}

// ===== LED 처리 함수 =====
void processLEDUpdate() {
  if (new_led_message_received) {
    new_led_message_received = false;
    
    switch (led_status) {
      case 1:  // 기쁨
        setSolidColor(0, 255, 0);  // 초록색
        Serial.println("📨 기쁨 -> 💚 초록색");
        break;
      case 2:  // 슬픔
        setSolidColor(0, 0, 255);  // 파란색
        Serial.println("📨 슬픔 -> 💙 파란색");
        break;
      case 3:  // 화남
        setSolidColor(255, 0, 0);  // 빨간색
        Serial.println("📨 화남 -> ❤️ 빨간색");
        break;
      default:  // 꺼짐
        turnOffAll();
        Serial.println("📨 알 수 없음 -> 🔹 꺼짐");
        break;
    }
  }
}

// ===== LED 제어 함수들 =====
void setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
  Serial.printf("🎨 LED 제어! 색상: R=%d, G=%d, B=%d\n", r, g, b);
}

void turnOffAll() {
  setSolidColor(0, 0, 0);
}

// ===== 캘리브레이션 값 변경 함수 =====
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
        EEPROM.put(calVal_eepromAddress, newCalibrationValue);
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

// ===== SETUP 함수 =====
void setup() {
  // Serial 통신 초기화 (115200 baud)
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== ESP32 통합 센서 + LED 시스템 (micro-ROS Serial) ===");
  Serial.println("📡 통신 방식: USB Serial (115200 baud)");
  Serial.println("🌐 ROS2 Domain ID: 77");
  
  // 상태 LED 핀 설정
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  // Neopixel LED 초기화
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show(); // 모든 LED 끄기
  Serial.println("🌈 Neopixel LED Strip 초기화 완료!");
  
  // LED 테스트
  Serial.println("🧪 LED 테스트 중...");
  setSolidColor(255, 0, 0);  // 빨간색
  delay(300);
  setSolidColor(0, 255, 0);  // 초록색
  delay(300);
  setSolidColor(0, 0, 255);  // 파란색
  delay(300);
  turnOffAll();
  
  // HX711 로드셀 초기화
  Serial.println("⚖️  HX711 로드셀 초기화 중...");
  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("❌ 오류: HX711 연결을 확인하세요!");
    while (1) {
      setSolidColor(255, 0, 0);
      delay(200);
      turnOffAll();
      delay(200);
    }
  }
  
  // EEPROM에서 캘리브레이션 값 불러오기
  EEPROM.begin(512);
  float calibrationValue;
  EEPROM.get(calVal_eepromAddress, calibrationValue);
  
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
  delay(1000);
  Serial.println("🔗 micro-ROS agent 연결 대기 중...");
  Serial.println("💡 다음 명령어로 agent를 실행하세요:");
  Serial.println("   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200");
  
  if (connectMicroROS()) {
    Serial.println("=========================");
    
    setSolidColor(0, 255, 0);  // 초록색으로 연결 성공 표시
    delay(1000);
    turnOffAll();
    
    last_heartbeat = millis();
  } else {
    Serial.println("⚠️  micro-ROS 초기 연결 실패");
    Serial.println("💡 Agent가 실행되었는지 확인하고 ESP32를 재시작하세요.");
  }
  
  digitalWrite(STATUS_LED_PIN, LOW);
}

// ===== LOOP 함수 =====
void loop() {
  static boolean newWeightDataReady = 0;
  static unsigned long last_reconnect_attempt = 0;
  
  // LED 메시지 처리
  processLEDUpdate();
  
  // micro-ROS 재연결 로직
  if (!microros_connected) {
    if (millis() - last_reconnect_attempt > 5000) {
      Serial.println("🔄 micro-ROS 재연결 시도...");
      
      if (connectMicroROS()) {
        Serial.println("✅ micro-ROS 재연결 성공!");
        last_heartbeat = millis();
        consecutive_failures = 0;
      } else {
        Serial.println("❌ micro-ROS 재연결 실패");
        consecutive_failures++;
      }
      
      last_reconnect_attempt = millis();
    }
    
    // 연결되지 않은 경우 상태 표시
    static unsigned long last_blink = 0;
    if (millis() - last_blink >= 2000) {
      last_blink = millis();
      setSolidColor(255, 100, 0);  // 주황색으로 재연결 시도 표시
      delay(50);
      turnOffAll();
    }
    return;
  }
  
  // 새로운 무게 데이터 확인
  if (LoadCell.update()) {
    newWeightDataReady = true;
  }
  
  // 무게 측정 및 퍼블리시
  if (newWeightDataReady && (millis() - lastPublishTime > publishInterval)) {
    float weight = LoadCell.getData();
    
    // 무게 값 설정
    weight_msg.data = weight;
    
    // ROS2 토픽으로 퍼블리시
    rcl_ret_t ret = rcl_publish(&weight_publisher, &weight_msg, NULL);
    
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
      consecutive_failures = 0;
      last_heartbeat = millis();
    } else {
      Serial.println(" -> ❌ 퍼블리시 실패");
      consecutive_failures++;
      
      if (consecutive_failures >= 3) {
        Serial.println("🔄 연속 퍼블리시 실패로 인한 재연결");
        microros_connected = false;
        consecutive_failures = 0;
      }
    }
    
    newWeightDataReady = 0;
    lastPublishTime = millis();
    
    // LED 깜빡임으로 퍼블리시 상태 표시
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
  }
  
  // micro-ROS executor 실행
  if (microros_connected) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));  // 10ms 타임아웃
    
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
      static unsigned long last_error_log = 0;
      if (millis() - last_error_log > 5000) {
        Serial.printf("⚠️  Executor 에러: %d\n", ret);
        last_error_log = millis();
      }
    } else if (ret == RCL_RET_OK) {
      // 성공 시 간헐적으로 상태 LED 깜빡임
      static unsigned long last_activity = 0;
      if (millis() - last_activity > 10000) {  // 10초마다
        last_activity = millis();
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(2);
        digitalWrite(STATUS_LED_PIN, LOW);
      }
    }
  }
  
  // 하트비트 체크
  if (microros_connected && (millis() - last_heartbeat > 60000)) {
    Serial.println("💔 하트비트 타임아웃 - 재연결 시도");
    microros_connected = false;
    last_heartbeat = millis();
  }
  
  // 영점 조정 완료 확인
  if (LoadCell.getTareStatus() == true) {
    Serial.println("✅ 영점 조정 완료!");
  }
  
  // 시리얼 명령어 처리
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte == 'w') {
      Serial.println("=== 시스템 상태 ===");
      Serial.printf("🤖 micro-ROS: %s\n", microros_connected ? "연결됨" : "연결 끊김");
      Serial.printf("💓 마지막 하트비트: %lu ms 전\n", millis() - last_heartbeat);
      Serial.printf("🎨 현재 LED 상태: %d\n", led_status);
      Serial.printf("❌ 연속 실패 횟수: %d\n", consecutive_failures);
      Serial.println("🌐 도메인 ID: 77");
      Serial.println("📡 통신: USB Serial (115200)");
      Serial.println("==================");
    }
    else if (inByte == 'm') {
      Serial.println("🔄 micro-ROS 수동 재연결 시도...");
      if (connectMicroROS()) {
        last_heartbeat = millis();
        consecutive_failures = 0;
      }
    }
    else if (inByte == 'R') {
      Serial.println("🔄 ESP32 재시작...");
      ESP.restart();
    }
    else if (inByte == 't') {
      Serial.println("🧪 LED 테스트 모드");
      setSolidColor(255, 0, 0);
      delay(500);
      setSolidColor(0, 255, 0);
      delay(500);
      setSolidColor(0, 0, 255);
      delay(500);
      turnOffAll();
      Serial.println("✅ LED 테스트 완료");
    }
    else if (inByte == 'z') {
      Serial.println("🔄 영점 조정 중...");
      LoadCell.tareNoDelay();
    }
    else if (inByte == 'c') {
      changeCalFactor();
    }
  }
  
  delay(10);  // 더 빠른 루프
}