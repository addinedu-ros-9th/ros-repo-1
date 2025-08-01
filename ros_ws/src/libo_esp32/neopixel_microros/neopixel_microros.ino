/*
   ESP32 + Neopixel + micro-ROS LED 제어 코드 (유선 버전)
   무게 센서 코드 패턴을 정확히 따라 구현
*/

#include <FastLED.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// LED 스트립 설정
#define LED_PIN     2        // ESP32의 GPIO 2번 핀
#define NUM_LEDS    30       // LED 개수
#define BRIGHTNESS  64       // 밝기 (0-255)
#define LED_TYPE    WS2812B  // LED 타입
#define COLOR_ORDER GRB      // 색상 순서

CRGB leds[NUM_LEDS];

#ifdef LED_BUILTIN
const int STATUS_LED_PIN = LED_BUILTIN;
#else
const int STATUS_LED_PIN = 13;
#endif

// micro-ROS 관련 변수 (무게 센서와 동일한 구조)
rcl_node_t node;
rcl_subscription_t subscription;
std_msgs__msg__String led_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;

// LED 제어 관련 변수
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 100;  // 100ms마다 LED 상태 체크

// 연결 상태
bool microros_connected = false;

// LED 상태 변수
volatile int current_led_status = 0;  // 0: 꺼짐, 1: 기쁨, 2: 슬픔, 3: 화남

// micro-ROS 에러 핸들러 (무게 센서와 동일)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    delay(100);
  }
}

// 구독자 콜백 함수 (디버깅 강화)
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // 콜백 호출 확인
  Serial.println("🔔 콜백 함수 호출됨!");
  
  // 메시지 유효성 검사
  if (msg == NULL) {
    Serial.println("❌ 메시지가 NULL입니다");
    return;
  }
  
  if (msg->data.data == NULL) {
    Serial.println("❌ 메시지 데이터가 NULL입니다");
    return;
  }
  
  Serial.printf("📨 수신 메시지: '%s'\n", msg->data.data);
  
  // 메시지 처리
  if (strcmp(msg->data.data, "기쁨") == 0) {
    current_led_status = 1;
    Serial.println("✅ 기쁨 상태로 설정");
  } else if (strcmp(msg->data.data, "슬픔") == 0) {
    current_led_status = 2;
    Serial.println("✅ 슬픔 상태로 설정");
  } else if (strcmp(msg->data.data, "화남") == 0) {
    current_led_status = 3;
    Serial.println("✅ 화남 상태로 설정");
  } else {
    current_led_status = 0;
    Serial.printf("⚠️  알 수 없는 메시지: '%s'\n", msg->data.data);
  }
}

// micro-ROS 연결 시도 (무게 센서와 동일한 패턴)
bool connectMicroROS() {
  Serial.println("🔧 micro-ROS 연결 시도 중...");
  
  // micro-ROS 시리얼 transport 설정
  set_microros_transports();
  
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
  ret = rclc_node_init_default(&node, "esp32_neopixel_controller", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 노드 생성 실패");
    return false;
  }
  
  // 구독자 생성 (토픽 이름 수정)
  ret = rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/led_status");  // 앞에 슬래시 추가
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 구독자 생성 실패");
    return false;
  }

  // 실행자 생성
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 실행자 생성 실패");
    return false;
  }
  
  // 구독자를 실행자에 추가
  ret = rclc_executor_add_subscription(&executor, &subscription, &led_msg, &subscription_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 구독자 추가 실패");
    return false;
  }
  
  Serial.println("✅ micro-ROS 연결 성공!");
  Serial.println("🌐 도메인 ID: 26 설정됨");
  return true;
}

void setup() {
    // WiFi 연결 (OTA용)
  WiFi.begin("AIE_509_2.4G", "addinedu_class1");
  ArduinoOTA.begin();
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 Neopixel micro-ROS (무게센서 패턴) 시작 ===");

  // STATUS LED 핀 설정
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  // FastLED 초기화
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
  Serial.println("✅ Neopixel 준비 완료!");

  // 초기 LED 테스트
  setSolidColor(255, 0, 0);  // 빨간색
  delay(500);
  setSolidColor(0, 255, 0);  // 초록색
  delay(500);
  setSolidColor(0, 0, 255);  // 파란색
  delay(500);
  turnOffAll();

  // micro-ROS 연결 시도
  microros_connected = connectMicroROS();
  
  if (microros_connected) {
    Serial.println("📡 LED 상태를 /led_status 토픽에서 구독합니다.");
    Serial.println("🌐 ROS Domain ID: 26");
    Serial.println("🔌 시리얼 연결 사용");
    Serial.println("=========================");
  } else {
    Serial.println("⚠️  micro-ROS 연결 실패, 재시도합니다...");
  }
  
  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  ArduinoOTA.handle();
  static boolean newDataReady = 0;
  static unsigned long last_heartbeat = 0;
  static unsigned long last_reconnect_attempt = 0;
  static int consecutive_failures = 0;
  
  // micro-ROS 연결 상태 확인 및 재시도 (무게 센서 패턴)
  if (!microros_connected) {
    if (millis() - last_reconnect_attempt > 3000) {  // 3초마다 재시도
      Serial.println("🔄 micro-ROS 재연결 시도...");
      microros_connected = connectMicroROS();
      last_reconnect_attempt = millis();
      
      if (!microros_connected) {
        consecutive_failures++;
        Serial.printf("❌ micro-ROS 재연결 실패 (%d회)\n", consecutive_failures);
      } else {
        consecutive_failures = 0;
        Serial.println("✅ micro-ROS 재연결 성공!");
      }
    }
    return;
  }
  
  // LED 상태 업데이트 (무게 센서 패턴을 따라)
  if (millis() - lastUpdateTime > updateInterval) {
    
    // LED 제어
    switch (current_led_status) {
      case 1:  // 기쁨
        setSolidColor(0, 255, 0);  // 초록색
        Serial.println("📊 LED: 💚 기쁨 (초록색)");
        break;
      case 2:  // 슬픔
        setSolidColor(0, 0, 255);  // 파란색
        Serial.println("📊 LED: 💙 슬픔 (파란색)");
        break;
      case 3:  // 화남
        setSolidColor(255, 0, 0);  // 빨간색
        Serial.println("📊 LED: ❤️ 화남 (빨간색)");
        break;
      default:  // 꺼짐
        turnOffAll();
        Serial.println("📊 LED: 🔹 꺼짐");
        break;
    }
    
    lastUpdateTime = millis();
    
    // LED 깜빡임으로 상태 표시 (무게 센서 패턴)
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  // micro-ROS 실행자 실행 (디버깅 강화)
  if (microros_connected) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    
    // 실행자 상태를 더 자세히 로깅
    static unsigned long last_executor_log = 0;
    if (millis() - last_executor_log > 2000) {  // 2초마다 상태 로그
      Serial.printf("🔄 Executor 상태: %d ", ret);
      if (ret == RCL_RET_OK) {
        Serial.println("(성공)");
      } else if (ret == RCL_RET_TIMEOUT) {
        Serial.println("(타임아웃 - 정상)");
      } else {
        Serial.printf("(에러: %d)\n", ret);
      }
      last_executor_log = millis();
    }
    
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
      consecutive_failures++;
      Serial.printf("⚠️  Executor 에러 발생: %d (연속 실패: %d)\n", ret, consecutive_failures);
      
      if (consecutive_failures >= 5) {
        Serial.println("🔄 연속 실패로 재연결 시도");
        microros_connected = false;
        consecutive_failures = 0;
      }
    } else {
      if (consecutive_failures > 0) {
        consecutive_failures = 0;  // 성공 시 실패 카운터 초기화
      }
    }
  }

  // 시리얼 명령어 처리 (무게 센서 패턴)
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte == 'w') {
      Serial.println("=== 상세 시스템 상태 ===");
      Serial.printf("🤖 micro-ROS 연결: %s\n", microros_connected ? "✅ 연결됨" : "❌ 연결 끊김");
      Serial.printf("🎨 현재 LED 상태: %d\n", current_led_status);
      Serial.printf("📊 연속 실패 횟수: %d\n", consecutive_failures);
      Serial.printf("⏰ 업타임: %lu ms\n", millis());
      Serial.println("🔌 시리얼 연결 사용 중");
      Serial.println("📡 구독 토픽: /led_status");
      Serial.println("========================");
    }
    else if (inByte == 'm') {
      Serial.println("🔄 micro-ROS 수동 재연결 시도...");
      microros_connected = connectMicroROS();
      consecutive_failures = 0;
    }
    else if (inByte == 'R') {
      Serial.println("🔄 완전 재시작...");
      ESP.restart();
    }
    else if (inByte == 't') {
      Serial.println("🧪 LED 테스트");
      setSolidColor(255, 0, 0);  // 빨간색
      delay(500);
      setSolidColor(0, 255, 0);  // 초록색
      delay(500);
      setSolidColor(0, 0, 255);  // 파란색
      delay(500);
      turnOffAll();
      Serial.println("✅ 테스트 완료");
    }
  }
}

// LED 제어 함수들
void setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
}

void turnOffAll() {
  setSolidColor(0, 0, 0);
}