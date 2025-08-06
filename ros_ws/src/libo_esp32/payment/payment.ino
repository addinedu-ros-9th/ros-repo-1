/*
   ESP32 RFID 결제 시스템 (micro-ROS Serial 통신)
   - MFRC522 RFID 리더기로 카드 인식
   - micro-ROS Serial을 통해 /rfid_payment 토픽으로 카드 정보 퍼블리시
   - 결제 성공시 LED 표시 및 피드백
   - 도메인 ID: 26 (프로젝트 표준)
   - 통신: USB Serial (115200 baud)
*/

#include <SPI.h>
#include <MFRC522.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <stdio.h>

// ===== RFID 핀 설정 =====
#define RST_PIN         19    // RFID RST 핀
#define SS_PIN          5     // RFID SDA 핀 (SS)
#define SCK_PIN         18    // RFID SCK 핀
#define MISO_PIN        19    // RFID MISO 핀 (RST와 공유)
#define MOSI_PIN        23    // RFID MOSI 핀

// ===== LED 핀 =====
#define SUCCESS_LED_PIN 2     // 결제 성공 LED (내장 LED)

// ===== RFID 객체 생성 =====
MFRC522 mfrc522(SS_PIN, RST_PIN);

// ===== micro-ROS 설정 =====
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_init_options_t init_options;

// 퍼블리셔들
rcl_publisher_t rfid_publisher;      // RFID 카드 데이터
rcl_publisher_t payment_publisher;   // 결제 상태

// 구독자 (결제 결과 수신)
rcl_subscription_t payment_result_subscription;

// 메시지 객체들
std_msgs__msg__String rfid_msg;
std_msgs__msg__Bool payment_status_msg;
std_msgs__msg__String payment_result_msg;

// 메시지 버퍼 (정적 할당)
char rfid_buffer[50];
char payment_result_buffer[100];

// ===== 상태 변수 =====
bool microros_connected = false;

// RFID 관련
String lastCardUID = "";
unsigned long lastCardTime = 0;
const unsigned long cardCooldown = 3000; // 3초 쿨다운

// 결제 상태
bool payment_in_progress = false;
unsigned long payment_start_time = 0;
const unsigned long payment_timeout = 10000; // 10초 타임아웃

// LED 제어
bool led_state = false;
unsigned long led_blink_timer = 0;

// ===== micro-ROS 에러 핸들러 =====
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  Serial.println("💥 micro-ROS 치명적 오류 발생!");
  while(1) {
    digitalWrite(SUCCESS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(SUCCESS_LED_PIN, LOW);
    delay(100);
  }
}

// ===== 결제 결과 콜백 함수 =====
void payment_result_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  Serial.print("💳 결제 결과 수신: ");
  Serial.println(msg->data.data);
  
  String result = String(msg->data.data);
  
  if (result.startsWith("SUCCESS")) {
    // 결제 성공
    payment_success_feedback();
    Serial.println("✅ 결제가 성공적으로 완료되었습니다!");
  } 
  else if (result.startsWith("FAILED")) {
    // 결제 실패
    payment_failed_feedback();
    Serial.println("❌ 결제에 실패했습니다!");
  }
  
  payment_in_progress = false;
}

// ===== micro-ROS 연결 함수 (Serial 통신) =====
bool connectMicroROS() {
  Serial.println("🔧 micro-ROS Serial 연결 시도 중...");
  
  // Serial transport 설정 (WiFi 대신)
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  
  // init options 설정 (도메인 ID 26)
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
  
  // micro-ROS 지원 초기화
  ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS support 초기화 실패");
    return false;
  }
  
  // 노드 생성
  ret = rclc_node_init_default(&node, "esp32_rfid_payment", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 노드 생성 실패");
    return false;
  }
  
  // RFID 데이터 퍼블리셔 생성
  ret = rclc_publisher_init_default(
    &rfid_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "rfid_payment");
  if (ret != RCL_RET_OK) {
    Serial.println("❌ RFID 퍼블리셔 생성 실패");
    return false;
  }
  
  // 결제 상태 퍼블리셔 생성
  ret = rclc_publisher_init_default(
    &payment_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "payment_status");
  if (ret != RCL_RET_OK) {
    Serial.println("❌ 결제 상태 퍼블리셔 생성 실패");
    return false;
  }
  
  // 결제 결과 구독자 생성
  ret = rclc_subscription_init_default(
    &payment_result_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "payment_result");
  if (ret != RCL_RET_OK) {
    Serial.println("❌ 결제 결과 구독자 생성 실패");
    return false;
  }

  // 실행자 생성 (구독자 1개)
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 실행자 생성 실패");
    return false;
  }
  
  // 구독자를 실행자에 추가
  ret = rclc_executor_add_subscription(&executor, &payment_result_subscription, &payment_result_msg, &payment_result_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial.println("❌ 구독자 실행자 추가 실패");
    return false;
  }
  
  // 메시지 버퍼 설정
  rfid_msg.data.data = rfid_buffer;
  rfid_msg.data.size = 0;
  rfid_msg.data.capacity = sizeof(rfid_buffer);
  
  payment_result_msg.data.data = payment_result_buffer;
  payment_result_msg.data.size = 0;
  payment_result_msg.data.capacity = sizeof(payment_result_buffer);
  
  Serial.println("✅ micro-ROS Serial 연결 성공!");
  Serial.println("🌐 도메인 ID: 26 설정됨");
  Serial.println("🔌 USB Serial 통신 사용");
  return true;
}

// ===== RFID 초기화 함수 =====
void initRFID() {
  Serial.println("🏷️  RFID 시스템 초기화 중...");
  
  // SPI 핀 설정
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  
  // MFRC522 초기화
  mfrc522.PCD_Init();
  
  // RFID 리더기 버전 확인
  mfrc522.PCD_DumpVersionToSerial();
  
  Serial.println("✅ RFID 시스템 준비 완료!");
  Serial.println("💳 결제용 RFID 카드를 리더기에 대주세요...");
}

// ===== RFID 카드 읽기 함수 =====
String readRFIDCard() {
  // 새로운 카드가 있는지 확인
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return "";
  }
  
  // 카드 선택
  if (!mfrc522.PICC_ReadCardSerial()) {
    return "";
  }
  
  // UID를 문자열로 변환
  String cardUID = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    cardUID += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
    cardUID += String(mfrc522.uid.uidByte[i], HEX);
  }
  cardUID.toUpperCase();
  
  // 카드 선택 해제
  mfrc522.PICC_HaltA();
  
  return cardUID;
}

// ===== 결제 요청 처리 함수 =====
void processPaymentRequest(String cardUID) {
  Serial.print("💳 결제 요청 - 카드 UID: ");
  Serial.println(cardUID);
  
  // RFID 데이터 퍼블리시
  String rfid_data = "CARD_ID:" + cardUID + ",TIMESTAMP:" + String(millis());
  strcpy(rfid_buffer, rfid_data.c_str());
  rfid_msg.data.size = strlen(rfid_buffer);
  
  rcl_ret_t ret = rcl_publish(&rfid_publisher, &rfid_msg, NULL);
  if (ret == RCL_RET_OK) {
    Serial.println("📡 RFID 데이터 전송 성공");
  } else {
    Serial.println("❌ RFID 데이터 전송 실패");
  }
  
  // 결제 진행 상태 설정
  payment_in_progress = true;
  payment_start_time = millis();
  
  // 결제 상태 퍼블리시 (true = 진행 중)
  payment_status_msg.data = true;
  ret = rcl_publish(&payment_publisher, &payment_status_msg, NULL);
  if (ret == RCL_RET_OK) {
    Serial.println("📡 결제 상태 전송 성공 (진행 중)");
  }
  
  // 카드 정보 저장 (중복 방지용)
  lastCardUID = cardUID;
  lastCardTime = millis();
  
  // LED 깜빡임 시작 (결제 진행 중 표시)
  led_blink_timer = millis();
  
  Serial.println("⏳ 결제 처리 중... 잠시만 기다려 주세요.");
}

// ===== 결제 성공 피드백 =====
void payment_success_feedback() {
  Serial.println("🎉 결제 성공!");
  
  // 성공 LED 켜기 (3초간)
  digitalWrite(SUCCESS_LED_PIN, HIGH);
  delay(3000);
  digitalWrite(SUCCESS_LED_PIN, LOW);
  
  // 결제 상태 리셋
  payment_status_msg.data = false;
  rcl_publish(&payment_publisher, &payment_status_msg, NULL);
}

// ===== 결제 실패 피드백 =====
void payment_failed_feedback() {
  Serial.println("💥 결제 실패!");
  
  // 실패 LED 깜빡임 (3번)
  for (int i = 0; i < 3; i++) {
    digitalWrite(SUCCESS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(SUCCESS_LED_PIN, LOW);
    delay(200);
  }
  
  // 결제 상태 리셋
  payment_status_msg.data = false;
  rcl_publish(&payment_publisher, &payment_status_msg, NULL);
}

// ===== LED 깜빡임 처리 =====
void handleLEDBlink() {
  if (payment_in_progress) {
    // 결제 진행 중 LED 깜빡임
    if (millis() - led_blink_timer > 500) {
      led_state = !led_state;
      digitalWrite(SUCCESS_LED_PIN, led_state);
      led_blink_timer = millis();
    }
  }
}

// ===== 결제 타임아웃 처리 =====
void handlePaymentTimeout() {
  if (payment_in_progress && (millis() - payment_start_time > payment_timeout)) {
    Serial.println("⏰ 결제 타임아웃!");
    payment_failed_feedback();
    payment_in_progress = false;
  }
}

// ===== SETUP 함수 =====
void setup() {
  Serial.begin(115200);
  delay(2000);  // Serial 통신 안정화 대기
  Serial.println("=== ESP32 RFID 결제 시스템 (Serial) 시작 ===");
  Serial.println("🔌 통신 방식: USB Serial (115200 baud)");
  Serial.println("🌐 ROS2 Domain ID: 26");
  
  // 핀 초기화
  pinMode(SUCCESS_LED_PIN, OUTPUT);
  digitalWrite(SUCCESS_LED_PIN, LOW);
  
  // RFID 초기화
  initRFID();
  
  // micro-ROS 연결 대기 및 시도
  Serial.println("🔗 micro-ROS agent 연결 대기 중...");
  Serial.println("💡 다음 명령어로 agent를 실행하세요:");
  Serial.println("   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6");
  Serial.println("   (포트는 실제 ESP32 연결 포트로 변경)");
  
  delay(1000);
  microros_connected = connectMicroROS();
  
  if (microros_connected) {
    Serial.println("=========================");
    Serial.println("🏪 RFID 결제 시스템 준비 완료!");
    Serial.println("💳 카드를 리더기에 대주세요...");
    Serial.println("=========================");
    
    // 준비 완료 LED 표시
    for (int i = 0; i < 2; i++) {
      digitalWrite(SUCCESS_LED_PIN, HIGH);
      delay(200);
      digitalWrite(SUCCESS_LED_PIN, LOW);
      delay(200);
    }
  } else {
    Serial.println("⚠️  micro-ROS 초기 연결 실패");
    Serial.println("💡 Agent가 실행되었는지 확인하고 ESP32를 재시작하세요.");
    Serial.println("📋 연결 체크리스트:");
    Serial.println("   1. USB 케이블 연결 확인");
    Serial.println("   2. Agent 실행 확인");
    Serial.println("   3. 포트 번호 확인 (/dev/ttyUSB0, /dev/ttyACM0 등)");
    Serial.println("   4. ROS2 환경 설정 확인 (jazzy 명령어)");
  }
}

// ===== LOOP 함수 =====
void loop() {
  static unsigned long last_heartbeat = 0;
  static unsigned long last_reconnect_attempt = 0;
  static int consecutive_failures = 0;
  
  // micro-ROS 연결 상태 확인 및 재시도
  if (!microros_connected) {
    if (millis() - last_reconnect_attempt > 5000) {  // 5초마다 재시도
      Serial.println("🔄 micro-ROS Serial 재연결 시도...");
      microros_connected = connectMicroROS();
      last_reconnect_attempt = millis();
      
      if (!microros_connected) {
        consecutive_failures++;
        Serial.printf("❌ micro-ROS 재연결 실패 (%d회)\n", consecutive_failures);
        Serial.println("💡 Agent 연결 상태를 확인하세요:");
        Serial.println("   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200");
      } else {
        consecutive_failures = 0;
        Serial.println("✅ micro-ROS Serial 재연결 성공!");
      }
    }
    delay(100);
    return;
  }
  
  // micro-ROS 실행자 스핀
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  // RFID 카드 읽기
  String cardUID = readRFIDCard();
  
  if (cardUID.length() > 0) {
    // 카드 쿨다운 체크 (같은 카드 중복 처리 방지)
    if (cardUID != lastCardUID || (millis() - lastCardTime > cardCooldown)) {
      if (!payment_in_progress) {
        Serial.println("🏷️  RFID 카드 감지됨!");
        Serial.println("📱 결제 시스템으로 전송 중...");
        
        processPaymentRequest(cardUID);
      } else {
        Serial.println("⏳ 이미 결제가 진행 중입니다. 잠시만 기다려 주세요.");
      }
    }
  }
  
  // LED 깜빡임 처리
  handleLEDBlink();
  
  // 결제 타임아웃 처리
  handlePaymentTimeout();
  
  // 하트비트 (30초마다 상태 출력)
  if (millis() - last_heartbeat > 30000) {
    Serial.println("💓 RFID 결제 시스템 정상 작동 중...");
    last_heartbeat = millis();
  }
  
  delay(100);  // CPU 부하 방지
}