#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// WiFi 설정
char ssid[] = "AIE_509_2.4G";
char password[] = "addinedu_class1";

// PC IP 주소 (실제 PC IP로 변경 필요)
char agent_ip[] = "192.168.0.76";  // WiFi IP 사용
size_t agent_port = 8888;

// LED 스트립 설정
#define LED_PIN     2        // ESP32의 GPIO 2번 핀
#define NUM_LEDS    30       // LED 개수 (메모리 절약)
#define BRIGHTNESS  64       // 밝기 (0-255)

// Adafruit_NeoPixel 객체 생성
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// micro-ROS 설정
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscription;
rclc_executor_t executor;
std_msgs__msg__String msg;

// 메시지 버퍼 (정적 할당)
char msg_buffer[50];  // 메시지용 정적 버퍼

// 연결 상태 변수
bool wifi_connected = false;
bool microros_connected = false;
unsigned long last_heartbeat = 0;

// 콜백 처리용 변수
volatile bool new_message_received = false;
volatile int led_status = 0;  // 0: 꺼짐, 1: 기쁨(초록), 2: 슬픔(파랑), 3: 화남(빨강)
char last_received_status[20] = "";

#ifdef LED_BUILTIN
const int STATUS_LED_PIN = LED_BUILTIN;
#else
const int STATUS_LED_PIN = 13;  // 상태 LED용 (Neopixel과 분리)
#endif

// 구독자 콜백 함수 (최대한 단순화)
void subscription_callback(const void * msgin)
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
  
  // 문자열 복사 제거 (메모리 절약)
  // strncpy(last_received_status, received_msg->data.data, sizeof(last_received_status) - 1);
  
  new_message_received = true;
  last_heartbeat = millis();
  
  // digitalWrite 제거 (시간 절약)
  // digitalWrite(STATUS_LED_PIN, HIGH);
  // digitalWrite(STATUS_LED_PIN, LOW);
}

// micro-ROS 에러 핸들러
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  Serial.println("💥 micro-ROS 치명적 오류 발생!");
  while(1){
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    delay(100);
  }
}

// WiFi 연결 함수
bool connectWiFi() {
  Serial.print("📡 WiFi 연결 중... SSID: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  const int max_attempts = 20;
  
  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("");
    Serial.println("✅ WiFi 연결 성공!");
    Serial.print("📶 IP 주소: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    wifi_connected = false;
    Serial.println("");
    Serial.println("❌ WiFi 연결 실패!");
    return false;
  }
}

// micro-ROS 연결 함수 (도메인 ID 26 설정)
bool connectMicroROS() {
  Serial.println("🔧 micro-ROS 연결 시도 중...");
  
  // micro-ROS transport 설정
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  
  allocator = rcl_get_default_allocator();
  
  // init options 설정 (도메인 ID 26)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ init options 초기화 실패: %d\n", ret);
    return false;
  }
  
  // 도메인 ID 26 설정
  ret = rcl_init_options_set_domain_id(&init_options, 26);
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
  ret = rclc_node_init_default(&node, "neopixel_subscriber", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ 노드 생성 실패: %d\n", ret);
    return false;
  }
  
  // 구독자 생성
  ret = rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "led_status");
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ 구독자 생성 실패: %d\n", ret);
    return false;
  }
  
  // executor 생성 (로드셀과 동일)
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ executor 생성 실패: %d\n", ret);
    return false;
  }
  
  // 메시지 버퍼 초기화 (정적 할당)
  msg.data.data = msg_buffer;
  msg.data.size = 0;
  msg.data.capacity = sizeof(msg_buffer);
  
  // 구독자를 executor에 추가
  ret = rclc_executor_add_subscription(&executor, &subscription, &msg, &subscription_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial.printf("❌ 구독자 추가 실패: %d\n", ret);
    return false;
  }
  
  microros_connected = true;
  
  Serial.println("✅ micro-ROS 연결 성공!");
  Serial.println("🌐 도메인 ID: 26 설정됨");
  Serial.printf("💾 메시지 버퍼: %d bytes 할당됨\n", sizeof(msg_buffer));
  return true;
}

// LED 처리 함수
void processLEDUpdate() {
  if (new_message_received) {
    new_message_received = false;
    
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

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== ESP32 micro-ROS Neopixel LED Controller ===");
  
  // 상태 LED 핀 설정
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  // LED 초기화 (최종 활성화)
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show(); // 모든 LED 끄기
  
  Serial.println("🌈 WS2812 LED Strip 초기화 완료! (Adafruit_NeoPixel)");
  
  // 초기 LED 테스트 (임시 비활성화)
  Serial.println("🧪 LED 테스트 중...");
  // setSolidColor(255, 0, 0);  // 빨간색
  // delay(300);
  // setSolidColor(0, 255, 0);  // 초록색
  // delay(300);
  // setSolidColor(0, 0, 255);  // 파란색
  // delay(300);
  // turnOffAll();
  
  // WiFi 연결
  Serial.println("📡 WiFi 연결 시작...");
  if (!connectWiFi()) {
    Serial.println("❌ WiFi 연결 실패로 인해 종료합니다.");
    while(1) {
      setSolidColor(255, 0, 0);
      delay(500);
      turnOffAll();
      delay(500);
    }
  }
  
  // micro-ROS 연결 시도
  delay(3000);
  
  if (connectMicroROS()) {
    Serial.println("📡 /led_status 토픽 구독 준비 완료");
    Serial.println("=========================");
    
    setSolidColor(0, 255, 0);  // 초록색으로 연결 성공 표시
    delay(1000);
    turnOffAll();
    
    last_heartbeat = millis();
  } else {
    Serial.println("⚠️  micro-ROS 초기 연결 실패");
  }
  
  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  // 새 메시지 처리
  processLEDUpdate();
  
  // WiFi 연결 상태 확인
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi 연결 끊김. 재연결 시도...");
    wifi_connected = false;
    microros_connected = false;
    
    WiFi.disconnect();
    delay(1000);
    if (connectWiFi()) {
      delay(3000);
      connectMicroROS();
    }
    return;
  }
  
  // micro-ROS 재연결 로직
  if (!microros_connected) {
    static unsigned long last_reconnect_attempt = 0;
    if (millis() - last_reconnect_attempt > 5000) {
      Serial.println("🔄 micro-ROS 재연결 시도...");
      
      if (connectMicroROS()) {
        Serial.println("✅ micro-ROS 재연결 성공!");
        last_heartbeat = millis();
      } else {
        Serial.println("❌ micro-ROS 재연결 실패");
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
  
  // micro-ROS executor 실행 (타임아웃 없음)
  if (microros_connected) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));  // 10ms로 단축
    
    // 타임아웃은 정상이므로 에러로 처리하지 않음
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
      static unsigned long last_error_log = 0;
      if (millis() - last_error_log > 5000) {  // 5초마다만 로그
        Serial.printf("⚠️  Executor 에러: %d\n", ret);
        last_error_log = millis();
      }
      // 에러가 있어도 즉시 재연결하지 않고 계속 시도
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
  
  // 시리얼 명령어 처리
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte == 'w') {
      Serial.println("=== 시스템 상태 ===");
      Serial.printf("📶 WiFi: %s", WiFi.status() == WL_CONNECTED ? "연결됨" : "연결 끊김");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf(" (%s)", WiFi.localIP().toString().c_str());
      }
      Serial.println();
      Serial.printf("🤖 micro-ROS: %s\n", microros_connected ? "연결됨" : "연결 끊김");
      Serial.printf("💓 마지막 하트비트: %lu ms 전\n", millis() - last_heartbeat);
      Serial.printf("🎨 현재 LED 상태: %d\n", led_status);
      Serial.println("==================");
    }
    else if (inByte == 'm') {
      Serial.println("🔄 micro-ROS 수동 재연결 시도...");
      if (connectMicroROS()) {
        last_heartbeat = millis();
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
  }
  
  delay(10);  // 더 빠른 루프
}

// 단색으로 모든 LED 설정하는 함수 (최종 활성화)
void setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
  Serial.printf("🎨 실제 LED 제어! 색상: R=%d, G=%d, B=%d\n", r, g, b);
}

// LED 모두 끄는 함수
void turnOffAll() {
  setSolidColor(0, 0, 0);
}