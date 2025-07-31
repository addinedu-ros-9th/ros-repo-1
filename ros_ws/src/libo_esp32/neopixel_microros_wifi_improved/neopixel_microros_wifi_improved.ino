#include <FastLED.h>
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
#define NUM_LEDS    30       // LED 개수 (실제 개수에 맞게 조정)
#define BRIGHTNESS  64       // 밝기 (0-255)
#define LED_TYPE    WS2812B  // LED 타입
#define COLOR_ORDER GRB      // 색상 순서

CRGB leds[NUM_LEDS];

// micro-ROS 설정
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscription;
rclc_executor_t executor;
std_msgs__msg__String msg;

// WiFi 연결 상태 변수
bool wifi_connected = false;
unsigned long last_wifi_check = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000; // 10초마다 체크

// 구독자 콜백 함수
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  Serial.print("Received status: ");
  Serial.println(msg->data.data);
  
  // 상태에 따른 LED 제어
  if (strcmp(msg->data.data, "기쁨") == 0) {
    setSolidColor(0, 255, 0);  // 초록색
    Serial.println("Setting LED to GREEN (기쁨)");
  } else if (strcmp(msg->data.data, "슬픔") == 0) {
    setSolidColor(0, 0, 255);  // 파란색
    Serial.println("Setting LED to BLUE (슬픔)");
  } else if (strcmp(msg->data.data, "화남") == 0) {
    setSolidColor(255, 0, 0);  // 빨간색
    Serial.println("Setting LED to RED (화남)");
  } else {
    // 알 수 없는 상태는 LED 끄기
    turnOffAll();
    Serial.println("Unknown status, turning off LED");
  }
}

// micro-ROS 에러 핸들러
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  Serial.println("micro-ROS error occurred!");
  while(1){
    delay(100);
  }
}

// WiFi 연결 함수 (개선된 버전)
bool connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  const int max_attempts = 20; // 10초 대기
  
  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    wifi_connected = true;
    return true;
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed!");
    wifi_connected = false;
    return false;
  }
}

// WiFi 연결 상태 확인 및 재연결
void checkWiFiConnection() {
  unsigned long current_time = millis();
  
  if (current_time - last_wifi_check >= WIFI_CHECK_INTERVAL) {
    last_wifi_check = current_time;
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Attempting to reconnect...");
      wifi_connected = false;
      if (connectWiFi()) {
        Serial.println("WiFi reconnected successfully!");
      } else {
        Serial.println("WiFi reconnection failed!");
      }
    } else {
      // 연결 상태 로그 (1분마다)
      static unsigned long last_status_log = 0;
      if (current_time - last_status_log >= 60000) {
        last_status_log = current_time;
        Serial.print("WiFi Status: Connected, IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(", RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== ESP32 micro-ROS Neopixel LED Controller ===");
  
  // FastLED 초기화
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
  Serial.println("WS2812 LED Strip initialized!");
  
  // 초기 LED 테스트
  Serial.println("Testing LED...");
  setSolidColor(255, 0, 0);  // 빨간색
  delay(500);
  setSolidColor(0, 255, 0);  // 초록색
  delay(500);
  setSolidColor(0, 0, 255);  // 파란색
  delay(500);
  turnOffAll();
  
  // WiFi 연결
  Serial.println("Starting WiFi connection...");
  if (connectWiFi()) {
    Serial.println("WiFi connection successful!");
  } else {
    Serial.println("WiFi connection failed! Retrying...");
    delay(2000);
    if (!connectWiFi()) {
      Serial.println("WiFi connection failed again! Check your credentials.");
    }
  }
  
  // micro-ROS 초기화
  Serial.println("Initializing micro-ROS...");
  set_microros_transports();
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  
  // 지원 객체 생성
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // 노드 생성
  RCCHECK(rclc_node_init_default(&node, "neopixel_subscriber", "", &support));
  
  // 구독자 생성
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "led_status"));
  
  // executor 생성 및 구독자 추가
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription, &msg, &subscription_callback, ON_NEW_DATA));
  
  Serial.println("micro-ROS subscriber initialized successfully!");
  Serial.println("Waiting for LED status messages...");
  
  // 연결 성공 표시
  setSolidColor(0, 255, 0);  // 초록색으로 연결 성공 표시
  delay(1000);
  turnOffAll();
}

void loop() {
  // WiFi 연결 상태 확인
  checkWiFiConnection();
  
  // micro-ROS 실행
  if (wifi_connected) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  } else {
    // WiFi가 연결되지 않은 경우 LED로 상태 표시
    static unsigned long last_blink = 0;
    if (millis() - last_blink >= 1000) {
      last_blink = millis();
      setSolidColor(255, 0, 0);  // 빨간색으로 연결 실패 표시
      delay(100);
      turnOffAll();
    }
  }
  
  delay(100);
}

// 단색으로 모든 LED 설정하는 함수
void setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
}

// LED 모두 끄는 함수
void turnOffAll() {
  setSolidColor(0, 0, 0);
}

// 페이드 효과
void fadeInOut(uint8_t r, uint8_t g, uint8_t b) {
  // 페이드 인 (밝아지기)
  for(int brightness = 0; brightness <= 255; brightness += 5) {
    setSolidColorWithBrightness(r, g, b, brightness);
    delay(30);
  }
  
  delay(500);  // 잠시 대기
  
  // 페이드 아웃 (어두워지기)
  for(int brightness = 255; brightness >= 0; brightness -= 5) {
    setSolidColorWithBrightness(r, g, b, brightness);
    delay(30);
  }
}

// 밝기를 포함한 단색 설정 함수
void setSolidColorWithBrightness(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  // 밝기 값을 0-255에서 0-100 퍼센트로 변환
  float brightnessScale = brightness / 255.0;
  
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r * brightnessScale, g * brightnessScale, b * brightnessScale);
  }
  FastLED.show();
}