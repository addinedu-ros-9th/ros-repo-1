#include <FastLED.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

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
    setSolidColor(0, 255, 0);  // 초록색
    Serial.println("Setting LED to GREEN (슬픔)");
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
  while(1){
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  
  // FastLED 초기화
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
  Serial.println("WS2812 LED Strip initialized!");
  
  // micro-ROS 초기화
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
  
  Serial.println("micro-ROS subscriber initialized!");
}

void loop() {
  // micro-ROS 실행
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
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