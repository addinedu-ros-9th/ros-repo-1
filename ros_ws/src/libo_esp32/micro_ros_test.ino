/*
   micro-ROS 라이브러리 설치 확인 테스트 코드
*/

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== micro-ROS 라이브러리 테스트 ===");
  Serial.println("✅ micro-ROS 라이브러리가 정상적으로 설치되었습니다!");
  Serial.println("✅ 이제 weight_sensor_microros.ino를 컴파일할 수 있습니다.");
  Serial.println("================================");
}

void loop() {
  delay(1000);
  Serial.println("micro-ROS 라이브러리 정상 작동 중...");
} 