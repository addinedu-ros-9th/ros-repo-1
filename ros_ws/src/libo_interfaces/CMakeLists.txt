cmake_minimum_required(VERSION 3.8)
project(libo_interfaces)

# 필요한 패키지들을 찾습니다.
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 파일을 기반으로 인터페이스를 생성합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heartbeat.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# 테스트 관련 설정을 찾습니다.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 설정을 마무리합니다.
ament_package()