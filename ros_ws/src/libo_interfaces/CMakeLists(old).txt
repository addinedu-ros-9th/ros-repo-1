cmake_minimum_required(VERSION 3.8)
project(libo_interfaces)

# 필요한 패키지들을 찾는다
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heartbeat.msg"
  "msg/RobotStatus.msg"
)

# 테스트 의존성을 찾는다
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 설정을 마무리한다
ament_package()