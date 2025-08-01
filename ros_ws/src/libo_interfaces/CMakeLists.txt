cmake_minimum_required(VERSION 3.8)
project(libo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 및 서비스 파일 추가
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BookInfo.msg"
  "msg/Heartbeat.msg"
  "msg/OverallStatus.msg"  # OverallStatus 메시지 추가
  "srv/BookSearch.srv"
  "srv/Navigate.srv"
  "srv/EscortRequest.srv"
  "srv/TaskRequest.srv"
  "msg/GoalPose.msg"  # 이름 충돌을 피하기 위해 GoalPose.msg로 변경
  "msg/Waypoint.msg"  # 새로 만든 웨이포인트 메시지 추가
  "srv/SetGoal.srv"
  "msg/VoiceCommand.msg"
  "srv/CancelNavigation.srv"
  "srv/NavigationResult.srv"

  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
