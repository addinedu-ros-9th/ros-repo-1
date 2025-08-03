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
  # === msg ===
  "msg/BookInfo.msg"
  "msg/DetectionTimer.msg"      # DetectionTimer 메시지 추가
  "msg/GoalPose.msg"            # 이름 충돌을 피하기 위해 GoalPose.msg로 변경
  "msg/Heartbeat.msg"
  "msg/OverallStatus.msg"       # OverallStatus 메시지 추가
  "msg/TaskStatus.msg"
  "msg/TalkCommand.msg"
  "msg/VoiceCommand.msg"        # 음성 명령 메시지
  "msg/Waypoint.msg"            # 새로 만든 웨이포인트 메시지 추가

  # === srv ===
  "srv/ActivateDetector.srv"
  "srv/ActivateQRScanner.srv"   # QR 스캐너 활성화 서비스
  "srv/ActivateTalker.srv"
  "srv/ActivateTracker.srv"
  "srv/BookSearch.srv"
  "srv/CancelNavigation.srv"    # 네비게이션 취소 서비스
  "srv/DeactivateDetector.srv"
  "srv/DeactivateQRScanner.srv" # QR 스캐너 비활성화 서비스
  "srv/DeactivateTalker.srv"
  "srv/EscortRequest.srv"
  "srv/EndTask.srv"
  "srv/Navigate.srv"
  "srv/NavigationResult.srv"    # 네비게이션 결과 서비스
  "srv/RobotQRCheck.srv"
  "srv/SetGoal.srv"
  "srv/TaskRequest.srv"

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
