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
  # === 메시지 파일 ===
  "msg/BookInfo.msg"             # 도서 정보 메시지
  "msg/DetectionStatus.msg"      # 장애물감지상태
  "msg/DetectionTimer.msg"       # 감지 타이머 메시지 추가
  "msg/FaceExpression.msg"       # 표정 메시지 추가
  "msg/GoalPose.msg"             # 이름 중복을 피하기 위해 GoalPose.msg로 변경
  "msg/Heartbeat.msg"            # 하트비트 메시지
  "msg/HumanInfo.msg"            # 휴먼타입지정
  "msg/OverallStatus.msg"        # OverallStatus 메시지 추가
  "msg/TalkCommand.msg"          # 음성 명령 메시지
  "msg/TaskStatus.msg"           # 작업 상태 메시지
  "msg/VoiceCommand.msg"         # 음성 인식 결과 메시지
  "msg/Waypoint.msg"             # 지도 만든 위치들이 포함된 메시지 추가
  # === 서비스 파일 ===
  "srv/ActivateDetector.srv"     # OR 스캐너 활성화 서비스
  "srv/ActivateQRScanner.srv"    # QR 스캐너 활성화 서비스
  "srv/ActivateTalker.srv"       # 토커 활성화 서비스
  "srv/ActivateTracker.srv"      # Tracker 활성화 서비스
  "srv/AddGoalLocation.srv"      # 목표 위치 추가 서비스
  "srv/BookSearch.srv"           # 도서 검색 서비스
  "srv/CancelNavigation.srv"     # 네비게이션 취소 서비스
  "srv/DeactivateDetector.srv"   # OR 스캐너 비활성화 서비스
  "srv/DeactivateQRScanner.srv"  # QR 스캐너 비활성화 서비스
  "srv/DeactivateTalker.srv"     # 토커 비활성화 서비스
  "srv/DeactivateTracker.srv"    # Tracker 비활성화 서비스
  "srv/EndTask.srv"              # 작업 종료 서비스
  "srv/EscortRequest.srv"        # 에스코트 요청 서비스
  "srv/KioskQRCheck.srv"         # 키오스크 QR 코드 확인 서비스
  "srv/Navigate.srv"             # 네비게이션 서비스
  "srv/NavigationResult.srv"     # 네비게이션 결과 반환서비스
  "srv/RobotQRCheck.srv"         # 로봇 QR 코드 확인 서비스
  "srv/SetGoal.srv"              # 목표 좌표 설정 서비스
  "srv/TaskRequest.srv"          # 네비게이션 요청 서비스

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
