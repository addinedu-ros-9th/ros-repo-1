cmake_minimum_required(VERSION 3.8)
project(libo_interfaces)

<<<<<<< HEAD
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
=======
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
  "srv/BookSearch.srv"
  "srv/Navigate.srv"
  "msg/GoalPose.msg"  # 이름 충돌을 피하기 위해 GoalPose.msg로 변경
  "msg/Waypoint.msg"  # 새로 만든 웨이포인트 메시지 추가
  DEPENDENCIES builtin_interfaces
)

>>>>>>> 23b574b5f2a2530de462f9d069b923485056e7d1
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

<<<<<<< HEAD
# 패키지 설정을 마무리합니다.
ament_package()
=======
ament_package()
>>>>>>> 23b574b5f2a2530de462f9d069b923485056e7d1
