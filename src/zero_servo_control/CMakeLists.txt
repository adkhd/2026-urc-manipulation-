cmake_minimum_required(VERSION 3.8)
project(zero_servo_control)

# 권장: C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# === JoyToServoPub 컴포넌트가 필요로 하는 의존성들 ===
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)


# (선택) moveit_servo에 종속적인 설정/타입을 쓸 경우
find_package(moveit_servo REQUIRED)

# ------------------------------------------
# ① 기존 실행파일 (그대로 유지)
# ------------------------------------------
add_executable(keyboard_teleop_node src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop_node
  rclcpp
  geometry_msgs
)


# ------------------------------------------
# ② JoyToServoPub 컴포넌트 라이브러리 추가
#    (※ 파일 경로/이름 네 환경에 맞게 수정)
# ------------------------------------------
add_library(joy_to_servo_component SHARED
  src/JoytoServo.cpp
)

ament_target_dependencies(joy_to_servo_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  control_msgs
  moveit_msgs
  std_srvs
)

# rclcpp_components 에 등록 (플러그인 이름은 cpp의 매크로와 "완전히 동일"해야 함)
# cpp 끝에:  RCLCPP_COMPONENTS_REGISTER_NODE(zero_servo::JoyToServoPub)
# ⇒ 여기 문자열도 "zero_servo::JoyToServoPub" 로 맞춘다.
rclcpp_components_register_nodes(joy_to_servo_component "zero_servo::JoyToServoPub")

# 설치 (lib 로)
install(TARGETS joy_to_servo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# (필요시) include / launch / config 설치
# install(DIRECTORY include/ DESTINATION include)
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ------------------------------------------
# 테스트 (그대로)
# ------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
