cmake_minimum_required(VERSION 3.8)
project(puppy_vr_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(puppy_control_msgs REQUIRED)

include_directories(include)

# VR(Quest) UDP 조이스틱 수신 → /puppy_control/velocity/autogait 발행
add_executable(vr_udp_teleop src/vr_udp_teleop.cpp)
ament_target_dependencies(vr_udp_teleop rclcpp std_srvs puppy_control_msgs)

# /image_raw/compressed(JPEG) → UDP 청크 전송 (Quest에서 영상 수신용)
add_executable(camera_udp_sender src/camera_udp_sender.cpp)
ament_target_dependencies(camera_udp_sender rclcpp sensor_msgs)

# 배터리/Wi-Fi/링크 상태 → UDP 1Hz 전송 (Quest HUD 표시용)
add_executable(robot_status_sender src/robot_status_sender.cpp)
ament_target_dependencies(robot_status_sender rclcpp std_msgs)

install(TARGETS
  vr_udp_teleop
  camera_udp_sender
  robot_status_sender
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# colcon test 로 실행되는 순수 로직 테스트 (ROS 불필요)
if(BUILD_TESTING)
  add_executable(test_vr_logic test/test_vr_logic.cpp)
  add_test(NAME test_vr_logic COMMAND test_vr_logic)
endif()

ament_package()
