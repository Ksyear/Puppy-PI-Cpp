cmake_minimum_required(VERSION 3.8)
project(peripherals_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(puppy_control_msgs REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

# 무선 게임패드 → /puppy_control/velocity/autogait, /puppy_control/pose
add_executable(remote_control_joystick src/remote_control_joystick.cpp)
ament_target_dependencies(remote_control_joystick
  rclcpp geometry_msgs std_srvs puppy_control_msgs ros_robot_controller_msgs)

# 키보드(w/a/s/d) → controller/cmd_vel
add_executable(teleop_key_control src/teleop_key_control.cpp)
ament_target_dependencies(teleop_key_control rclcpp geometry_msgs)

# IMU orientation → TF
add_executable(tf_broadcaster_imu src/tf_broadcaster_imu.cpp)
ament_target_dependencies(tf_broadcaster_imu rclcpp geometry_msgs sensor_msgs tf2_ros)

install(TARGETS
  remote_control_joystick
  teleop_key_control
  tf_broadcaster_imu
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
