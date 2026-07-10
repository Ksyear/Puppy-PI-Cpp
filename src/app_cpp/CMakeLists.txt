cmake_minimum_required(VERSION 3.8)
project(app_cpp)

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
find_package(puppy_control_msgs REQUIRED)

# LiDAR 앱: /scan → /cmd_vel_nav (모드: 장애물 회피/추적/경비)
add_executable(lidar src/lidar.cpp)
ament_target_dependencies(lidar
  rclcpp geometry_msgs sensor_msgs std_srvs puppy_control_msgs)

install(TARGETS lidar DESTINATION lib/${PROJECT_NAME})

ament_package()
