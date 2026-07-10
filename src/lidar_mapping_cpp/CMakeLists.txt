cmake_minimum_required(VERSION 3.8)
project(lidar_mapping_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(lidar_mapping src/lidar_mapping_node.cpp)
ament_target_dependencies(lidar_mapping
  rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs tf2_ros)

install(TARGETS lidar_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# colcon test: 가상 8x8m 방 시뮬레이션으로 스캔매칭/지도 정확도 회귀 검사
if(BUILD_TESTING)
  add_executable(test_mapping_core test/test_mapping_core.cpp)
  add_test(NAME test_mapping_core COMMAND test_mapping_core)
endif()

ament_package()
