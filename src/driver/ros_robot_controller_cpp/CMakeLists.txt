cmake_minimum_required(VERSION 3.8)
project(ros_robot_controller_cpp)

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
find_package(ros_robot_controller_msgs REQUIRED)

include_directories(include)

add_executable(ros_robot_controller
  src/ros_robot_controller_node.cpp
  src/board.cpp
)
ament_target_dependencies(ros_robot_controller
  rclcpp std_msgs std_srvs sensor_msgs ros_robot_controller_msgs
)

install(TARGETS ros_robot_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# colcon test: 시리얼 프레임이 파이썬 SDK 기준 바이트와 일치하는지 회귀 검사
if(BUILD_TESTING)
  add_executable(test_board_protocol test/test_board_protocol.cpp)
  add_test(NAME test_board_protocol COMMAND test_board_protocol)
endif()

ament_package()
