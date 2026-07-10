cmake_minimum_required(VERSION 3.8)
project(puppy_control_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(puppy_control_msgs REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

include_directories(include)

# puppy_control 노드 = ROS 래퍼(puppy_control_node.cpp) + 보행 엔진(stub_engine.cpp)
# 엔진 구현이 완성되면 stub_engine.cpp 를 실제 구현 파일로 교체하면 된다
add_executable(puppy_control
  src/puppy_control_node.cpp
  src/stub_engine.cpp
)
ament_target_dependencies(puppy_control
  rclcpp std_msgs std_srvs geometry_msgs sensor_msgs
  puppy_control_msgs ros_robot_controller_msgs
)

install(TARGETS puppy_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
