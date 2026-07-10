cmake_minimum_required(VERSION 3.8)
project(obstacle_climb_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(puppy_control_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(obstacle_climb src/obstacle_climb.cpp)
ament_target_dependencies(obstacle_climb
  rclcpp sensor_msgs cv_bridge std_srvs puppy_control_msgs)
target_link_libraries(obstacle_climb ${OpenCV_LIBS})

install(TARGETS obstacle_climb DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
