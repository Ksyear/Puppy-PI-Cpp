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

# 공개 저장소에는 stub 만 둔다. 사용자가 보유한 Hiwonder 이미지로부터 작성한
# 비공개 C++ 엔진은 private/src/*.cpp 에 두며, 이 디렉터리는 .gitignore 대상이다.
file(GLOB PRIVATE_ENGINE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/private/src/*.cpp")
if(PRIVATE_ENGINE_SOURCES)
  set(PUPPY_ENGINE_SOURCES ${PRIVATE_ENGINE_SOURCES})
  message(STATUS "puppy_control_cpp: using local private PuppyEngine")
else()
  set(PUPPY_ENGINE_SOURCES src/stub_engine.cpp)
  message(STATUS "puppy_control_cpp: private engine not found; using non-driving stub")
endif()

add_executable(puppy_control
  src/puppy_control_node.cpp
  ${PUPPY_ENGINE_SOURCES}
)
if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/private/include")
  target_include_directories(puppy_control PRIVATE private/include)
endif()
ament_target_dependencies(puppy_control
  rclcpp std_msgs std_srvs geometry_msgs sensor_msgs
  puppy_control_msgs ros_robot_controller_msgs
)

install(TARGETS puppy_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
