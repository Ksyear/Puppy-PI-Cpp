cmake_minimum_required(VERSION 3.16)

project(puppypi_workspace_bootstrap LANGUAGES NONE)

# Keep CMake's own files separate from colcon's build/ directory.
if(CMAKE_BINARY_DIR STREQUAL "${CMAKE_SOURCE_DIR}/build")
  message(FATAL_ERROR
    "'build/' is reserved for colcon. Configure with: "
    "cmake -S . -B cmake-build")
endif()

find_program(BASH_EXECUTABLE bash REQUIRED)

add_custom_target(robot
  COMMAND "${BASH_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/setup_from_scratch.sh" robot
  COMMAND "${BASH_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/build_all.sh"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  USES_TERMINAL
  COMMENT "Provision Ubuntu 22.04/ROS2 Humble and build the PuppyPi workspace"
  VERBATIM
)

add_custom_target(robot_deps
  COMMAND "${BASH_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/setup_from_scratch.sh" robot
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  USES_TERMINAL
  COMMENT "Install ROS2 Humble, LD19 driver, dependencies and udev rules"
  VERBATIM
)

add_custom_target(workspace_build
  COMMAND "${BASH_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/build_all.sh"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  USES_TERMINAL
  COMMENT "Build the PuppyPi ROS2 workspace"
  VERBATIM
)

add_custom_target(workspace_test
  COMMAND "${BASH_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test_all.sh"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  USES_TERMINAL
  COMMENT "Run the offline test suite"
  VERBATIM
)
