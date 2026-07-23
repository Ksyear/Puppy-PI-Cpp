cmake_minimum_required(VERSION 3.0.2)
project(ros1_maze_escape)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  puppy_control
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES frontier_core
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    move_base_msgs
    nav_msgs
    puppy_control
    roscpp
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(frontier_core
  src/frontier_detector.cpp
  src/heuristic_frontier_scorer.cpp
)
add_dependencies(frontier_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_core ${catkin_LIBRARIES})

add_executable(cmd_vel_adapter_node src/cmd_vel_adapter_node.cpp)
add_dependencies(cmd_vel_adapter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_adapter_node ${catkin_LIBRARIES})

add_executable(maze_brain_node src/maze_brain_node.cpp)
add_dependencies(maze_brain_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_brain_node frontier_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(velocity_adapter_test test/velocity_adapter_test.cpp)
  if(TARGET velocity_adapter_test)
    add_dependencies(velocity_adapter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(velocity_adapter_test ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(frontier_scorer_test test/frontier_scorer_test.cpp)
  if(TARGET frontier_scorer_test)
    add_dependencies(frontier_scorer_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(
      frontier_scorer_test
      frontier_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(frontier_detector_test test/frontier_detector_test.cpp)
  if(TARGET frontier_detector_test)
    add_dependencies(frontier_detector_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(
      frontier_detector_test
      frontier_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS frontier_core cmd_vel_adapter_node maze_brain_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
