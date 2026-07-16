cmake_minimum_required(VERSION 3.0.2)
project(puppy_vr_control_noetic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  puppy_control
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  geometry_msgs nav_msgs puppy_control rospy sensor_msgs std_msgs std_srvs tf
)

catkin_install_python(PROGRAMS
  scripts/vr_udp_teleop.py
  scripts/camera_udp_sender.py
  scripts/robot_status_sender.py
  scripts/lidar_mapping.py
  scripts/mapping_run_recorder.py
  scripts/retro_effect.py
  scripts/maze_autonomy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README_MAZE.md README_MAPPING_VALIDATION.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
