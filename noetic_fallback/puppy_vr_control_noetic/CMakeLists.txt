cmake_minimum_required(VERSION 3.0.2)
project(puppy_vr_control_noetic)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vr_udp_teleop.py
  scripts/camera_udp_sender.py
  scripts/robot_status_sender.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
