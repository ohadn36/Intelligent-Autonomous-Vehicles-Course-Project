cmake_minimum_required(VERSION 3.0.2)
project(summit_xl_autonomous_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/waypoint_patrol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
