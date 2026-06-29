cmake_minimum_required(VERSION 3.0.2)
project(summit_xl_autonomous_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  actionlib
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/structured_mapper.py
  scripts/exploration_monitor.py
  scripts/exploration_utils.py
  scripts/click_goal.py
  scripts/navigation_status.py
  scripts/set_initial_pose_from_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
