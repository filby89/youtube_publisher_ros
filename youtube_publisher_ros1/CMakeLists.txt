cmake_minimum_required(VERSION 3.0.2)
project(youtube_publisher_ros1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/youtube_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
