cmake_minimum_required(VERSION 3.0.2)
project(lab4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/pole_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

