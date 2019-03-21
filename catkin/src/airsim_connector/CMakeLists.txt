cmake_minimum_required(VERSION 2.8.3)
project(airsim_connector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  airsim
  pose_detection
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs airsim pose_detection
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
