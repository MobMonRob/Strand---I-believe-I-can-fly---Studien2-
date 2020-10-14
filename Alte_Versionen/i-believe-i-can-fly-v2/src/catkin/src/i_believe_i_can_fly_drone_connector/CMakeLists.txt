cmake_minimum_required(VERSION 2.8.3)
project(i_believe_i_can_fly_drone_connector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ardrone_autonomy
  i_believe_i_can_fly_common
  i_believe_i_can_fly_pose_detection
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  ardrone_autonomy
  i_believe_i_can_fly_common
  i_believe_i_can_fly_pose_detection
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
