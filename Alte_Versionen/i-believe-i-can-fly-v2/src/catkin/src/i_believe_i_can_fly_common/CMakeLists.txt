cmake_minimum_required(VERSION 2.8.3)
project(i_believe_i_can_fly_common)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
    Reset.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
