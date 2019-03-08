cmake_minimum_required(VERSION 2.8.3)
project(pose_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  person_detection
)

add_message_files(
  FILES
    Instruction.msg
    Instructions.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime person_detection
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
