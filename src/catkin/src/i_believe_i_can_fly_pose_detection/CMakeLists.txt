cmake_minimum_required(VERSION 2.8.3)
project(i_believe_i_can_fly_pose_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  i_believe_i_can_fly_common
  i_believe_i_can_fly_person_detection
)

add_message_files(
  FILES
    Calibration.msg
    Instruction.msg
    Instructions.msg
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
  i_believe_i_can_fly_common
  i_believe_i_can_fly_person_detection
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
