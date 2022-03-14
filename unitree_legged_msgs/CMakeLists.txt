cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  Go1/BmsState.msg
  Go1/MotorCmd.msg
  Go1/MotorState.msg
  Go1/Cartesian.msg
  Go1/IMU.msg
  Go1/LED.msg
  Go1/LowCmd.msg
  Go1/LowState.msg
  Go1/HighCmd.msg
  Go1/HighState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs 
)

#############
## Install ##
#############

# Mark topic names header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)