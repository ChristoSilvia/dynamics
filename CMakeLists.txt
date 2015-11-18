cmake_minimum_required(VERSION 2.8.3)
project(dynamics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
)

add_service_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

# build

include_directories(
  ${catkin_INCLUDE_DIRS}
)
