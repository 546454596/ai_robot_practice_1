cmake_minimum_required(VERSION 3.1)

project(ai_robot_interfaces)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  MpTraj.msg
  TopoMetric.msg
)

add_service_files(
  FILES
  restart_nav.srv
  set_destination.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
