cmake_minimum_required(VERSION 3.1)

project(ai_robot_interface)

set(CMAKE_CXX_STANDARD 14)

set(MSG_DEP_SET std_msgs geometry_msgs nav_msgs sensor_msgs)

find_package(catkin REQUIRED COMPONENTS ${MSG_DEP_SET})

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

generate_messages(
  DEPENDENCIES
  ${MSG_DEP_SET}
)

catkin_package(
  CATKIN_DEPENDS
  ${MSG_DEP_SET}
  message_runtime
)
