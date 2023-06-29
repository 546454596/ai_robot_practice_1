cmake_minimum_required(VERSION 3.1)

project(ai_robot_sensors)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  razor_imu_9dof
  ublox_gps
  ublox_msgs
  ublox_serialization
  zed_wrapper
)
