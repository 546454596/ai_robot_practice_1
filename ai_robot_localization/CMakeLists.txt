cmake_minimum_required(VERSION 3.1)

project(ai_robot_localization)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  cartographer_ros
)

catkin_install_python(PROGRAMS nodes/init_carto_loc nodes/posepub
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
