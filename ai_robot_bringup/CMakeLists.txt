cmake_minimum_required(VERSION 3.1)

project(ai_robot_bringup)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)

catkin_package()

file(GLOB SCRIPTS scripts/*.sh)

install(PROGRAMS ${SCRIPTS}
  DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(DIRECTORY launch
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
