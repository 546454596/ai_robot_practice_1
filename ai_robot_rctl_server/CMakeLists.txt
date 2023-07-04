cmake_minimum_required(VERSION 3.1)

project(ai_robot_rctl_server)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  std_msgs
  roscpp
)

add_message_files(
  FILES
  RctlMapMetaData.msg
)

generate_messages(DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  nav_msgs
  std_msgs
)

add_executable(rctl_server
  src/main.cpp
  src/rctl_server.cpp
)

target_include_directories(rctl_server PRIVATE ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(rctl_server PRIVATE Boost::filesystem ${catkin_LIBRARIES})
add_dependencies(rctl_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS rctl_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
