cmake_minimum_required(VERSION 3.1)

project(ai_robot_navigation)

set(CMAKE_CXX_STANDARD 14)

# Fix geographiclib cmake file not found
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io visualization)
find_package(catkin REQUIRED COMPONENTS
  # ai_robot_interfaces already includs std_msgs, geometry_msgs, nav_msgs and sensor_msgs required
  ai_robot_interfaces
  cv_bridge
  image_transport
  mavros_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
)

catkin_package()

if (OpenCV_VERSION_MAJOR EQUAL 4)
  add_compile_definitions(OPENCV2)
endif()

add_definitions(${PCL_DEFINITIONS})

include_directories(include
  src/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(brain_node
  src/brain/main.cpp
  src/brain/ai_brain.cpp
  src/brain/ar_drone.cpp
  src/brain/nav_data.cpp
  src/brain/drone_object_ros.cpp
  src/brain/pioneer_3at.cpp
  src/brain/pos_control.cpp
  src/brain/pos_trajectory.cpp
  src/findpath/findpath_srm.cpp
  src/point_to_point/point_to_point.cpp
  src/utils/adrc.cpp
  src/utils/eso.cpp
  src/utils/math_aux.cpp
  src/utils/pid_controller.cpp
  src/utils/pid_fuzzy.cpp
)

add_dependencies(brain_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(brain_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(pose_ekf_node
  src/pose_ekf/main.cpp
  src/pose_ekf/pose_ekf.cpp
  src/pose_ekf/vicon.cpp
  src/utils/math_aux.cpp
)

target_link_libraries(pose_ekf_node
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_executable(obsavoid_node
  src/obsavoid/main.cpp
  src/obsavoid/p3at_obsavoid.cpp
  src/brain/pioneer_3at.cpp
  src/utils/math_aux.cpp
)

add_dependencies(obsavoid_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(obsavoid_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(pointcloud_viewer_node
  src/obsavoid/pointcloud_viewer.cpp
)

target_link_libraries(pointcloud_viewer_node
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(gennodemap_node
  src/gennodemap/src/keyframehandle.cpp
  src/gennodemap/src/ros_main.cpp
)

target_link_libraries(gennodemap_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(test_findpath
  src/findpath/test_findpath.cpp
  src/findpath/findpath_srm.cpp
  src/utils/math_aux.cpp
)

target_link_libraries(test_findpath
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(set_destination_client
  src/findpath/set_destination_client.cpp
)

target_link_libraries(set_destination_client
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    brain_node
    gennodemap_node
    obsavoid_node
    pointcloud_viewer_node
    pose_ekf_node
    set_destination_client
    test_findpath
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
