project(gennodemap)
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_FLAGS -std=c++11)
# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
if( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
endif()

find_package(Boost)
find_package(PCL REQUIRED COMPONENTS common io filters)
FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization )
find_package(OpenCV REQUIRED)
#find_package(eigen)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  ADD_EXECUTABLE(gennodemap
    src/main.cpp
    src/keyframehandle.cpp
  )

  target_link_libraries(gennodemap
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )
endif()

