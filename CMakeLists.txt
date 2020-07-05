cmake_minimum_required(VERSION 2.4.6)
project(dxslam_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    image_feature_msgs
    cv_bridge
    message_filters
    tf
  )

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
   message(FATAL_ERROR "Eigen3 not found.")
endif()

find_package(Pangolin REQUIRED)

set(DXSLAM_SEARCH_PATH "${PROJECT_SOURCE_DIR};${PROJECT_SOURCE_DIR}/../../;${PROJECT_SOURCE_DIR}/../../../;${PROJECT_SOURCE_DIR}/../../../../")
find_path(DXSLAM_DIR
  dxslam/include/System.h
  ${DXSLAM_SEARCH_PATH}
  )
if(NOT DXSLAM_DIR)
  message(FATAL_ERROR "DXSLAM not found under " ${DXSLAM_SEARCH_PATH})
else()
  set(DXSLAM_TOP_DIR ${DXSLAM_DIR}/dxslam/)
  message(STATUS "Found DXSLAM: " ${DXSLAM_TOP_DIR})
  add_definitions(-DDXSLAM_PATH="${DXSLAM_TOP_DIR}")
  add_definitions(-DCONFIG_PATH="${PROJECT_SOURCE_DIR}/config")
endif()
set(DXSLAM_LIBRARIES
  ${DXSLAM_TOP_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${DXSLAM_TOP_DIR}/lib/libDXSLAM.so
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        image_feature_msgs
    DEPENDS Eigen
)

include_directories(
  ${DXSLAM_TOP_DIR}/Thirdparty/fbow/src
  ${catkin_INCLUDE_DIRS}
  ${DXSLAM_TOP_DIR}
  ${DXSLAM_TOP_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${DXSLAM_LIBRARIES}
)

# Node for RGB-D camera
add_executable(dxslam
  src/slam_node.cpp
)

target_link_libraries(dxslam
  ${LIBS}
)

install(
    TARGETS
        dxslam
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
