cmake_minimum_required(VERSION 2.4.6)
project(maslam)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cv_bridge
    message_filters
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

#find_package(ORB_SLAM2 REQUIRED)
#if(NOT ORB_SLAM2_FOUND)
##   message(FATAL_ERROR "ORB_SLAM2 not found.")
#endif()

set(ORB_SLAM2_SEARCH_PATH "${CMAKE_SOURCE_DIR}/../../../")
find_path(ORB_SLAM2_DIR
  ORB_SLAM2/include/System.h
  ${ORB_SLAM2_SEARCH_PATH}
  )
set(ORB_SLAM2_TOP_DIR ${ORB_SLAM2_DIR}/ORB_SLAM2/)
set(ORB_SLAM2_LIBRARIES
  ${ORB_SLAM2_TOP_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM2_TOP_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM2_TOP_DIR}/lib/libORB_SLAM2.so
)
message(STATUS "Found ORB_SLAM2: " ${ORB_SLAM2_TOP_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ORB_SLAM2_TOP_DIR}
  ${ORB_SLAM2_TOP_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBS
#  boost_system
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM2_LIBRARIES}
)

# Node for RGB-D camera
add_executable(maslam
  src/maslam.cpp
)

target_link_libraries(maslam
  ${LIBS}
)

