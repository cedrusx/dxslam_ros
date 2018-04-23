cmake_minimum_required(VERSION 2.4.6)
project(maslam)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
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

#find_package(ORB_SLAM2 REQUIRED)
#if(NOT ORB_SLAM2_FOUND)
##   message(FATAL_ERROR "ORB_SLAM2 not found.")
#endif()

if(EXISTS ${CMAKE_SOURCE_DIR}/maslam) # catkin_make
  set(MASLAM_DIR ${CMAKE_SOURCE_DIR}/maslam)
else() #catkin build
  set(MASLAM_DIR ${CMAKE_SOURCE_DIR})
endif()
set(ORB_SLAM2_SEARCH_PATH "${MASLAM_DIR}/../../../;${MASLAM_DIR}/../../../../")
find_path(ORB_SLAM2_DIR
  ORB_SLAM2/include/System.h
  ${ORB_SLAM2_SEARCH_PATH}
  )
if(NOT ORB_SLAM2_DIR)
  message(FATAL_ERROR "ORB_SLAM2 not found under " ${ORB_SLAM2_SEARCH_PATH})
else()
  set(ORB_SLAM2_TOP_DIR ${ORB_SLAM2_DIR}/ORB_SLAM2/)
  message(STATUS "Found ORB_SLAM2: " ${ORB_SLAM2_TOP_DIR})
endif()
set(ORB_SLAM2_LIBRARIES
  ${ORB_SLAM2_TOP_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM2_TOP_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM2_TOP_DIR}/lib/libORB_SLAM2.so
)

if(EXISTS ${CMAKE_SOURCE_DIR}/maslam) # catkin_make
  add_definitions(-DORB_SLAM2_PATH='"${ORB_SLAM2_TOP_DIR}"')
  add_definitions(-DCONFIG_PATH='"${PROJECT_SOURCE_DIR}/config"')
else() #catkin build
  add_definitions(-DORB_SLAM2_PATH="${ORB_SLAM2_TOP_DIR}")
  add_definitions(-DCONFIG_PATH="${PROJECT_SOURCE_DIR}/config")
endif()

catkin_package(
    CATKIN_DEPENDS
        roscpp
    DEPENDS Eigen
)

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

install(
    TARGETS
        maslam
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
