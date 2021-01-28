cmake_minimum_required(VERSION 2.4.6)
project(dxslam_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -Wno-ignored-attributes -O3 -march=native -std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    image_feature_msgs
    cv_bridge
    message_filters
    tf
  )

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
endif()
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
endif()
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
endif()

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
   message(FATAL_ERROR "Eigen3 not found.")
endif()

find_package(Pangolin REQUIRED)

set(SLAM_SEARCH_PATH "${PROJECT_SOURCE_DIR};${PROJECT_SOURCE_DIR}/../../;${PROJECT_SOURCE_DIR}/../../../;${PROJECT_SOURCE_DIR}/../../../../")

unset(DXSLAM_DIR CACHE)
find_path(DXSLAM_DIR
  dxslam/include/System.h
  ${SLAM_SEARCH_PATH}
  )
if(NOT DXSLAM_DIR)
  set(BUILD_DXSLAM OFF CACHE BOOL "Build DXSLAM" FORCE)
  message(WARNING "DXSLAM not found under " ${SLAM_SEARCH_PATH})
else()
  set(BUILD_DXSLAM ON CACHE BOOL "Build DXSLAM" FORCE)
  set(DXSLAM_TOP_DIR ${DXSLAM_DIR}/dxslam/)
  message(STATUS "Found DXSLAM: " ${DXSLAM_TOP_DIR})
  add_definitions(-DDXSLAM_PATH="${DXSLAM_TOP_DIR}")
  add_definitions(-DCONFIG_PATH="${PROJECT_SOURCE_DIR}/config")
  set(DXSLAM_INCLUDE_DIRS
    ${DXSLAM_TOP_DIR}/Thirdparty/fbow/src
    ${DXSLAM_TOP_DIR}
    ${DXSLAM_TOP_DIR}/include
  )
  set(DXSLAM_LIBRARIES
    ${DXSLAM_TOP_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${DXSLAM_TOP_DIR}/lib/libDXSLAM.so
  )
endif()

unset(ORBSLAM2_DIR CACHE)
find_path(ORBSLAM2_DIR
  ORB_SLAM2/include/System.h
  ${SLAM_SEARCH_PATH}
  )
if(NOT ORBSLAM2_DIR)
  set(BUILD_ORBSLAM2 OFF CACHE BOOL "Build ORB_SLAM2" FORCE)
  message(WARNING "ORB_SLAM2 not found under " ${SLAM_SEARCH_PATH})
else()
  set(BUILD_ORBSLAM2 ON CACHE BOOL "Build ORB_SLAM2" FORCE)
  set(ORBSLAM2_TOP_DIR ${ORBSLAM2_DIR}/ORB_SLAM2/)
  message(STATUS "Found ORB_SLAM2: " ${ORBSLAM2_TOP_DIR})
  add_definitions(-DORBSLAM2_PATH="${ORBSLAM2_TOP_DIR}")
  set(ORBSLAM2_INCLUDE_DIRS
    ${ORBSLAM2_TOP_DIR}
    ${ORBSLAM2_TOP_DIR}/include
  )
  set(ORBSLAM2_LIBRARIES
    ${ORBSLAM2_TOP_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORBSLAM2_TOP_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${ORBSLAM2_TOP_DIR}/lib/libORB_SLAM2.so
  )
endif()

add_definitions(-DCONFIG_PATH="${PROJECT_SOURCE_DIR}/config")

catkin_package(
    CATKIN_DEPENDS
        roscpp
        image_feature_msgs
    DEPENDS EIGEN3
)

set(COMMON_INCLUDE_DIRS
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(COMMON_LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

if(BUILD_DXSLAM)
  add_executable(dxslam src/slam_node.cpp)
  target_include_directories(dxslam PUBLIC ${COMMON_INCLUDE_DIRS} ${DXSLAM_INCLUDE_DIRS})
  target_link_libraries(dxslam ${COMMON_LIBS} ${DXSLAM_LIBRARIES})
  set_target_properties(dxslam PROPERTIES COMPILE_DEFINITIONS "BUILD_DXSLAM")
  set(INSTALL_TARGETS ${INSTALL_TARGETS} dxslam)
endif()

if(BUILD_ORBSLAM2)
  add_executable(orbslam2 src/slam_node.cpp)
  target_include_directories(orbslam2 PUBLIC ${COMMON_INCLUDE_DIRS} ${ORBSLAM2_INCLUDE_DIRS})
  target_link_libraries(orbslam2 ${COMMON_LIBS} ${ORBSLAM2_LIBRARIES})
  set_target_properties(orbslam2 PROPERTIES COMPILE_DEFINITIONS "BUILD_ORBSLAM2")
  set(INSTALL_TARGETS ${INSTALL_TARGETS} orbslam2)
endif()

install(
    TARGETS
      ${INSTALL_TARGETS}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
