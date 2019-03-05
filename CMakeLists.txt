PROJECT(slam_backend)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling SLAM Backend")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# TODO: Joydeep, are you also getting this '__float128 not implemented'
# compilation bug? I am fixing it using strict ansi but I don't love this
# solution.
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror -D__STRICT_ANSI__")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

SET(libs roslib roscpp rosbag glog X11 pthread gflags ${CERES_LIBRARIES})

ADD_LIBRARY(shared_library
            src/gui_helpers.cc
            src/timer.cc)

ROSBUILD_ADD_EXECUTABLE(slam-backend
                        src/slam-backend-solver.cc
                        src/slam-backend-main.cc)
TARGET_LINK_LIBRARIES(slam-backend shared_library ${libs})

ADD_EXECUTABLE(eigen-tutorial
               src/eigen_tutorial.cc)
