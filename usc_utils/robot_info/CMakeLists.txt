cmake_minimum_required(VERSION 2.4.6)
project(robot_info)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  #bullet
  usc_utilities
  tf
  sensor_msgs
  #orocos_kdl
  kdl_parser
  urdf
)

find_package(Bullet REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Bullet_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS cmake_modules geometry_msgs usc_utilities tf sensor_msgs kdl_parser urdf
               DEPENDS Boost Bullet orocos_kdl)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(robot_info
  src/robot_info.cpp
  src/robot_info_init.cpp
  src/forward_kinematics.cpp
  src/kdl_treefksolverjointposaxis.cpp
  src/robot_monitor.cpp
  src/trajectory_timing_generator.cpp
)
target_link_libraries(robot_info ${orocos_kdl_LIBRARIES} ${Bullet_LIBRARIES} ${catkin_LIBRARIES})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
