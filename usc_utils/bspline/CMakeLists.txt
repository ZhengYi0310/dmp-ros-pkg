cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(bspline)
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
  std_msgs
  roscpp
)





catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
	       CATKIN_DEPENDS std_msgs roscpp
              )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/BSplineLib.cpp
)



#add_executable(usc_utilities_test
#	test/asserts_enabled_test.cpp
#	test/asserts_disabled_test.cpp
#	test/param_server_test.cpp
#	test/accumulator_test.cpp
#	test/test_main.cpp
#)
#add_dependencies(tests usc_utilities_test)
#add_rostest(launch/usc_utilities_test.test)
#target_link_libraries(usc_utilities_test usc_utilities)
#target_link_libraries(usc_utilities_test gtest)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
