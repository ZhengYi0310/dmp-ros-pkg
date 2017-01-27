cmake_minimum_required(VERSION 2.4.6)
project(dmp_behaviors)

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
  roscpp
  geometry_msgs
  std_srvs
  actionlib
  actionlib_msgs
  usc_utilities
  dmp_behavior_actions
  tf
  angles
  task_msgs
  skill_library
  dynamic_movement_primitive
  dynamic_movement_primitive_utilities
)

find_package(Bullet REQUIRED)
include_directories(${Bullet_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(scan_for_objects
#	src/scan_for_objects_node.cpp
#	src/scan_for_objects.cpp
#)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS roscpp geometry_msgs std_srvs actionlib actionlib_msgs usc_utilities dmp_behavior_actions tf angles task_msgs skill_library dynamic_movement_primitive dynamic_movement_primitive_utilities 
               DEPENDS Bullet)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(learning_from_demonstration
	src/learning_from_demonstration_node.cpp
	src/learning_from_demonstration.cpp
)
target_link_libraries(learning_from_demonstration ${catkin_LIBRARIES})
add_dependencies(learning_from_demonstration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_dmp
	src/test_dmp_node.cpp
	src/test_dmp.cpp
)
target_link_libraries(test_dmp ${catkin_LIBRARIES})
add_dependencies(test_dmp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(learning_from_demonstration_client
	src/learning_from_demonstration_client_node.cpp
)
target_link_libraries(learning_from_demonstration_client ${catkin_LIBRARIES})
add_dependencies(learning_from_demonstration_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(example ${PROJECT_NAME})
