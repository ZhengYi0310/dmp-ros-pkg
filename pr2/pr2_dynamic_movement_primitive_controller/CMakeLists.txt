cmake_minimum_required(VERSION 2.4.6)
project(pr2_dynamic_movement_primitive_controller)

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
  rosrt
  filters
  geometry_msgs
  std_msgs
  pluginlib
  control_toolbox
  pr2_controller_interface
  pr2_mechanism_model
  pr2_controllers_msgs
  dynamic_movement_primitive
  usc_utilities
  robot_info
)

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${Boost_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})

add_message_files(
   FILES
   JointPositionVelocityStamped.msg
   NullspaceTermStamped.msg
   PoseTwistStamped.msg
   Waypoint.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS include 
               LIBRARIES ${PROJECT_NAME} 
               CATKIN_DEPENDS roscpp rosrt filters geometry_msgs std_msgs pluginlib control_toolbox pr2_controller_interface pr2_mechanism_model pr2_controllers_msgs dynamic_movement_primitive usc_utilities robot_info message_runtime    
               DEPENDS Eigen Eigen3 Boost orocos_kdl)

#common commands for building c++ executables and libraries

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/dmp_joint_position_controller.cpp
  src/joint_position_controller.cpp
  src/dmp_ik_controller.cpp
#  src/dmp_dual_ik_controller.cpp
  src/variable_name_map.cpp
  src/cartesian_twist_controller_ik_with_nullspace_optimization.cpp 
)
target_link_libraries(${PROJECT_NAME}  ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES}) 

add_executable(test_dmp_joint_position_controller
  test/test_dmp_joint_position_controller.cpp
)
target_link_libraries(test_dmp_joint_position_controller ${PROJECT_NAME})
add_dependencies(test_dmp_joint_position_controller ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) 

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
