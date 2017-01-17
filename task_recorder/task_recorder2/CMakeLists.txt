cmake_minimum_required(VERSION 2.8.3)
project(task_recorder2)

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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  tf
  orocos_kdl
  filters
  message_filters
  sensor_msgs
  angles
  usc_utilities
  task_recorder2_msgs
  task_recorder2_utilities
  dynamic_movement_primitive
  message_generation
  cmake_modules
  rosconsole
  genmsg
)

find_package(Orocos_kdl REQUIRED)
include_directories(${Orocos_kdl_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${Boost_DEFINITIONS})

add_service_files(FILES
	GetDataSample.srv
	InterruptRecording.srv
	StartRecording.srv
        StartStreaming.srv
	StopRecording.srv)

# rosbuild_genmsg()
generate_messages(DEPENDENCIES task_recorder2_msgs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME} task_recorder2_manager task_recorder2_manager_client
               CATKIN_DEPENDS roscpp rosbag tf filters message_filters sensor_msgs angles usc_utilities task_recorder2_msgs task_recorder2_utilities dynamic_movement_primitive message_runtime cmake_modules rosconsole 
               DEPENDS Boost Eigen3 Eigen Orocos_kdl)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/joint_states_recorder.cpp
	src/pose_recorder.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_recorder_node
  src/task_recorder_node.cpp
)
target_link_libraries(task_recorder_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(task_recorder_node ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(task_recorder2_manager
  src/task_recorder_manager.cpp
)
target_link_libraries(task_recorder2_manager ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(task_recorder2_manager ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_recorder_manager_node
  src/task_recorder_manager_node.cpp
)
target_link_libraries(task_recorder_manager_node ${catkin_LIBRARIES} ${PROJECT_NAME} task_recorder2_manager)

add_dependencies(task_recorder_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${task_recorder2_manager_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


add_library(task_recorder2_manager_client
  src/task_recorder_manager_client.cpp
)
target_link_libraries(task_recorder2_manager_client task_recorder2_manager ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(task_recorder2_manager_client ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_labeler_node
  src/task_labeler_node.cpp
)
target_link_libraries(task_labeler_node ${catkin_LIBRARIES})# ${PROJECT_NAME} task_recorder2_manager task_recorder2_manager_client)
add_dependencies(task_labeler_node ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_task_recorder_manager_node
  test/test_task_recorder_manager.cpp
)
target_link_libraries(test_task_recorder_manager_node ${PROJECT_NAME} task_recorder2_manager task_recorder2_manager_client)
add_dependencies(test_task_recorder_manager_node ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_task_recorder_node
  test/test_task_recorder_node.cpp
)
target_link_libraries(test_task_recorder_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(test_task_recorder_node ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
