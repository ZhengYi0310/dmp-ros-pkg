cmake_minimum_required(VERSION 2.4.6)
project(dynamic_movement_primitive)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/test")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)





find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  roscpp
  usc_utilities
  rosbag
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${Boost_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories( "dmpLib/include/" )
link_directories( ${PROJECT_SOURCE_DIR}/dmpLib/lib/ )

add_message_files(
   FILES
   CanonicalSystemMsg.msg
   CanonicalSystemParametersMsg.msg
   CanonicalSystemStateMsg.msg
   ControllerStatusMsg.msg
   DMPUtilitiesMsg.msg
   DynamicMovementPrimitiveMsg.msg
   DynamicMovementPrimitiveParametersMsg.msg
   DynamicMovementPrimitiveStateMsg.msg
   NC2010CanonicalSystemMsg.msg
   NC2010CanonicalSystemParametersMsg.msg
   NC2010CanonicalSystemStateMsg.msg
   NC2010DynamicMovementPrimitiveMsg.msg
   NC2010DynamicMovementPrimitiveParametersMsg.msg
   NC2010DynamicMovementPrimitiveStateMsg.msg
   NC2010TransformationSystemMsg.msg
   NC2010TransformationSystemParametersMsg.msg
   NC2010TransformationSystemStateMsg.msg
   Model.msg
   StateMsg.msg
   TimeMsg.msg
   TransformationSystemMsg.msg
   TransformationSystemParametersMsg.msg
   TransformationSystemStateMsg.msg
   TypeMsg.msg
   

 )

generate_messages(DEPENDENCIES std_msgs sensor_msgs)
#rosbuild_gensrv()

#add_custom_target(
#   extern_lib
#   COMMAND make
#   ${PROJECT_SOURCE_DIR}/lwrLib
#)

#add_custom_target(
#   extern_lib
#   COMMAND make
#   ${PROJECT_SOURCE_DIR}/dmpLib
#)
catkin_package(INCLUDE_DIRS include "dmpLib/include/"
               LIBRARIES dynamic_movement_primitive dmp++ lwr locally_weighted_regression
               CATKIN_DEPENDS cmake_modules message_runtime roscpp usc_utilities rosbag sensor_msgs std_msgs
               DEPENDS Eigen Eigen3 Boost)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dmp++

  	dmpLib/src/dmpLib/logger.cpp
	dmpLib/src/dmpLib/trajectory.cpp
	
	dmpLib/src/dmpLib/dynamic_movement_primitive_base.cpp
	dmpLib/src/dmpLib/dynamic_movement_primitive_state_base.cpp
	dmpLib/src/dmpLib/dynamic_movement_primitive_parameters_base.cpp
        
	dmpLib/src/dmpLib/canonical_system_base.cpp
	dmpLib/src/dmpLib/canonical_system_state_base.cpp
	dmpLib/src/dmpLib/canonical_system_parameters_base.cpp
        
	dmpLib/src/dmpLib/transformation_system_base.cpp
	dmpLib/src/dmpLib/transformation_system_state_base.cpp
	dmpLib/src/dmpLib/transformation_system_parameters_base.cpp
	
	dmpLib/src/dmpLib/nc2010_canonical_system_parameters.cpp
	dmpLib/src/dmpLib/nc2010_canonical_system_state.cpp
	dmpLib/src/dmpLib/nc2010_canonical_system.cpp
	
	dmpLib/src/dmpLib/nc2010_transformation_system.cpp
	dmpLib/src/dmpLib/nc2010_transformation_system_parameters.cpp
	dmpLib/src/dmpLib/nc2010_transformation_system_state.cpp
	
	dmpLib/src/dmpLib/nc2010_dynamic_movement_primitive.cpp
	dmpLib/src/dmpLib/nc2010_dynamic_movement_primitive_parameters.cpp
	dmpLib/src/dmpLib/nc2010_dynamic_movement_primitive_state.cpp
)

add_library(lwr
	dmpLib/src/lwrLib/lwr.cpp
	dmpLib/src/lwrLib/lwr_parameters.cpp
	dmpLib/src/lwrLib/logger.cpp
)


# link against liblwr.a
target_link_libraries(dmp++ lwr)
target_link_libraries(dmp++ 
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_LIBRARIES}
)

add_executable(lwr_test dmpLib/test_lwr/test_locally_weigthed_regression.cpp)
target_link_libraries(lwr_test lwr)

add_executable(dmp_test
  dmpLib/test_dmp/test_dynamic_movement_primitive.cpp
  dmpLib/test_dmp/test_trajectory.cpp
  dmpLib/test_dmp/test_data.cpp
  dmpLib/test_dmp/nc2010_test.cpp
)
target_link_libraries(dmp_test dmp++)

#target_link_libraries(../dmpLib/test/dmp_test dmp++)
#rosbuild_link_boost(../dmpLib/test/dmp_test system filesystem)
#target_link_libraries(../dmpLib/test/dmp_test ${Boost_LIBRARIES})  

# common commands for building c++ executables and libraries

add_library(locally_weighted_regression
	src/locally_weighted_regression.cpp
)
target_link_libraries(locally_weighted_regression lwr)
target_link_libraries(locally_weighted_regression ${Boost_LIBRARIES})

add_library(${PROJECT_NAME}
  src/dynamic_movement_primitive.cpp
  src/transformation_system.cpp
  src/canonical_system.cpp

#  src/icra2009_dynamic_movement_primitive.cpp
#  src/icra2009_transformation_system.cpp
#  src/icra2009_canonical_system.cpp

  src/nc2010_dynamic_movement_primitive.cpp
  src/nc2010_transformation_system.cpp
  src/nc2010_canonical_system.cpp
)
target_link_libraries(${PROJECT_NAME} dmp++ lwr)
#rosbuild_link_boost(${PROJECT_NAME} system filesystem) 
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES}) 

 

catkin_add_gtest(dynamic_movement_primitive_test
  test/dynamic_movement_primitive_test_1.cpp
  dmpLib/test_dmp/test_trajectory.cpp
  dmpLib/test_dmp/test_data.cpp
  dmpLib/test_dmp/nc2010_test.cpp
)
target_link_libraries(dynamic_movement_primitive_test dmp++ gtest ${catkin_LIBRARIES} ${PROJECT_NAME} locally_weighted_regression ${Boost_LIBRARIES})
add_dependencies(dynamic_movement_primitive_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(tests dynamic_movement_primitive_test)
#target_link_libraries(dynamic_movement_primitive_test gtest)
#target_link_libraries(dynamic_movement_primitive_test ${PROJECT_NAME})
#add_rostest(launch/dynamic_movement_primitive_test.test)
#target_link_libraries(dynamic_movement_primitive_test ${Boost_LIBRARIES}) 
