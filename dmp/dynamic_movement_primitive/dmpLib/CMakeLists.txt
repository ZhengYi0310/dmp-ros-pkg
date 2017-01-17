cmake_minimum_required(VERSION 2.4.6)

#if(COMMAND cmake_policy)
#  cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)

#project (dmpLib)
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
#set the default path for built libraries to the "lib" directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/test")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${Boost_DEFINITIONS})

add_definitions(-Wall -Wextra)
add_definitions(-O0 -g3 -ggdb3 -fno-inline)
set(CMAKE_BUILD_TYPE Debug)

# TODO: fix this
include_directories( "../lwrLib/include/" )
link_directories( "../lwrLib/lib/" )





add_library(dmp++
	src/dmpLib/logger.cpp
	src/dmpLib/trajectory.cpp
	
	src/dmpLib/dynamic_movement_primitive_base.cpp
	src/dmpLib/dynamic_movement_primitive_state_base.cpp
	src/dmpLib/dynamic_movement_primitive_parameters_base.cpp
        
	src/dmpLib/canonical_system_base.cpp
	src/dmpLib/canonical_system_state_base.cpp
	src/dmpLib/canonical_system_parameters_base.cpp
        
	src/dmpLib/transformation_system_base.cpp
	src/dmpLib/transformation_system_state_base.cpp
	src/dmpLib/transformation_system_parameters_base.cpp
	
	src/dmpLib/nc2010_canonical_system_parameters.cpp
	src/dmpLib/nc2010_canonical_system_state.cpp
	src/dmpLib/nc2010_canonical_system.cpp
	
	src/dmpLib/nc2010_transformation_system.cpp
	src/dmpLib/nc2010_transformation_system_parameters.cpp
	src/dmpLib/nc2010_transformation_system_state.cpp
	
	src/dmpLib/nc2010_dynamic_movement_primitive.cpp
	src/dmpLib/nc2010_dynamic_movement_primitive_parameters.cpp
	src/dmpLib/nc2010_dynamic_movement_primitive_state.cpp

	
)

add_library(lwr
	src/lwrLib/lwr.cpp
	src/lwrLib/lwr_parameters.cpp
	src/lwrLib/logger.cpp
)
target_link_libraries(lwr ${Boost_LIBRARIES})




# link against liblwr.a
target_link_libraries(dmp++ lwr)

find_package(Boost COMPONENTS system filesystem REQUIRED)
target_link_libraries(dmp++ 
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_LIBRARIES}
)


add_executable(lwr_test test_lwr/test_locally_weigthed_regression.cpp)

target_link_libraries(lwr_test lwr)

add_executable(test_dmp
  test_dmp/test_dynamic_movement_primitive.cpp
  test_dmp/test_trajectory.cpp
  test_dmp/test_data.cpp
  test_dmp/nc2010_test.cpp
)
target_link_libraries(test_dmp dmp++)





