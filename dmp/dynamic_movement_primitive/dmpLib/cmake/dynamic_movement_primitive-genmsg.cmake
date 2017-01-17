# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dynamic_movement_primitive: 24 messages, 0 services")

set(MSG_I_FLAGS "-Idynamic_movement_primitive:/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dynamic_movement_primitive_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg" "dynamic_movement_primitive/TransformationSystemParametersMsg:dynamic_movement_primitive/NC2010TransformationSystemStateMsg:dynamic_movement_primitive/StateMsg:dynamic_movement_primitive/Model:dynamic_movement_primitive/NC2010TransformationSystemParametersMsg:dynamic_movement_primitive/TransformationSystemMsg:dynamic_movement_primitive/TransformationSystemStateMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg" "dynamic_movement_primitive/CanonicalSystemParametersMsg:dynamic_movement_primitive/StateMsg:dynamic_movement_primitive/CanonicalSystemStateMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg" "dynamic_movement_primitive/TimeMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg" "dynamic_movement_primitive/StateMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg" "dynamic_movement_primitive/DynamicMovementPrimitiveParametersMsg:dynamic_movement_primitive/NC2010TransformationSystemMsg:dynamic_movement_primitive/NC2010TransformationSystemParametersMsg:dynamic_movement_primitive/CanonicalSystemStateMsg:dynamic_movement_primitive/NC2010TransformationSystemStateMsg:dynamic_movement_primitive/DynamicMovementPrimitiveStateMsg:dynamic_movement_primitive/NC2010CanonicalSystemStateMsg:dynamic_movement_primitive/CanonicalSystemMsg:dynamic_movement_primitive/NC2010DynamicMovementPrimitiveParametersMsg:dynamic_movement_primitive/TransformationSystemParametersMsg:dynamic_movement_primitive/StateMsg:dynamic_movement_primitive/NC2010CanonicalSystemParametersMsg:dynamic_movement_primitive/NC2010DynamicMovementPrimitiveStateMsg:dynamic_movement_primitive/Model:dynamic_movement_primitive/TimeMsg:dynamic_movement_primitive/NC2010CanonicalSystemMsg:dynamic_movement_primitive/NC2010DynamicMovementPrimitiveMsg:dynamic_movement_primitive/TransformationSystemMsg:dynamic_movement_primitive/DynamicMovementPrimitiveMsg:dynamic_movement_primitive/TransformationSystemStateMsg:dynamic_movement_primitive/CanonicalSystemParametersMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg" "dynamic_movement_primitive/DynamicMovementPrimitiveParametersMsg:dynamic_movement_primitive/NC2010TransformationSystemMsg:dynamic_movement_primitive/NC2010TransformationSystemParametersMsg:dynamic_movement_primitive/CanonicalSystemStateMsg:dynamic_movement_primitive/NC2010TransformationSystemStateMsg:dynamic_movement_primitive/DynamicMovementPrimitiveStateMsg:dynamic_movement_primitive/NC2010CanonicalSystemStateMsg:dynamic_movement_primitive/CanonicalSystemMsg:dynamic_movement_primitive/NC2010DynamicMovementPrimitiveParametersMsg:dynamic_movement_primitive/TransformationSystemParametersMsg:dynamic_movement_primitive/StateMsg:dynamic_movement_primitive/NC2010CanonicalSystemParametersMsg:dynamic_movement_primitive/Model:dynamic_movement_primitive/TimeMsg:dynamic_movement_primitive/NC2010CanonicalSystemMsg:dynamic_movement_primitive/NC2010DynamicMovementPrimitiveStateMsg:dynamic_movement_primitive/TransformationSystemMsg:dynamic_movement_primitive/DynamicMovementPrimitiveMsg:dynamic_movement_primitive/TransformationSystemStateMsg:dynamic_movement_primitive/CanonicalSystemParametersMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg" "dynamic_movement_primitive/TimeMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg" "dynamic_movement_primitive/DynamicMovementPrimitiveStateMsg:dynamic_movement_primitive/DynamicMovementPrimitiveParametersMsg:dynamic_movement_primitive/TimeMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg" "dynamic_movement_primitive/StateMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg" "dynamic_movement_primitive/NC2010CanonicalSystemParametersMsg:dynamic_movement_primitive/CanonicalSystemStateMsg:dynamic_movement_primitive/CanonicalSystemMsg:dynamic_movement_primitive/StateMsg:dynamic_movement_primitive/CanonicalSystemParametersMsg:dynamic_movement_primitive/NC2010CanonicalSystemStateMsg"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg" "dynamic_movement_primitive/Model"
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg" ""
)

get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg" NAME_WE)
add_custom_target(_dynamic_movement_primitive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_movement_primitive" "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg" "dynamic_movement_primitive/Model:dynamic_movement_primitive/TransformationSystemStateMsg:dynamic_movement_primitive/TransformationSystemParametersMsg:dynamic_movement_primitive/StateMsg"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_cpp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
)

### Generating Services

### Generating Module File
_generate_module_cpp(dynamic_movement_primitive
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dynamic_movement_primitive_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dynamic_movement_primitive_generate_messages dynamic_movement_primitive_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_movement_primitive_gencpp)
add_dependencies(dynamic_movement_primitive_gencpp dynamic_movement_primitive_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_movement_primitive_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_lisp(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
)

### Generating Services

### Generating Module File
_generate_module_lisp(dynamic_movement_primitive
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dynamic_movement_primitive_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dynamic_movement_primitive_generate_messages dynamic_movement_primitive_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp _dynamic_movement_primitive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_movement_primitive_genlisp)
add_dependencies(dynamic_movement_primitive_genlisp dynamic_movement_primitive_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_movement_primitive_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)
_generate_msg_py(dynamic_movement_primitive
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
)

### Generating Services

### Generating Module File
_generate_module_py(dynamic_movement_primitive
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dynamic_movement_primitive_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dynamic_movement_primitive_generate_messages dynamic_movement_primitive_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DMPUtilitiesMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/Model.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/StateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/DynamicMovementPrimitiveMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/ControllerStatusMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/CanonicalSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010DynamicMovementPrimitiveStateMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010TransformationSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TimeMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/NC2010CanonicalSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemParametersMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TypeMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/msg/TransformationSystemMsg.msg" NAME_WE)
add_dependencies(dynamic_movement_primitive_generate_messages_py _dynamic_movement_primitive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_movement_primitive_genpy)
add_dependencies(dynamic_movement_primitive_genpy dynamic_movement_primitive_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_movement_primitive_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_movement_primitive
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(dynamic_movement_primitive_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(dynamic_movement_primitive_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_movement_primitive
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(dynamic_movement_primitive_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(dynamic_movement_primitive_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_movement_primitive
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(dynamic_movement_primitive_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(dynamic_movement_primitive_generate_messages_py sensor_msgs_generate_messages_py)
