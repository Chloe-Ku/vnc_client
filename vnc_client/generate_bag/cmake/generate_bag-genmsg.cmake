# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "generate_bag: 2 messages, 0 services")

set(MSG_I_FLAGS "-Igenerate_bag:/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(generate_bag_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg" NAME_WE)
add_custom_target(_generate_bag_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "generate_bag" "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg" "geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/TwistWithCovariance:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:nav_msgs/Odometry"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg" NAME_WE)
add_custom_target(_generate_bag_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "generate_bag" "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/generate_bag
)
_generate_msg_cpp(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/generate_bag
)

### Generating Services

### Generating Module File
_generate_module_cpp(generate_bag
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/generate_bag
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(generate_bag_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(generate_bag_generate_messages generate_bag_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_cpp _generate_bag_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_cpp _generate_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(generate_bag_gencpp)
add_dependencies(generate_bag_gencpp generate_bag_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS generate_bag_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/generate_bag
)
_generate_msg_eus(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/generate_bag
)

### Generating Services

### Generating Module File
_generate_module_eus(generate_bag
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/generate_bag
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(generate_bag_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(generate_bag_generate_messages generate_bag_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_eus _generate_bag_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_eus _generate_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(generate_bag_geneus)
add_dependencies(generate_bag_geneus generate_bag_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS generate_bag_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/generate_bag
)
_generate_msg_lisp(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/generate_bag
)

### Generating Services

### Generating Module File
_generate_module_lisp(generate_bag
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/generate_bag
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(generate_bag_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(generate_bag_generate_messages generate_bag_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_lisp _generate_bag_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_lisp _generate_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(generate_bag_genlisp)
add_dependencies(generate_bag_genlisp generate_bag_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS generate_bag_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/generate_bag
)
_generate_msg_nodejs(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/generate_bag
)

### Generating Services

### Generating Module File
_generate_module_nodejs(generate_bag
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/generate_bag
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(generate_bag_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(generate_bag_generate_messages generate_bag_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_nodejs _generate_bag_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_nodejs _generate_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(generate_bag_gennodejs)
add_dependencies(generate_bag_gennodejs generate_bag_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS generate_bag_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/generate_bag
)
_generate_msg_py(generate_bag
  "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/generate_bag
)

### Generating Services

### Generating Module File
_generate_module_py(generate_bag
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/generate_bag
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(generate_bag_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(generate_bag_generate_messages generate_bag_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/Odom.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_py _generate_bag_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/2020SpringTeam22/src/generate_bag/msg/DriveInfo.msg" NAME_WE)
add_dependencies(generate_bag_generate_messages_py _generate_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(generate_bag_genpy)
add_dependencies(generate_bag_genpy generate_bag_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS generate_bag_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/generate_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/generate_bag
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(generate_bag_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(generate_bag_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/generate_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/generate_bag
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(generate_bag_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(generate_bag_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/generate_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/generate_bag
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(generate_bag_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(generate_bag_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/generate_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/generate_bag
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(generate_bag_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(generate_bag_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/generate_bag)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/generate_bag\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/generate_bag
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(generate_bag_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(generate_bag_generate_messages_py nav_msgs_generate_messages_py)
endif()
