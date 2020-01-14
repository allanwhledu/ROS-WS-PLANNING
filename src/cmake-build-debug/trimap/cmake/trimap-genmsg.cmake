# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "trimap: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itrimap:/home/bit/ROS-WS-PLANNING/src/trimap/msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(trimap_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg" NAME_WE)
add_custom_target(_trimap_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trimap" "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(trimap
  "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trimap
)

### Generating Services

### Generating Module File
_generate_module_cpp(trimap
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trimap
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(trimap_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(trimap_generate_messages trimap_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg" NAME_WE)
add_dependencies(trimap_generate_messages_cpp _trimap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trimap_gencpp)
add_dependencies(trimap_gencpp trimap_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trimap_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(trimap
  "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trimap
)

### Generating Services

### Generating Module File
_generate_module_eus(trimap
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trimap
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(trimap_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(trimap_generate_messages trimap_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg" NAME_WE)
add_dependencies(trimap_generate_messages_eus _trimap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trimap_geneus)
add_dependencies(trimap_geneus trimap_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trimap_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(trimap
  "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trimap
)

### Generating Services

### Generating Module File
_generate_module_lisp(trimap
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trimap
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(trimap_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(trimap_generate_messages trimap_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg" NAME_WE)
add_dependencies(trimap_generate_messages_lisp _trimap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trimap_genlisp)
add_dependencies(trimap_genlisp trimap_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trimap_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(trimap
  "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trimap
)

### Generating Services

### Generating Module File
_generate_module_nodejs(trimap
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trimap
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(trimap_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(trimap_generate_messages trimap_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg" NAME_WE)
add_dependencies(trimap_generate_messages_nodejs _trimap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trimap_gennodejs)
add_dependencies(trimap_gennodejs trimap_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trimap_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(trimap
  "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trimap
)

### Generating Services

### Generating Module File
_generate_module_py(trimap
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trimap
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(trimap_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(trimap_generate_messages trimap_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bit/ROS-WS-PLANNING/src/trimap/msg/Trimap.msg" NAME_WE)
add_dependencies(trimap_generate_messages_py _trimap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trimap_genpy)
add_dependencies(trimap_genpy trimap_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trimap_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trimap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trimap
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(trimap_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(trimap_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trimap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trimap
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(trimap_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(trimap_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trimap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trimap
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(trimap_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(trimap_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trimap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trimap
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(trimap_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(trimap_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trimap)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trimap\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trimap
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(trimap_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(trimap_generate_messages_py std_msgs_generate_messages_py)
endif()
