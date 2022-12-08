# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "asl_turtlebot: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iasl_turtlebot:/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(asl_turtlebot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg" NAME_WE)
add_custom_target(_asl_turtlebot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "asl_turtlebot" "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg" ""
)

get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg" NAME_WE)
add_custom_target(_asl_turtlebot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "asl_turtlebot" "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg" "asl_turtlebot/DetectedObject"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/asl_turtlebot
)
_generate_msg_cpp(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/asl_turtlebot
)

### Generating Services

### Generating Module File
_generate_module_cpp(asl_turtlebot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/asl_turtlebot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(asl_turtlebot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(asl_turtlebot_generate_messages asl_turtlebot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_cpp _asl_turtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_cpp _asl_turtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(asl_turtlebot_gencpp)
add_dependencies(asl_turtlebot_gencpp asl_turtlebot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS asl_turtlebot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/asl_turtlebot
)
_generate_msg_eus(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/asl_turtlebot
)

### Generating Services

### Generating Module File
_generate_module_eus(asl_turtlebot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/asl_turtlebot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(asl_turtlebot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(asl_turtlebot_generate_messages asl_turtlebot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_eus _asl_turtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_eus _asl_turtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(asl_turtlebot_geneus)
add_dependencies(asl_turtlebot_geneus asl_turtlebot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS asl_turtlebot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/asl_turtlebot
)
_generate_msg_lisp(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/asl_turtlebot
)

### Generating Services

### Generating Module File
_generate_module_lisp(asl_turtlebot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/asl_turtlebot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(asl_turtlebot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(asl_turtlebot_generate_messages asl_turtlebot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_lisp _asl_turtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_lisp _asl_turtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(asl_turtlebot_genlisp)
add_dependencies(asl_turtlebot_genlisp asl_turtlebot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS asl_turtlebot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/asl_turtlebot
)
_generate_msg_nodejs(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/asl_turtlebot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(asl_turtlebot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/asl_turtlebot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(asl_turtlebot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(asl_turtlebot_generate_messages asl_turtlebot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_nodejs _asl_turtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_nodejs _asl_turtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(asl_turtlebot_gennodejs)
add_dependencies(asl_turtlebot_gennodejs asl_turtlebot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS asl_turtlebot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/asl_turtlebot
)
_generate_msg_py(asl_turtlebot
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/asl_turtlebot
)

### Generating Services

### Generating Module File
_generate_module_py(asl_turtlebot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/asl_turtlebot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(asl_turtlebot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(asl_turtlebot_generate_messages asl_turtlebot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObject.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_py _asl_turtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/src/asl_turtlebot/msg/DetectedObjectList.msg" NAME_WE)
add_dependencies(asl_turtlebot_generate_messages_py _asl_turtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(asl_turtlebot_genpy)
add_dependencies(asl_turtlebot_genpy asl_turtlebot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS asl_turtlebot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/asl_turtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/asl_turtlebot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(asl_turtlebot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/asl_turtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/asl_turtlebot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(asl_turtlebot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/asl_turtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/asl_turtlebot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(asl_turtlebot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/asl_turtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/asl_turtlebot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(asl_turtlebot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/asl_turtlebot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/asl_turtlebot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/asl_turtlebot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(asl_turtlebot_generate_messages_py std_msgs_generate_messages_py)
endif()
