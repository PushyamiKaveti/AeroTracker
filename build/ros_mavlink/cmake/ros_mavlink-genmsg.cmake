# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ros_mavlink: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iros_mavlink:/home/pushyamikaveti/AeroTracker/src/ros_mavlink/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ros_mavlink_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ros_mavlink
  "/home/pushyamikaveti/AeroTracker/src/ros_mavlink/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_mavlink
)

### Generating Services

### Generating Module File
_generate_module_cpp(ros_mavlink
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_mavlink
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ros_mavlink_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ros_mavlink_generate_messages ros_mavlink_generate_messages_cpp)

# target for backward compatibility
add_custom_target(ros_mavlink_gencpp)
add_dependencies(ros_mavlink_gencpp ros_mavlink_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_mavlink_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ros_mavlink
  "/home/pushyamikaveti/AeroTracker/src/ros_mavlink/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_mavlink
)

### Generating Services

### Generating Module File
_generate_module_lisp(ros_mavlink
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_mavlink
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ros_mavlink_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ros_mavlink_generate_messages ros_mavlink_generate_messages_lisp)

# target for backward compatibility
add_custom_target(ros_mavlink_genlisp)
add_dependencies(ros_mavlink_genlisp ros_mavlink_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_mavlink_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ros_mavlink
  "/home/pushyamikaveti/AeroTracker/src/ros_mavlink/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_mavlink
)

### Generating Services

### Generating Module File
_generate_module_py(ros_mavlink
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_mavlink
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ros_mavlink_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ros_mavlink_generate_messages ros_mavlink_generate_messages_py)

# target for backward compatibility
add_custom_target(ros_mavlink_genpy)
add_dependencies(ros_mavlink_genpy ros_mavlink_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_mavlink_generate_messages_py)


debug_message(2 "ros_mavlink: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_mavlink)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_mavlink
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ros_mavlink_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_mavlink)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_mavlink
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ros_mavlink_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_mavlink)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_mavlink\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_mavlink
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ros_mavlink_generate_messages_py std_msgs_generate_messages_py)
