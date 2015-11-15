# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kalman_zumy: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kalman_zumy_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv" NAME_WE)
add_custom_target(_kalman_zumy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kalman_zumy" "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv" "geometry_msgs/Transform:geometry_msgs/Quaternion:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv" NAME_WE)
add_custom_target(_kalman_zumy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kalman_zumy" "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv" "geometry_msgs/Vector3"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(kalman_zumy
  "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_zumy
)
_generate_srv_cpp(kalman_zumy
  "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_zumy
)

### Generating Module File
_generate_module_cpp(kalman_zumy
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_zumy
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kalman_zumy_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kalman_zumy_generate_messages kalman_zumy_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv" NAME_WE)
add_dependencies(kalman_zumy_generate_messages_cpp _kalman_zumy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv" NAME_WE)
add_dependencies(kalman_zumy_generate_messages_cpp _kalman_zumy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kalman_zumy_gencpp)
add_dependencies(kalman_zumy_gencpp kalman_zumy_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kalman_zumy_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(kalman_zumy
  "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_zumy
)
_generate_srv_lisp(kalman_zumy
  "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_zumy
)

### Generating Module File
_generate_module_lisp(kalman_zumy
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_zumy
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kalman_zumy_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kalman_zumy_generate_messages kalman_zumy_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv" NAME_WE)
add_dependencies(kalman_zumy_generate_messages_lisp _kalman_zumy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv" NAME_WE)
add_dependencies(kalman_zumy_generate_messages_lisp _kalman_zumy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kalman_zumy_genlisp)
add_dependencies(kalman_zumy_genlisp kalman_zumy_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kalman_zumy_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(kalman_zumy
  "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_zumy
)
_generate_srv_py(kalman_zumy
  "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_zumy
)

### Generating Module File
_generate_module_py(kalman_zumy
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_zumy
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kalman_zumy_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kalman_zumy_generate_messages kalman_zumy_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/NuSrv.srv" NAME_WE)
add_dependencies(kalman_zumy_generate_messages_py _kalman_zumy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa15/class/ee106a-ck/ros_workspaces/project/src/kalman_zumy/srv/ImuSrv.srv" NAME_WE)
add_dependencies(kalman_zumy_generate_messages_py _kalman_zumy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kalman_zumy_genpy)
add_dependencies(kalman_zumy_genpy kalman_zumy_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kalman_zumy_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_zumy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_zumy
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(kalman_zumy_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(kalman_zumy_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(kalman_zumy_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_zumy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_zumy
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(kalman_zumy_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(kalman_zumy_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(kalman_zumy_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_zumy)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_zumy\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_zumy
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(kalman_zumy_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(kalman_zumy_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(kalman_zumy_generate_messages_py sensor_msgs_generate_messages_py)
