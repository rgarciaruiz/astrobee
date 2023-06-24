cmake_minimum_required(VERSION 3.0)
project(execute_asap)

add_compile_options(-std=c++14)

find_package(catkin2 REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  trajectory_msgs
  sensor_msgs
  ff_msgs
)

catkin_package(
 LIBRARIES
 DEPENDS rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)

install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../astrobee/scripts/env_wrapper_asap.sh DESTINATION ${CMAKE_INSTALL_PREFIX} )