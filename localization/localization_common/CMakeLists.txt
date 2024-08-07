#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 3.0)
project(localization_common)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  astrobee
	camera
	config_reader
	msg_conversions
  ff_msgs
)

# Find GTSAM
find_package(GTSAM REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# Allow other packages to use python scripts from this package
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} ${GTSAM_LIBRARIES} gtsam
  CATKIN_DEPENDS
    astrobee
    camera
    config_reader
    msg_conversions
    ff_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/averager.cc
  src/combined_nav_state.cc
  src/combined_nav_state_covariances.cc
  src/distance_scaled_pose_covariance_interpolater.cc
  src/pose_covariance_interpolater.cc
  src/pose_interpolater.cc
  src/pose_with_covariance_interpolater.cc
  src/rate_timer.cc
  src/ros_timer.cc
  src/stats_logger.cc
  src/time.cc
  src/timer.cc
  src/test_utilities.cc
  src/utilities.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gtsam)

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_pose_interpolater
    test/test_pose_interpolater.test
    test/test_pose_interpolater.cc
  )
  target_link_libraries(test_pose_interpolater
    ${PROJECT_NAME}  
  )

  add_rostest_gtest(test_pose_with_covariance_interpolater
    test/test_pose_with_covariance_interpolater.test
    test/test_pose_with_covariance_interpolater.cc
  )
  target_link_libraries(test_pose_with_covariance_interpolater
    ${PROJECT_NAME}  
  )

  add_rostest_gtest(test_timestamped_set
    test/test_timestamped_set.test
    test/test_timestamped_set.cc
  )
  target_link_libraries(test_timestamped_set
    ${PROJECT_NAME}  
  )
  add_rostest_gtest(test_utilities_localization_common
    test/test_utilities.test
    test/test_utilities.cc
  )
  target_link_libraries(test_utilities_localization_common
    ${PROJECT_NAME}  
  )
endif()

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
