cmake_minimum_required(VERSION 3.5)
project(track_to_track_fusion)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

## Compile with warnings as errors
if( "${CMAKE_COMPILER_IS_GNUCXX}" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if(DEFINED ENV{ROS_VERSION})
#     if("$ENV{ROS_VERSION}" STREQUAL "1")
#         include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeListsRos1.txt)
#     elseif("$ENV{ROS_VERSION}" STREQUAL "2")
#         message (NOTICE "ROS2 is sourced. Configuring project for ROS2 ...")
#         include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeListsRos2.txt)
#     else()
#         message(SEND_ERROR "Unknown ROS version! (Should be '1' or '2'). Won't continue with configuration!")
#     endif()
# else()
#     message(SEND_ERROR "Environment variable 'ROS_VERSION' is undefined! Won't continue with configuration!")
# endif()

add_compile_options(-fpermissive)

add_definitions(-DROS2)

#Workaround for FastRTPS not found bug, see:
#https://answers.ros.org/question/334581/could-not-find-fastrtps-missing-fastrtps_include_dir-fastrtps_libraries/
set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(perception_kit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIR}
  ${perception_kit_msgs_INCLUDE_DIRS})

message(STATUS "Variables: ${device_engine_INCLUDE_DIRS}")

add_library(${PROJECT_NAME}_objects_processors
  src/objects_processors/existence_threshold_filter.cpp
  src/objects_processors/map_filter.cpp
)

add_library(${PROJECT_NAME}_objects_prediction
  src/objects_prediction/constant_velocity.cpp
)

add_library(${PROJECT_NAME}_feature_fusion
  src/${PROJECT_NAME}/feature/classification.cpp
  src/${PROJECT_NAME}/feature/dimension.cpp
  src/${PROJECT_NAME}/feature/dynamics.cpp
  src/${PROJECT_NAME}/feature/existence_probability.cpp
  src/${PROJECT_NAME}/feature/feature_fusion_interfaces.cpp
  src/${PROJECT_NAME}/feature/orientation.cpp
  src/${PROJECT_NAME}/feature/position.cpp
)

add_library(${PROJECT_NAME}_cost_functions
  src/${PROJECT_NAME}/cost/cost_functions.cpp
)

add_library(${PROJECT_NAME}_weights
  src/${PROJECT_NAME}/weights.cpp
  src/${PROJECT_NAME}/object_with_trace.cpp
  src/${PROJECT_NAME}/trace_storage.cpp
  src/${PROJECT_NAME}/cost_cell.cpp
)

##################################
# target "track_to_track_fusion"
##################################
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/fusion_interface.cpp
  src/${PROJECT_NAME}/fusion_postprocessor_interface.cpp
  src/${PROJECT_NAME}/fusion_preprocessor_interface.cpp
  src/${PROJECT_NAME}/object_fusion_executer.cpp
  src/${PROJECT_NAME}/weights.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  perception_kit_msgs
  #radar_gen5_msgs 
  visualization_msgs
  )

##################################
# target "track_to_track_fusion_node"
##################################
add_executable(${PROJECT_NAME}_node
  src/ros2/${PROJECT_NAME}_node_main.cpp
  src/ros2/fusion_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  # ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_cost_functions
  ${PROJECT_NAME}_objects_processors
  ${PROJECT_NAME}_objects_prediction
  ${PROJECT_NAME}_feature_fusion
  ${PROJECT_NAME}_weights
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()