cmake_minimum_required(VERSION 3.5.1)
project(radar_locations_decoder_gen5_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)

## message(${CMAKE_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake/Modules")
message(${CMAKE_MODULE_PATH})

find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(radar_gen5_common REQUIRED)
find_package(radar_gen5_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(radar_locations_decoder_gen5 REQUIRED)
find_package(vfc REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# These .msg are not used at all and subject to clean up. The used ones have been
# transferred to radar_gen5/radar_gen5_msgs
## Generate messages in the 'msg' folder



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
#  LIBRARIES ${PROJECT_NAME}_lib
#  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL}
#)

#ament_export_dependencies(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)


###########
## Build ##
###########


file (GLOB_RECURSE HEADER_FILES_DECODER_GEN5_ROS include/radar_locations_decoder_gen5_ros/*.h*)
set (SRC_FILES_DECODER_GEN5_ROS
    ${HEADER_FILES_DECODER_GEN5_ROS}
    src/radar_locations_decoder_gen5_node.cpp
    src/radar_locations_decoder_gen5_ros.cpp
    src/internal/device_information.cpp
    src/internal/utils.cpp
)

add_library(${PROJECT_NAME}_lib SHARED
  ${SRC_FILES_DECODER_GEN5_ROS}
)

# We add an alias for our target because this makes it easier to include this package
# from the outside without using catkin
#add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME}_lib)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    include
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
#    ${diagnostic_updater_INCLUDE_DIRS}
#    ${std_msgs_INCLUDE_DIRS}
#    ${radar_msgs_INCLUDE_DIRS}
#    ${radar_gen5_msgs_INCLUDE_DIRS}
#    ${std_msgs_INCLUDE_DIRS}
#    ${radar_locations_decoder_gen5_INCLUDE_DIRS}
#    ${radar_ros_time_converter_INCLUDE_DIRS}
#    ${roscpp_INCLUDE_DIRS}
#    ${YAML_CPP_INCLUDE_DIRS}
#  PRIVATE
#    ${geometry_msgs_INCLUDE_DIRS}
)

# To link with dependent packages that are ament packages, instead of using target_link_libraries(), 
# ament_target_dependencies() is a more concise and more thorough way of handling build flags. 
# It automatically handles both the include directories defined in _INCLUDE_DIRS and linking libraries 
# defined in _LIBRARIES.
# https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
ament_target_dependencies(${PROJECT_NAME}_lib
  diagnostic_updater
  std_msgs
  radar_msgs
  radar_gen5_msgs
  radar_gen5_common
  geometry_msgs
  rclcpp
  radar_locations_decoder_gen5
  vfc
#  yaml-cpp
)



target_link_libraries(${PROJECT_NAME}_lib
#  PUBLIC
    ${YAML_CPP_LIBRARIES}
)

add_executable(${PROJECT_NAME}_exe src/main.cpp)
#add_executable(${PROJECT_NAME}::node ALIAS ${PROJECT_NAME}_exe)

target_include_directories(${PROJECT_NAME}_exe
  PRIVATE
    include
)

target_link_libraries(${PROJECT_NAME}_exe
  PRIVATE
    ${PROJECT_NAME}_lib
#	${radar_locations_decoder_gen5_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#ament_target_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING REGEX ".*\\.(h.*|inl|tpp)"
)

install(
  TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING REGEX ".*\\.(yaml|launch)"
)

install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radar_locations_decoder_gen5_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

###########################
## PJ-MP Integration     ##
###########################
## As of now, the PJ-MP infrastructure requires every package to contain certain targets related
## to test execution using the catchkin tool. Therefore, we add an empty target here. Within the
## PJ-MP team, we will work on abandoning this requirement
if(NOT TARGET build_catchkin_tests)
  add_custom_target(build_catchkin_tests)
endif()
if(NOT TARGET run_catchkin_tests)
  add_custom_target(run_catchkin_tests)
endif()

ament_package()
