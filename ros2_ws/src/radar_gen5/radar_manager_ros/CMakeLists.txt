cmake_minimum_required(VERSION 3.5.1)
project(radar_manager_ros)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL
#  diagnostic_updater
#  message_generation
#  roscpp
#  roslib
#  radar_msgs
#  geometry_msgs
#)

#set(${PROJECT_NAME}_CATKIN_PACKAGES_INTERNAL
#)

#find_package(ament_cmake REQUIRED
#  COMPONENTS
#    ${${PROJECT_NAME}_CATKIN_PACKAGES_INTERNAL}
#    ${${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL}
#)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)


#Workaround for FastRTPS not found bug, see:
#https://answers.ros.org/question/334581/could-not-find-fastrtps-missing-fastrtps_include_dir-fastrtps_libraries/
set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)



###########
## Build ##
###########


set_source_files_properties(C_FILES PROPERTIES LANGUAGE CXX)

file (GLOB_RECURSE HEADER_FILES_MANAGER include/radar_manager_ros/*.h*)

add_library(${PROJECT_NAME}_lib 
  ${HEADER_FILES_MANAGER}
  src/radar_base.cpp
  src/communication/odt.cpp
  src/communication/tcp_socket.c
  src/communication/vdb.cpp
  src/communication/xcp.c
  src/radar_manager.cpp
  src/radar_manager_parameters.cpp
)

# We add an alias for our target because this makes it easier to include this package
# from the outside without using catkin
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME}_lib)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    include
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
#    ${diagnostic_updater_INCLUDE_DIRS}
#    ${radar_msgs_INCLUDE_DIRS}
#    ${roscpp_INCLUDE_DIRS}
#    ${roslib_INCLUDE_DIRS}
#  PRIVATE
#    ${geometry_msgs_INCLUDE_DIRS}
)


# To link with dependent packages that are ament packages, instead of using target_link_libraries(), 
# ament_target_dependencies() is a more concise and more thorough way of handling build flags. 
# It automatically handles both the include directories defined in _INCLUDE_DIRS and linking libraries 
# defined in _LIBRARIES.
# https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  ament_index_cpp
  diagnostic_updater
  radar_msgs
  geometry_msgs
)



# To link with packages that are not ament packages, such as system dependencies like Boost, or a library being 
# built in the same CMakeLists.txt, use target_link_libraries():
#target_link_libraries(${PROJECT_NAME}_lib
#  PUBLIC
#    ${diagnostic_updater_LIBRARIES}
#    ${radar_msgs_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${roslib_LIBRARIES}
#  PRIVATE
#    ${geometry_msgs_LIBRARIES}
#)


add_executable(${PROJECT_NAME}_exe src/main.cpp)

target_include_directories(${PROJECT_NAME}_exe
  PRIVATE
    include
)

target_link_libraries(${PROJECT_NAME}_exe
  PRIVATE
    ${PROJECT_NAME}_lib
)

set_target_properties(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})



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
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
#  FILES_MATCHING REGEX ".*\\.(yaml|launch)"
)

install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radar_reader.cpp)
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
