cmake_minimum_required(VERSION 3.5.1)
project(radar_gen5_common)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL
#)

#set(${PROJECT_NAME}_CATKIN_PACKAGES_INTERNAL
#)

#find_package(catkin REQUIRED
#  COMPONENTS
#    ${${PROJECT_NAME}_CATKIN_PACKAGES_INTERNAL}
#    ${${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL}
#)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

## Uncomment this if vfc is requrired by this package
#find_package(vfc REQUIRED)
## a hack to make it possible to build project with catkin build as well as with cmake only build
## in catkin build: if vfc is not added to CATKIN_DEPENDS - dependent projects will fail to compile
#if(${vfc_FOUND_CATKIN_PROJECT})
#  list(APPEND ${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL vfc)
#endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
ament_export_dependencies(${${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL})

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}_lib
#  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL}
#)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_lib SHARED
  src/clock_definition.cpp
  src/decoder_type.cpp
  src/ticked_timestamp.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME}_lib)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    include
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
)


# ??? Needed ???
# To link with dependent packages that are ament packages, instead of using target_link_libraries(), 
# ament_target_dependencies() is a more concise and more thorough way of handling build flags. 
# It automatically handles both the include directories defined in _INCLUDE_DIRS and linking libraries 
# defined in _LIBRARIES.
# https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
#ament_target_dependencies(${PROJECT_NAME}_lib
#  rclcpp
#)
#target_link_libraries(${PROJECT_NAME}_lib
#  PUBLIC
#)

#############
## Install ##
#############

install(
  DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING REGEX ".*\\.(h.*|inl|tpp)"
)

install(
  TARGETS ${PROJECT_NAME}_lib
#  ARCHIVE DESTINATION lib/${PROJECT_NAME}
#  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# This was duplicate !?
#install(
#  DIRECTORY include/radar_gen5_common/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../radar_gen5_common
#)

#############
## Testing ##
#############


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
