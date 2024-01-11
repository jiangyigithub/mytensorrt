cmake_minimum_required(VERSION 3.5.1)
project(radar_msgs)

if(NOT DEFINED ENV{ROS_VERSION})
  message( SEND_ERROR "Environment variable ROS_VERSION not found. Skipping build" )

# If ROS1
elseif($ENV{ROS_VERSION} EQUAL 1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    RadarFactoryDataArray.msg
    RadarFactoryData.msg
    RadarROB2.msg
    ROBData.msg
    Attribute.msg ##lnl, timer
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

####################################
## for usage without catkin build ##
####################################
if(NOT TARGET ${PROJECT_NAME}::${PROJECT_NAME})
  add_library(${PROJECT_NAME} INTERFACE)
  add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages)

  # Go out of any directories until we hit the main "devel" directory
  # TODO: clean this up as soon as we find a better way to handle ros
  # message generation
  set(PREFIX_DIR ${CATKIN_DEVEL_PREFIX})
  get_filename_component(DIR ${PREFIX_DIR} NAME)
  string(FIND ${DIR} "devel" DIR_POS)
  while(NOT DIR_POS EQUAL 0)
    get_filename_component(PREFIX_DIR ${PREFIX_DIR} DIRECTORY)
    get_filename_component(DIR ${PREFIX_DIR} NAME)
    string(FIND ${DIR} "devel" DIR_POS)
  endwhile()
  set(rosmsg_path "${PREFIX_DIR}/${gencpp_INSTALL_DIR}")
  target_include_directories(${PROJECT_NAME}
    INTERFACE "${rosmsg_path}"
  )
  unset(PREFIX_DIR)
  unset(rosmsg_path)
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radar_msgs.cpp)
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

# If ROS2
else()
  #Workaround for FastRTPS not found bug, see:
  #https://answers.ros.org/question/334581/could-not-find-fastrtps-missing-fastrtps_include_dir-fastrtps_libraries/
  set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
  set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)
  
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  set(msg_files
    "msg/RadarROB2.msg"
    "msg/ROBData.msg"
    "msg/RadarFactoryDataArray.msg"
    "msg/RadarFactoryData.msg"
    "msg/Attribute.msg"
  )
  
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs geometry_msgs
  )
  
  ament_package()
  
endif()
