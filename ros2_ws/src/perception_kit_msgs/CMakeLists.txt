cmake_minimum_required(VERSION 3.5)
project(perception_kit_msgs)

if(NOT DEFINED ENV{ROS_VERSION})
  message( SEND_ERROR "Environment variable ROS_VERSION not found. Skipping build" )
elseif($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    roscpp
    std_msgs
    nav_msgs
  )

  add_message_files(
    FILES
    Attribute.msg
    Classification.msg
    DynamicOccupancyGrid.msg
    Motion.msg
    Object.msg
    Objects.msg
  )
  generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
  )
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES perception_kit_msgs
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
  #  DEPENDS system_lib
  )

else()

  set(CMAKE_CXX_STANDARD 14)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  set(msg_files
    "msg/Attribute.msg"
    "msg/Classification.msg"
    "msg/DynamicOccupancyGrid.msg"
    "msg/Motion.msg"
    "msg/Object.msg"
    "msg/Objects.msg"
  )
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs geometry_msgs nav_msgs
  )
  ament_package()
endif()
