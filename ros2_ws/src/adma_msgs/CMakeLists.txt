cmake_minimum_required(VERSION 3.5)
project(adma_msgs)

if(NOT DEFINED ENV{ROS_VERSION})
  message( SEND_ERROR "Environment variable ROS_VERSION not found. Skipping build" )
elseif($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
  )

  add_message_files(
    FILES
    AdmaData.msg
    AdmaDeltaData.msg
  )
  generate_messages(
  )


else()

  set(CMAKE_CXX_STANDARD 14)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AdmaData.msg"
    "msg/AdmaDeltaData.msg"
  )
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files} 
  )
  ament_package()
endif()
