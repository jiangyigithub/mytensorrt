cmake_minimum_required(VERSION 3.5.1)
project(radar_locations_to_pointcloud_converter_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(radar_gen5_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)



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

#set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
#set(PCL_LIBRARIES /usr/include/pcl-1.8)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
file (GLOB_RECURSE HEADER_FILES_DECODER_GEN5_ROS include/radar_locations_to_pointcloud_converter_ros/*.h*)
set (SRC_FILES_DECODER_GEN5_ROS
    ${HEADER_FILES_DECODER_GEN5_ROS}
    src/radar_locations_to_pointcloud_converter_ros.cpp
)

add_library(${PROJECT_NAME}_lib SHARED
  ${SRC_FILES_DECODER_GEN5_ROS}
)

# We add an alias for our target because this makes it easier to include this package
# from the outside without using catkin
#add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME}_lib)

include_directories(/usr/include/pcl-1.8)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    include
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)

#message(WARNING "this is pcl:" ${PCL_INCLUDE_DIRS} "00:" ${PCL_LIBRARIES} )

# To link with dependent packages that are ament packages, instead of using target_link_libraries(), 
# ament_target_dependencies() is a more concise and more thorough way of handling build flags. 
# It automatically handles both the include directories defined in _INCLUDE_DIRS and linking libraries 
# defined in _LIBRARIES.
# https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  radar_gen5_msgs
  tf2_ros
  tf2_geometry_msgs
)



target_link_libraries(${PROJECT_NAME}_lib
    ${PCL_LIBRARIES}
)

#add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_exe src/main.cpp)

target_include_directories(${PROJECT_NAME}_exe
  PRIVATE
    include
)

target_link_libraries(${PROJECT_NAME}_exe
  PRIVATE
    ${PROJECT_NAME}_lib
    ${geometry_msgs_LIBRARIES}
    
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
