cmake_minimum_required(VERSION 3.5)
project(radar_object_tracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-fpermissive) 

#Workaround for FastRTPS not found bug, see:
#https://answers.ros.org/question/334581/could-not-find-fastrtps-missing-fastrtps_include_dir-fastrtps_libraries/
set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(perception_kit_msgs REQUIRED)
find_package(radar_gen5_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


file(GLOB_RECURSE SRC_CPP_FILES
     src/*.c*
    #  src/radar_tracker_matlab_wrapper/*.c*
    #  src/radar_tracker_matlab_wrapper/radar_tracker/*.c*
)


add_executable(${PROJECT_NAME}_node
  # src/object_list_radar_node.cpp
  ${SRC_CPP_FILES}
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  perception_kit_msgs
  radar_gen5_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_node
  # ${PROJECT_NAME}_lib
  # ${RADAR_TRACKER_SRC}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)


include_directories(
  # include
  # src
  # src/lidar_object_tracking
  # src/core_layered_map
  # src/geographic_transform
  # src/layered_map_classifier
  src/radar_tracker_matlab_wrapper
  src/radar_tracker_matlab_wrapper/radar_tracker
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)
# link_directories(
#   ${PCL_INCLUDE_DIRS}
#   # ${Boost_SYSTEM_LIBRARY}
# )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
# install(TARGETS ${PROJECT_NAME}_lib
#   DESTINATION lib
# )
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
