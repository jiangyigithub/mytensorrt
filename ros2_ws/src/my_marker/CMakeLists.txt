cmake_minimum_required(VERSION 3.5)
project(my_marker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-fpermissive) # 错误级别降为警告级别

#Workaround for FastRTPS not found bug, see:
#https://answers.ros.org/question/334581/could-not-find-fastrtps-missing-fastrtps_include_dir-fastrtps_libraries/
set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# add_library(${PROJECT_NAME}_lib SHARED
#   src/show_map_node.cpp
#   src/show_map.cpp
# )
# ament_target_dependencies(${PROJECT_NAME}_lib 
# rclcpp
# visualization_msgs
# )

add_executable(${PROJECT_NAME}_node  
src/show_map_node.cpp
src/show_map.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node 
rclcpp
visualization_msgs
)
# target_link_libraries(${PROJECT_NAME}_node
# ${PROJECT_NAME}_lib
# )


# install(TARGETS ${PROJECT_NAME}_lib
#   DESTINATION lib
# )
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
