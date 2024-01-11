cmake_minimum_required(VERSION 3.5)
project(perception_kit_visu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-fpermissive)

#Workaround for FastRTPS not found bug, see:
#https://answers.ros.org/question/334581/could-not-find-fastrtps-missing-fastrtps_include_dir-fastrtps_libraries/
set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(perception_kit_msgs REQUIRED)
#find_package(rqt_gui REQUIRED)
#find_package(rqt_gui_cpp REQUIRED)
find_package(rviz_common REQUIRED)
#find_package(rviz2 REQUIRED)
#or rviz or rviz2? -> with rviz2 error in rviz: OgreSceneNode.h: No such file or directory
find_package(visualization_msgs REQUIRED)
#find_package(OGRE)
find_package(tf2)
#Eigen3?
#libOpenGl, libGLX?

find_package(Qt5 ${rviz2_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
find_package(rviz_default_plugins REQUIRED)

include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_INCLUDE_DIRS}
  #Eigen?
)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp( MOC_FILES
  include/${PROJECT_NAME}/dynamic_objects_display.h
)

add_library(${PROJECT_NAME} SHARED
  src/dynamic_objects_display.cpp
  ${QT_UI_FILES}
  ${MOC_FILES}
)
#option(BUILD_SHARED_LIBS "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)" ON)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp perception_kit_msgs visualization_msgs rviz_common tf2) #rviz ##rqt_gui rqt_gui_cpp rviz2
#target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} rviz_common::rviz_common)
target_link_libraries(${PROJECT_NAME} rviz_default_plugins::rviz_default_plugins)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core)

add_executable(${PROJECT_NAME}_object_simulation
  src/object_simulation.cpp
)
ament_target_dependencies(${PROJECT_NAME}_object_simulation
  rclcpp perception_kit_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_object_simulation ${PROJECT_NAME})


#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#
#  catkin_add_gtest(${PROJECT_NAME}-tests
#    test/tests_${PROJECT_NAME}.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-tests
#    ${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets Qt5::Core
#  )
#  target_include_directories(${PROJECT_NAME}-tests
#  PRIVATE
#    ${PROJECT_NAME}
#  )
#endif()


install(TARGETS
  ${PROJECT_NAME}_object_simulation ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  #ARCHIVE DESTINATION lib/${PROJECT_NAME}
  #LIBRARY DESTINATION lib/${PROJECT_NAME}
  #RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.h"
#  #PATTERN ".svn" EXCLUDE
#)

#install(FILES
#  plugin_description.xml
#  DESTINATION share/${PROJECT_NAME}
#)
# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(DIRECTORY
  icons
  DESTINATION share/${PROJECT_NAME}
)
# Export the icons for the plugins
#ament_auto_package(INSTALL_TO_SHARE icons)





ament_package()
