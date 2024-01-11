cmake_minimum_required(VERSION 3.5)
project(perception_kit_object_transform)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perception_kit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/object_transform.cpp
  src/object_transform_component.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  perception_kit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry2
)
rclcpp_components_register_nodes(${PROJECT_NAME} "perception_kit::object_transform::ObjectTransform")

#add_library(${PROJECT_NAME}_python
#  src/python/python_binding.cpp
#)
#ament_target_dependencies(${PROJECT_NAME}_python
#  rclcpp
#  perception_kit_msgs
#  geometry_msgs
#  #${PROJECT_NAME}
#  ${PYTHON_LIBRARY}
#  ${Boost_LIBRARIES}
#)
#target_link_libraries(${PROJECT_NAME}_python
#    ${PROJECT_NAME}
#    ${PYTHON_LIBRARY}
#    ${rclcpp_LIBRARIES}
#    ${Boost_LIBRARIES}
#)


## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

#install(TARGETS ${PROJECT_NAME}_python 
#  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#)

# Workaround so python bindings can also be used from within devel-space
#set(PYTHON_LIB_DIR ${CATKIN_DEVEL_PREFIX}/lib/python2.7/dist-packages/${PROJECT_NAME})
#add_custom_command(TARGET ${PROJECT_NAME}_python
#    POST_BUILD
#    COMMAND mkdir -p ${PYTHON_LIB_DIR} && cp -v $<TARGET_FILE:${PROJECT_NAME}_python> ${PYTHON_LIB_DIR}
#    WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}
#    COMMENT "Copying library files to python directory"
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#   ${PROJECT_NAME}_plugin.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


if(CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/unit_tests/test_${PROJECT_NAME}.cpp
  )
  target_include_directories(${PROJECT_NAME}-test
    PRIVATE
      ${PROJECT_NAME}
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )

  find_package(rostest REQUIRED)
  add_rostest(test/integration_tests/ObjectTransformTest.test ARGS test_name:=Test01)

  catkin_add_nosetests(test/nosetests/test_perception_kit_object_transform.py
                       DEPENDENCIES ${catkin_EXPORTED_TARGETS})
endif()

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()