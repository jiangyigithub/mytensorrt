cmake_minimum_required(VERSION 3.5.1)
project(radar_locations_decoder_gen5)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(vfc REQUIRED)

# This is a hack to make it possible to build the project with a vfc that is a catkin package as
# well as one that isn't. Catkin dependencies should be propagated via CATKIN_DEPENDS, whereas
# non-catkin dependencies should be propagated via DEPENDS.
#set(${PROJECT_NAME}_DEPENDS "")
#if(${vfc_FOUND_CATKIN_PROJECT})
#  list(APPEND ${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL vfc)
#else()
#  list(APPEND ${PROJECT_NAME}_DEPENDS vfc)
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
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}_lib
#  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_PACKAGES_EXTERNAL}
#)

ament_export_dependencies(vfc)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_include_directories(include)

###########
## Build ##
###########


file (GLOB_RECURSE HEADER_FILES_DECODER_GEN5 include/radar_locations_decoder_gen5/*.h*)

set (SRC_FILES_DECODER_GEN5
    ${HEADER_FILES_DECODER_GEN5}
    src/radar_locations_decoder.cpp
    src/radar_locations_decoder_impl_X391.cpp
    src/radar_locations_decoder_impl_X310_RC1_local.cpp
    src/radar_locations_decoder_impl_X230.cpp
    src/radar_locations_decoder_impl_X220.cpp
    src/radar_locations_decoder_impl_X173.cpp
    src/radar_locations_decoder_impl_X171_DV1_local.cpp
    src/radar_locations_decoder_impl_X170.cpp
    src/radar_locations_decoder_impl_X169.cpp
    src/radar_locations_decoder_impl_X168_local.cpp
    src/radar_locations_decoder_impl_B167_local.cpp
    src/radar_locations_decoder_impl_X167.cpp
    src/radar_locations_decoder_impl_Y163.cpp
    src/radar_locations_decoder_impl_D161.cpp
    src/radar_locations_decoder_impl_X123.cpp
    src/radar_locations_decoder_impl_X151.cpp
    src/radar_locations_decoder_impl_Y140.cpp
    src/radar_locations_decoder_impl_RC18041.cpp
)

add_library(${PROJECT_NAME}_lib SHARED
  ${SRC_FILES_DECODER_GEN5}
)

# We add an alias for our target because this makes it easier to include this package
# from the outside without using catkin
#add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME}_lib)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    include
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
)

if(TARGET vfc::vfc)
  # if vfc package was loaded with a modern vfcConfig.cmake (and not with catkin) there is a target vfc::vfc
  ament_target_dependencies(${PROJECT_NAME}_lib
    PUBLIC
      vfc::vfc
  )
else()
  # vfc package was found with catkin
  target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
      ${vfc_INCLUDE_DIRS}
  )

  ament_target_dependencies(${PROJECT_NAME}_lib
    PUBLIC
      ${vfc_LIBRARIES}
  )
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/radar_locations_decoder_gen5_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(
  DIRECTORY include/${PROJECT_NAME}/
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

install(
#  DIRECTORY include/radar_locations_decoder_gen5/
  DIRECTORY include/radar_locations_decoder_gen5/
#  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../radar_locations_decoder_gen5
  DESTINATION include/${PROJECT_NAME}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radar_locations_decoder_gen5.cpp)
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
