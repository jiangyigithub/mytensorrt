cmake_minimum_required(VERSION 3.5.1)
project(vfc)


find_package(ament_cmake REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vfc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

ament_export_include_directories(include)


###########
## Build ##
###########

# remove ROSCONSOLE output at compile time for MinSizeRel build-type
#if(CMAKE_BUILD_TYPE MATCHES MINSIZEREL)
#  add_definitions(-DROSCONSOLE_MIN_SEVERITY=5)
#endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
)

#Set the include paths for qt-creator
file (GLOB_RECURSE HEADERS
*.h
*.hpp
)

#c and cpp files must be listed explicity! Do not use GLOB_RECURSE for those!
set(SOURCE_FILES_LIB
  src/dummy.cpp   #one c-file needed, so that a library can be built -> this allows files to be displayed in QT-Creator.
  ${HEADERS}
)

## Declare a cpp library (all c/cpp files should be in the library except the one containing the main call). This improves useability for qt-creator.
add_library(${PROJECT_NAME}_lib
 ${SOURCE_FILES_LIB}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vfc_node vfc_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(vfc_node
#   ${catkin_LIBRARIES}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

file(GLOB_RECURSE VFC_INCLUDE_CPPUNIT test/include_cppunit/*.hpp)
file(GLOB_RECURSE VFC_SOURCE_CPPUNIT  test/source_cppunit/*.cpp)

#catkin_add_gtest(${PROJECT_NAME}-test ${VFC_SOURCE_CPPUNIT})
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_package()
