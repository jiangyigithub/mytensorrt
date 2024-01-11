cmake_minimum_required(VERSION 3.5)
project(adma_publisher)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(perception_kit_msgs REQUIRED)
find_package(adma_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(GeographicLib REQUIRED)

# includes
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# target_link_libraries(${PROJECT_NAME}
#   ${GeographicLib_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   ${PCL_LIBRARIES})


# library
file(GLOB_RECURSE LIB_SRC
     src/*.c*
)
add_library(${PROJECT_NAME} SHARED
  ${LIB_SRC}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  perception_kit_msgs 
  adma_msgs
  Eigen3
  )


# exe
add_executable(${PROJECT_NAME}_node
  ${LIB_SRC}
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  perception_kit_msgs
  adma_msgs
  Eigen3
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  # ${GeographicLib_LIBRARIES}
)



# install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
ament_package()
