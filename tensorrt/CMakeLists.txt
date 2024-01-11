cmake_minimum_required(VERSION 3.10)
project(TensorRTExample)

# Find and include CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

include_directories("/usr/include/aarch64-linux-gnu")

# Add your source files
set(SOURCES main.cpp)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

set(TensorRT_LIBRARIES "/usr/lib/aarch64-linux-gnu")
file(GLOB LIBS "${TensorRT_LIBRARIES}/*.so")
# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${CUDA_LIBRARIES} ${LIBS})

# Set C++ version and compiler flags
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

# Set compiler flags (adjust as needed)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# If using C++11 or later, you can replace the above two lines with:
# target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
