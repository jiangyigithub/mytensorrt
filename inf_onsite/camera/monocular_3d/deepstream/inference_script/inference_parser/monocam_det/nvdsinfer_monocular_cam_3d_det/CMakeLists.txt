cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(PythonInterp REQUIRED)
project(demo)
# set(Torch_DIR /home/icv/Edward/deepstream/common/libtorch/share/cmake/Torch) 
set(Torch_DIR /home/icv/.local/lib/python3.8/site-packages/torch/share/cmake/Torch) 
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/include")
#main.cpp exe
add_library(demo SHARED ${PROJECT_SOURCE_DIR}/example_app.cpp)
#link libtorch .a .so
target_link_libraries(demo "${TORCH_LIBRARIES}")
target_include_directories(demo PRIVATE CUDA_INCLUDE_DIRS)
set_property(TARGET demo PROPERTY CXX_STANDARD 14)