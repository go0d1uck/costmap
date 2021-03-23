cmake_minimum_required(VERSION 3.19)

set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")

project(costmap VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)
#set(GLOG_LIB glog)
#find_package(OpenCV)

include_directories(include)
include_directories(/usr/aarch64-linux-gnu/include/opencv4)
link_directories(${CMAKE_CURRENT_SOURCE_DIR})
#include_directories(${OpenCV_INCLUDE_DIR})
#add_executable(test ${PROJECT_SOURCE_DIR}/test.cc)
add_library(costmap SHARED src/layer.cc src/costmap.cc)
#target_link_libraries(test PUBLIC costmap)
#target_link_libraries(costmap PUBLIC ${GLOG_LIB})
#target_link_libraries(costmap PUBLIC ${OpenCV_LIBS})
