cmake_minimum_required(VERSION 3.19)

project(costmap VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

include_directories(include)
add_library(costmap SHARED src/layer.cc src/layered.cc)
add_executable(test ${PROJECT_SOURCE_DIR}/test.cc)
target_link_libraries(test PUBLIC costmap)
