cmake_minimum_required(VERSION 3.19)

project(costmap VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)
set(GLOG_LIB glog)

include_directories(include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(test ${PROJECT_SOURCE_DIR}/test.cc)
add_library(costmap SHARED src/layer.cc src/costmap.cc)
target_link_libraries(test PUBLIC costmap)
target_link_libraries(costmap PUBLIC ${GLOG_LIB})


