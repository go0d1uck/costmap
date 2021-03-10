cmake_minimum_required(VERSION 3.19)

project(costmap)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)


add_library(costmap SHARED ${PROJECT_SOURCE_DIR}/src/layer.cpp)
