cmake_minimum_required(VERSION 3.19)

project(costmap VERSION 3.0)
configure_file(costmap_config.in costmap_config.h)

if(ANDROID)
  set(OpenCV_DIR
      /home/go0d1uck/data/antraume/costmap/opencv-3.4.3/build/install/sdk/native/jni/abi-arm64-v8a/
  )
  set(glog_DIR
    /home/go0d1uck/Downloads/glog-master/build_android/install/lib/cmake/glog
  )
  set(CMAKE_CXX_FLAGS_RELEASE_INIT "-o3")
else()
  set(OpenCV_DIR
    /home/go0d1uck/Downloads/opencv-3.4.0/build_x86_linux/install/share/OpenCV
  )
  set(glog_DIR
    /home/go0d1uck/Downloads/glog-master/build_android/install/lib/cmake/glog
  )
endif(ANDROID)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

find_package(OpenCV REQUIRED)
find_package (glog REQUIRED)
include_directories(include)
include_directories(${glog_DIR}/../../../include)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(test ${PROJECT_SOURCE_DIR}/test.cc)
add_library(costmap SHARED src/layer.cc src/costmap.cc)
add_library(ut SHARED src/layer.cc src/ut.cc)
if(USE_STATIC)
add_library(laser STATIC src/layer.cc src/laser.cc)
else()
add_library(laser SHARED src/layer.cc src/laser.cc)
endif(USE_STATIC)

if(ANDROID)
  target_link_libraries(costmap /home/go0d1uck/data/antraume/costmap/lib/libglog.so)
  target_link_libraries(ut /home/go0d1uck/data/antraume/costmap/lib/libglog.so)
  target_link_libraries(laser /home/go0d1uck/data/antraume/costmap/lib/libglog.so)
else()
  target_link_libraries(costmap /home/go0d1uck/data/antraume/costmap/lib/x86_64/glog/build/libglog.so)
endif(ANDROID)
target_link_libraries(test PUBLIC costmap ut laser)
target_link_libraries(costmap ${OpenCV_LIBS})
target_link_libraries(ut ${OpenCV_LIBS})
target_link_libraries(laser ${OpenCV_LIBS})


install(TARGETS costmap DESTINATION lib)
install(TARGETS ut DESTINATION lib)
install(TARGETS laser DESTINATION lib)
install(FILES ${PROJECT_SOURCE_DIR}/include/costmap.h DESTINATION include)
install(FILES ${PROJECT_SOURCE_DIR}/include/ut.h DESTINATION include)
install(FILES ${PROJECT_SOURCE_DIR}/include/layer.h DESTINATION include)
install(FILES ${PROJECT_SOURCE_DIR}/include/laser.h DESTINATION include)
install(FILES ${PROJECT_SOURCE_DIR}/config/costmap_config.yaml DESTINATION config)
install(FILES ${PROJECT_SOURCE_DIR}/config/ut_config.yaml DESTINATION config)
install(FILES ${PROJECT_SOURCE_DIR}/config/laser_config.yaml DESTINATION config)
