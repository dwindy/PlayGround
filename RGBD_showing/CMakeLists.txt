cmake_minimum_required(VERSION 3.17)
project(RGBD_showing)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_executable(RGBD_showing main.cpp)
target_link_libraries(RGBD_showing ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(RGBD_showing check_fix_depth.cpp)
#target_link_libraries(RGBD_showing ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES})