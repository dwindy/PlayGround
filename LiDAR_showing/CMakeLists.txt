cmake_minimum_required(VERSION 3.0)
project(LiDAR_showing)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_executable(LiDAR_showing main.cpp)
target_link_libraries(LiDAR_showing ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES} )

