cmake_minimum_required(VERSION 3.19)
project(PCL_ICP)

find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

set(CMAKE_CXX_STANDARD 17)

add_executable(PCL_ICP main.cpp)
target_link_libraries(PCL_ICP ${OpenCV_LIBS} ${PCL_LIBRARIES})