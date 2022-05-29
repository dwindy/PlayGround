cmake_minimum_required(VERSION 3.0)
project(PCLpointcloud)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )


add_executable(PCLpointcloud main.cpp)
target_link_libraries(PCLpointcloud ${PCL_LIBRARIES})