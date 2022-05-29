cmake_minimum_required(VERSION 3.17)
project(LiDARVO)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "debug")

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_executable(LiDARVO main.cpp)
target_link_libraries(LiDARVO ${Pangolin_LIBRARIES} ${PCL_LIBRARIES})