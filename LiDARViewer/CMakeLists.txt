cmake_minimum_required(VERSION 3.0)
project(LiDARViewer)

set(CMAKE_CXX_STANDARD 14)


find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_executable(LiDARViewer main.cpp tic_toc.h)
target_link_libraries(LiDARViewer ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(CannyViewer canny.cpp LSD_merge.cpp)
target_link_libraries(CannyViewer ${OpenCV_LIBS})

add_executable(LSDViewer LSD_demo.cpp)
target_link_libraries(LSDViewer ${OpenCV_LIBS})

add_executable(mergeLSD LSD_merge.cpp)
target_link_libraries(mergeLSD ${OpenCV_LIBS})