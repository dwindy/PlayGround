cmake_minimum_required(VERSION 3.21)
project(LiDARVisionG2O)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(Sophus REQUIRED)

if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif ()

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
)

add_executable(LiDARVisionG2O main.cpp Converter.cc)
target_link_libraries(LiDARVisionG2O ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Sophus})
