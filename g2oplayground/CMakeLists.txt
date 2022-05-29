cmake_minimum_required(VERSION 3.00)
project(g2oplayground)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(g2oplayground main.cpp Converter.cc)
target_link_libraries(g2oplayground  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${EIGEN3_LIBS})

