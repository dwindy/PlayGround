cmake_minimum_required(VERSION 2.8)

project(g2oplayground)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

list(APPEND CMAKE_MODULE_PATH /home/xin/Downloads/g2o/cmake_modules)
set(G2O_ROOT /home/xin/Downloads/g2o)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(CSparse REQUIRED)
find_package(OpenCV 3.0 QUIET)

#SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse g2o_types_sba)

include_directories(${PROJECT_SOURCE_DIR} ${G2O_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${OpenCV_DIRS})
add_executable(playG2OV2 main.cpp)
target_link_libraries(playG2OV2 ${G2O_LIBS} ${OpenCV_LIBS} ${EIGEN3_LIBS})