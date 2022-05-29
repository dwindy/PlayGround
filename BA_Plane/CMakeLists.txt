cmake_minimum_required(VERSION 3.17)
project(BA_Plane)

set(CMAKE_CXX_STANDARD 14)
find_package (Eigen3 REQUIRED)

add_executable(BA_Plane main.cpp)
target_link_libraries(BA_Plane Eigen3::Eigen)
target_link_libraries(BA_Plane ${PROJECT_SOURCE_DIR}/g2o/lib/libg2o.so)