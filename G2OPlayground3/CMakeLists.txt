cmake_minimum_required(VERSION 3.21)
project(G2OPlayground3)

set(CMAKE_CXX_STANDARD 11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

FIND_PACKAGE(G2O REQUIRED)
IF(G2O_FOUND)
    message("G2O include is found:" ${G2O_INCLUDE_DIRS})
    message("G2O lib:" ${G2O_LIBRARIES})
ENDIF(G2O_FOUND)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(CSparse REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

#SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse g2o_types_sba g2o_solver_csparse g2o_types_slam3d g2o_types_slam2d)
SET(G2O_LIBS cxsparse g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 g2o_solver_cholmod)
include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${OpenCV_DIRS} ${G2O_INCLUDE_DIRS})

add_executable(G2OPlayground3 main.cpp Converter.cc)
target_link_libraries(G2OPlayground3 ${G2O_LIBS} ${OpenCV_LIBS} ${CHOLMOD_LIBRARIES} cholmod)