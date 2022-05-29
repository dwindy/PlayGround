##############################################################################
# CUDA configuration
PROJECT("VaFRIC")


CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

SET(
  SOURCES
  ./src/VaFRIC/VaFRIC.h
  ./src/VaFRIC/VaFRIC.cpp
)

FIND_PACKAGE(OpenMP)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(CVD REQUIRED)
FIND_PACKAGE(Cg REQUIRED)
FIND_PACKAGE(Boost REQUIRED)



# C++ only includes
INCLUDE_DIRECTORIES(public:
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_BINARY_DIR} # for configuration files
        )


INCLUDE_DIRECTORIES(
  ${CVD_INCLUDE_DIR}
  ${CG_INCLUDE_PATH}
)

LINK_LIBRARIES(
  ${LIBRARY_NAME}
  ${GLEW_LIBRARY}
  ${CG_LIBRARY}
  ${CG_GL_LIBRARY}
  ${Boost_LIBRARIES}
  ${CVD_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${FREEGLUT_LIBRARY}
    -lGVars3
    -lboost_system    
    -llapack
    -lblas
  )

ADD_EXECUTABLE(test_vaFRIC
  ./src/test_vaFRIC.cpp
)

