cmake_minimum_required(VERSION 3.2)
project(MagnetometerFeatureAugumentationLocation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O4 -msse2 -msse3 -msse4 -lopencv_aruco -lpthread ")
find_package(Eigen3 REQUIRED)

if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# G2O
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O)
FIND_PACKAGE(CSparse)

#SET(G2O_LIB_TYPE SHARE)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ./include
        /usr/local/cuda/include
        /usr/include
        /usr/local/include
        ${CSPARSE_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
)

link_directories(
        /usr/local/lib
        /usr/lib
        #        ~/Code/g2o/libCMakeLists.txt:60
        #${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib
        ./include
)


SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator
        g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper
        g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3 cxsparse
        g2o_types_slam3d_addons)


set(SOURCE_FILES main.cpp)
add_executable(MagnetometerFeatureAugumentationLocation ${SOURCE_FILES})


add_executable(graph_build src/graph_build_test.cpp)
target_link_libraries(graph_build ${G2O_LIBS} ${CSPARSE_LIBRARY})
