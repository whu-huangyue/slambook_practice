CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(BA_G2O)
# set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_STANDARD 17)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# LIST(APPEND CMAKE_MODULE_PATH /home/hy/g2o/cmake_modules ) 

# SET( G2O_ROOT /usr/local/include/g2o )
# include_directories(${G2O_INCLUDE_DIRS})

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
include_directories(${G2O_INCLUDE_DIRS})

add_library(bal_common common.cpp)
add_executable(bundle_adjustment_g2o bundle_adjustment_g2o.cpp)

target_link_libraries(bundle_adjustment_g2o ${G2O_LIBS} bal_common fmt)