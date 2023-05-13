# cmake_minimum_required(VERSION 2.6)
# project(kecheng6)

# find_package(OpenCV REQUIRED)
# find_package(Sophus REQUIRED)
# find_package(Pangolin REQUIRED)

# include_directories(${OpenCV_INCLUDE_DIRS})

# add_executable(optical_flow optical_flow.cpp)
# target_link_libraries(optical_flow ${OpenCV_LIBS})

# add_executable(direct_method direct_method.cpp)
# target_link_libraries(direct_method ${OpenCV_LIBS})
# target_link_libraries(direct_method Sophus::Sophus)
# target_link_libraries(direct_method ${Pangolin_LIBRARIES})

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

PROJECT(ComputeDis)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(comupteDisparity computeDisparity.cpp)

target_link_libraries(comupteDisparity ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} fmt)
