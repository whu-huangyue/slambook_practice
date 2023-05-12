cmake_minimum_required(VERSION 2.6)
project(kecheng4)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(Eigen3 REQUIRED)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(undistort_image undistort_image.cpp)
add_executable(disparity disparity.cpp)
add_executable(gaussnewton gaussnewton.cpp)

target_link_libraries(undistort_image ${OpenCV_LIBS})
target_link_libraries(disparity ${OpenCV_LIBS})
target_link_libraries(disparity ${Pangolin_LIBRARIES})
target_link_libraries(gaussnewton ${OpenCV_LIBS})