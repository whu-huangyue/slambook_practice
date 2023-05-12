cmake_minimum_required(VERSION 2.6)
project(draw)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories("/usr/include/eigen3")

add_executable(draw_trajectory draw_trajectory.cpp)
add_executable(trajectoryError trajectoryError.cpp)

target_link_libraries(draw_trajectory Sophus::Sophus)
target_link_libraries(draw_trajectory ${Pangolin_LIBRARIES})

target_link_libraries(trajectoryError Sophus::Sophus)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})