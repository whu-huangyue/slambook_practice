cmake_minimum_required(VERSION 2.6)
project(kecheng5)

find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

# include_directories("/usr/include/eigen3")

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(computeORB computeORB.cpp)
add_executable(E2Rt E2Rt.cpp)
add_executable(gnba GN-BA.cpp)
add_executable(icp icppractice.cpp)

target_link_libraries(computeORB ${OpenCV_LIBS})
target_link_libraries(gnba Sophus::Sophus)

target_link_libraries(icp ${OpenCV_LIBS})
target_link_libraries(icp Sophus::Sophus)
target_link_libraries(icp ${Pangolin_LIBRARIES})