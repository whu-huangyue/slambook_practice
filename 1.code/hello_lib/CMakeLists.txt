cmake_minimum_required(VERSION 3.5)
SET(CMAKE_BUILD_TYPE "Release")
PROJECT(SAYHELLO)
#添加头文件
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
#添加可执行程序
ADD_EXECUTABLE(sayhello ${PROJECT_SOURCE_DIR}/src/useHello.cpp)
#添加库，链接
ADD_LIBRARY(hello SHARED ${PROJECT_SOURCE_DIR}/src/hello.cpp)
TARGET_LINK_LIBRARIES(sayhello hello)
#install
INSTALL(TARGETS hello LIBRARY DESTINATION /usr/local/lib)
INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION /usr/local/include)

# cmake_minimum_required(VERSION 3.5)
# SET(CMAKE_BUILD TYPE "Release")
# PROJECT(SAYHELLO)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# ADD_EXECUTABLE(sayhello useHello.cpp)

# ADD_LIBRARY(hello SHARED %{PROJECT_SOURCE_DIR}/src/hello.cpp)
# TARGET_LINK_LIBRARIES(sayhello hello)

# INSTALL(TARGETS hello LIBRARY DESTIMATION /usr/local/lib)
# INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION /usr/local/include)