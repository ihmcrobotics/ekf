cmake_minimum_required(VERSION 2.8)

project(NativeFilterMatrixOps)

find_package(JNI REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(${JNI_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES NativeFilterMatrixOps.cpp)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../libs)

add_library(NativeFilterMatrixOps SHARED ${SOURCE_FILES})
target_link_libraries (NativeFilterMatrixOps Eigen3::Eigen)