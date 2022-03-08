cmake_minimum_required(VERSION 3.7)
project(rino)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILECOMMANDS ON)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib/)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Threads)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_SOURCE_DIR}/include/)
link_directories(${CMAKE_SOURCE_DIR}/lib/)

aux_source_directory(. DIR_SRCS)
add_subdirectory(${CMAKE_SOURCE_DIR}/src)