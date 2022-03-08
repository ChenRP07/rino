cmake_minimum_required(VERSION 2.6)

add_library(Cloud SHARED ${CMAKE_SOURCE_DIR}/src/Cloud/Cloud.cpp )

target_link_libraries(Cloud ${PCL_LIBRARIES})
