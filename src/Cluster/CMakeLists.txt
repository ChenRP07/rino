cmake_minimum_required(VERSION 2.6)

add_library(Cluster SHARED ${CMAKE_SOURCE_DIR}/src/Cluster/Cluster.cpp )

target_link_libraries(Cluster ${PCL_LIBRARIES})
