add_library(Cluster SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Cluster.cpp )

target_link_libraries(Cluster ${PCL_LIBRARIES} libconfig.so)
