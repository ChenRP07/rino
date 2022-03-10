add_library(Octree SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Octree.cpp )

target_link_libraries(Octree ${PCL_LIBRARIES} libCloud.so libCluster.so libconfig.so libzstd.so)
