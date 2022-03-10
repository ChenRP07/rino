add_executable(test.exe test.cpp)

target_link_libraries(test.exe libCloud.so libCluster.so libconfig.so libOctree.so libzstd.so ${PCL_LIBRARIES} libboost_thread.so.1.71.0)