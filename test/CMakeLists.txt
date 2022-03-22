add_executable(decoder.exe decoder.cpp)

target_link_libraries(decoder.exe libCloud.so libCluster.so libconfig.so libOctree.so libzstd.so ${PCL_LIBRARIES} libboost_thread.so.1.71.0 parallel_decoder.so)

add_executable(psnr.exe ass.cpp)

target_link_libraries(psnr.exe libCloud.so libCluster.so libconfig.so libOctree.so libzstd.so ${PCL_LIBRARIES} libboost_thread.so.1.71.0)

add_executable(encoder.exe encoder.cpp)

target_link_libraries(encoder.exe libCloud.so libCluster.so libconfig.so libOctree.so libzstd.so ${PCL_LIBRARIES} libboost_thread.so.1.71.0)

add_executable(test.exe test.cpp)

target_link_libraries(test.exe libCloud.so libCluster.so libconfig.so libOctree.so libzstd.so ${PCL_LIBRARIES} libboost_thread.so.1.71.0)