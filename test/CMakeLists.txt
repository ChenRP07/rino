add_executable(test.exe test.cpp)

target_link_libraries(test.exe libCloud.so libCluster.so libconfig.so)