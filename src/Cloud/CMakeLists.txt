add_library(Cloud SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Cloud.cpp )

target_link_libraries(Cloud ${PCL_LIBRARIES} libconfig.so libCluster.so)
