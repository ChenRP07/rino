add_library(parallel_encoder
            SHARED
            config.cpp Cluster.cpp Cloud.cpp Octree.cpp parallel_encoder.cpp)
target_link_libraries(parallel_encoder PUBLIC libzstd.so ${PCL_LIBRARIES})

add_library(parallel_decoder
            SHARED
            config.cpp Cluster.cpp Cloud.cpp Octree.cpp parallel_decoder.cpp)
target_link_libraries(parallel_decoder PUBLIC libzstd.so ${PCL_LIBRARIES})

