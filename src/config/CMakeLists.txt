add_library(config SHARED ${CMAKE_CURRENT_SOURCE_DIR}/config.cpp )

target_link_libraries(config ${PCL_LIBRARIES})
