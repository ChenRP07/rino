add_executable(decoder.exe decoder.cpp)

target_link_libraries(decoder.exe libparallel_decoder.so)

add_executable(psnr.exe psnr.cpp)

target_link_libraries(psnr.exe ${PCL_LIBRARIES})

add_executable(encoder.exe encoder.cpp)

target_link_libraries(encoder.exe libparallel_encoder.so)

