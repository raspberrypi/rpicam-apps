cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

add_library(encoders encoder.cpp null_encoder.cpp h264_encoder.cpp mjpeg_encoder.cpp)
target_link_libraries(encoders jpeg)

install(TARGETS encoders LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

