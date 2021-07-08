cmake_minimum_required(VERSION 3.6)

add_library(encoders encoder.cpp null_encoder.cpp h264_encoder.cpp mjpeg_encoder.cpp)

install(TARGETS encoders LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

