cmake_minimum_required(VERSION 3.6)

add_library(outputs output.cpp file_output.cpp net_output.cpp circular_output.cpp)

install(TARGETS outputs LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

