cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

add_library(outputs output.cpp file_output.cpp net_output.cpp circular_output.cpp)

install(TARGETS outputs LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

