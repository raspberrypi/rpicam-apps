cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

find_library(EXIF_LIBRARY exif REQUIRED)
find_library(JPEG_LIBRARY jpeg REQUIRED)
find_library(TIFF_LIBRARY tiff REQUIRED)
find_library(PNG_LIBRARY png REQUIRED)

add_library(images bmp.cpp yuv.cpp jpeg.cpp png.cpp dng.cpp)
set_target_properties(images PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})
target_link_libraries(images jpeg exif png tiff)

install(TARGETS images LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
