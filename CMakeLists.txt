cmake_minimum_required(VERSION 3.6)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    message(STATUS "No previous build - default to Release build")
  endif()
endif()

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE CXX_FLAGS "-Wall -Wextra -pedantic -Wno-unused-parameter -faligned-new")
add_definitions(-Wfatal-errors)
add_definitions(-Wno-psabi)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-mfpu=neon-fp-armv8 -ftree-vectorize)

project(common)
add_library(common output.cpp file_output.cpp net_output.cpp circular_output.cpp egl_preview.cpp drm_preview.cpp jpeg.cpp)

project(encoders)
add_library(encoders encoder.cpp null_encoder.cpp h264_encoder.cpp mjpeg_encoder.cpp)

project(libcamera-still)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CAMERA REQUIRED camera)

include_directories(. "${CAMERA_INCLUDE_DIRS}" /usr/include/libdrm)

find_package( Boost REQUIRED COMPONENTS program_options )

find_library(EXIF_LIBRARY exif REQUIRED)
find_library(JPEG_LIBRARY jpeg REQUIRED)
find_library(TIFF_LIBRARY tiff REQUIRED)
find_library(PNG_LIBRARY png REQUIRED)
find_library(LIBCAMERA_LIBRARY libcamera.so REQUIRED)
message(STATUS "${LIBCAMERA_LIBRARY}")
find_package(X11 REQUIRED)
message(STATUS "${X11_INCLUDE_DIR}")
find_library(EPOXY_LIBRARY libepoxy.so REQUIRED)
message(STATUS "${EPOXY_LIBRARY}")
find_library(DRM_LIBRARY libdrm.so REQUIRED)
message(STATUS "${DRM_LIBRARY}")

find_library(IPA_RPI_LIBRARY ipa_rpi.so PATHS "${CAMERA_LIBRARY_DIRS}/libcamera" REQUIRED)
message(STATUS "${IPA_RPI_LIBRARY}")

add_executable(libcamera-still libcamera_still.cpp yuv.cpp dng.cpp png.cpp bmp.cpp)
target_link_libraries(libcamera-still common "${LIBCAMERA_LIBRARY}" "${IPA_RPI_LIBRARY}" exif ${Boost_LIBRARIES} jpeg tiff png pthread "${X11_LIBRARIES}" "${EPOXY_LIBRARY}" "${DRM_LIBRARY}")

project(libcamera-vid)

add_executable(libcamera-vid libcamera_vid.cpp)
target_link_libraries(libcamera-vid encoders common "${LIBCAMERA_LIBRARY}" "${IPA_RPI_LIBRARY}" ${Boost_LIBRARIES} pthread jpeg "${X11_LIBRARIES}" "${EPOXY_LIBRARY}" "${DRM_LIBRARY}")

project(libcamera-hello)

add_executable(libcamera-hello libcamera_hello.cpp)
target_link_libraries(libcamera-hello common "${LIBCAMERA_LIBRARY}" "${IPA_RPI_LIBRARY}" ${Boost_LIBRARIES} pthread "${X11_LIBRARIES}" "${EPOXY_LIBRARY}" "${DRM_LIBRARY}")

project(libcamera-raw)

add_executable(libcamera-raw libcamera_raw.cpp)
target_link_libraries(libcamera-raw encoders common "${LIBCAMERA_LIBRARY}" "${IPA_RPI_LIBRARY}" jpeg ${Boost_LIBRARIES} pthread "${X11_LIBRARIES}" "${EPOXY_LIBRARY}" "${DRM_LIBRARY}")

project(libcamera-jpeg)

add_executable(libcamera-jpeg libcamera_jpeg.cpp)
target_link_libraries(libcamera-jpeg common "${LIBCAMERA_LIBRARY}" "${IPA_RPI_LIBRARY}" exif jpeg ${Boost_LIBRARIES} pthread "${X11_LIBRARIES}" "${EPOXY_LIBRARY}" "${DRM_LIBRARY}")
