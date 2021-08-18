cmake_minimum_required(VERSION 3.6)

set(SRC negate_stage.cpp hdr_stage.cpp pwl.cpp histogram.cpp motion_detect_stage.cpp)
set(TARGET_LIBS "")

if ("${ENABLE_OPENCV}" STREQUAL "")
    set(ENABLE_OPENCV 1)
endif()

set(OpenCV_FOUND)
if (ENABLE_OPENCV)
    message(STATUS "Checking for OpenCV")
    find_package(OpenCV QUIET)
endif()

if (OpenCV_FOUND)
    message(STATUS "OpenCV library found:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
    include_directories(${OpenCV_INCLUDE_DIRS})
    set(SRC ${SRC} sobel_cv_stage.cpp face_detect_cv_stage.cpp annotate_cv_stage.cpp)
    set(TARGET_LIBS ${OpenCV_LIBS})
else()
    message(WARNING "OpenCV not found, some stages will not be built!")
endif()

add_library(post_processing_stages ${SRC})
target_link_libraries(post_processing_stages ${TARGET_LIBS})
target_compile_definitions(post_processing_stages PUBLIC OPENCV_PRESENT=${OpenCV_FOUND})

install(TARGETS post_processing_stages LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
