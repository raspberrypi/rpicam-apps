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
    # OpenCV has so many libraries, we're going to link only the ones we need.
    # But if you add more OpenCV stages, you may need more libraries here!
    set(OpenCV_LIBS_REDUCED -lopencv_core -lopencv_imgproc -lopencv_objdetect)
    message(STATUS "OpenCV library found:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS_REDUCED}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
    include_directories(${OpenCV_INCLUDE_DIRS})
    set(SRC ${SRC} sobel_cv_stage.cpp face_detect_cv_stage.cpp annotate_cv_stage.cpp plot_pose_cv_stage.cpp)
    set(TARGET_LIBS ${OpenCV_LIBS_REDUCED})
else()
    message(WARNING "OpenCV not found, some stages will not be built!")
endif()

set(ENABLE_TFLITE)
if (ENABLE_TFLITE)
    set(SRC ${SRC} object_classify_tf_stage.cpp pose_estimation_tf_stage)
    set(TARGET_LIBS ${TARGET_LIBS} tensorflow-lite)
endif()

add_library(post_processing_stages ${SRC})
target_link_libraries(post_processing_stages ${TARGET_LIBS})
target_compile_definitions(post_processing_stages PUBLIC OPENCV_PRESENT=${OpenCV_FOUND})

install(TARGETS post_processing_stages LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
