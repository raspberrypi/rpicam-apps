cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

if (NOT DEFINED ENABLE_LIBAV)
    set(ENABLE_LIBAV 1)
endif()

set(LIBAV_PRESENT 0)
set(SRC encoder.cpp null_encoder.cpp h264_encoder.cpp mjpeg_encoder.cpp)
set(TARGET_LIBS images)

if (ENABLE_LIBAV)
    message(STATUS "Checking for libavcodec")
    pkg_check_modules(LIBAV QUIET IMPORTED_TARGET
        libavcodec
        libavdevice
        libavformat
        libswresample
    )
    if (LIBAV_FOUND)
        include_directories(${LIBAV_INCLUDE_DIRS})
        set(SRC ${SRC} libav_encoder.cpp)
        set(TARGET_LIBS ${TARGET_LIBS} ${LIBAV_LIBRARIES})
        set(LIBAV_PRESENT 1)
        message(STATUS "libavcodec found:")
        message(STATUS "    libraries: ${LIBAV_LIBRARIES}")
    endif()
else()
    message(STATUS "Omitting libavcodec")
endif()

add_library(encoders ${SRC})
set_target_properties(encoders PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})
target_link_libraries(encoders ${TARGET_LIBS})
target_compile_definitions(encoders PUBLIC LIBAV_PRESENT=${LIBAV_PRESENT})

install(TARGETS encoders LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

list(APPEND ${PROJECT_NAME}_HEADERS
    encoder.hpp
    h264_encoder.hpp
    libav_encoder.hpp
    mjpeg_encoder.hpp
    null_encoder.hpp
)

install(FILES
    ${${PROJECT_NAME}_HEADERS}
    DESTINATION
    ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/encoder
    COMPONENT Devel
)
