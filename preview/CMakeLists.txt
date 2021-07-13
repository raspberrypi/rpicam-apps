cmake_minimum_required(VERSION 3.6)

pkg_check_modules(LIBDRM QUIET libdrm)
pkg_check_modules(X11 QUIET x11)
pkg_check_modules(EPOXY QUIET epoxy)

set(SRC "")
set(TARGET_LIBS "")

set(DRM_FOUND 0)
if (LIBDRM_FOUND)
    message(STATUS "LIBDRM_LINK_LIBRARIES=${LIBDRM_LINK_LIBRARIES}")
    include_directories(${LIBDRM_INCLUDE_DIRS})
    set(TARGET_LIBS ${TARGET_LIBS} ${LIBDRM_LIBRARIES})
    set(SRC ${SRC} drm_preview.cpp)
    set(DRM_FOUND 1)
else()
    message(WARNING "LIBDRM not found, this display mode will be unavailable!")
endif()

set(EGL_FOUND 0)
if (X11_FOUND AND EPOXY_FOUND)
    message(STATUS "X11_LINK_LIBRARIES=${X11_LINK_LIBRARIES}")
    message(STATUS "EPOXY_LINK_LIBRARIES=${EPOXY_LINK_LIBRARIES}")
    set(TARGET_LIBS ${TARGET_LIBS} ${X11_LIBRARIES} ${EPOXY_LIBRARIES})
    set(SRC ${SRC} egl_preview.cpp)
    set(EGL_FOUND 1)
else()
    message(WARNING "EGL libraries not found, this display mode will be unavailable!")
endif()

add_library(preview null_preview.cpp ${SRC})
target_link_libraries(preview ${TARGET_LIBS})

target_compile_definitions(preview PUBLIC LIBDRM_PRESENT=${DRM_FOUND})
target_compile_definitions(preview PUBLIC LIBEGL_PRESENT=${EGL_FOUND})

install(TARGETS preview LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

