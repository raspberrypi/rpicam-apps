cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

pkg_check_modules(LIBDRM QUIET libdrm)
pkg_check_modules(X11 QUIET x11)
pkg_check_modules(EPOXY QUIET epoxy)
pkg_check_modules(QTCORE QUIET Qt5Core)
pkg_check_modules(QTWIDGETS QUIET Qt5Widgets)

set(SRC "preview.cpp")
set(TARGET_LIBS "")

IF (NOT DEFINED ENABLE_DRM)
    SET(ENABLE_DRM 1)
endif()
set(DRM_FOUND 0)
if (ENABLE_DRM AND LIBDRM_FOUND)
    message(STATUS "LIBDRM_LINK_LIBRARIES=${LIBDRM_LINK_LIBRARIES}")
    include_directories(${LIBDRM_INCLUDE_DIRS})
    set(TARGET_LIBS ${TARGET_LIBS} ${LIBDRM_LIBRARIES})
    set(SRC ${SRC} drm_preview.cpp)
    set(DRM_FOUND 1)
    message(STATUS "LIBDRM display mode enabled")
else()
    message(STATUS "LIBDRM display mode will be unavailable!")
endif()

IF (NOT DEFINED ENABLE_X11)
    set(ENABLE_X11 1)
endif()
set(EGL_FOUND 0)
if (ENABLE_X11 AND X11_FOUND AND EPOXY_FOUND)
    message(STATUS "X11_LINK_LIBRARIES=${X11_LINK_LIBRARIES}")
    message(STATUS "EPOXY_LINK_LIBRARIES=${EPOXY_LINK_LIBRARIES}")
    set(TARGET_LIBS ${TARGET_LIBS} ${X11_LIBRARIES} ${EPOXY_LIBRARIES})
    set(SRC ${SRC} egl_preview.cpp)
    set(EGL_FOUND 1)
    message(STATUS "EGL display mode enabled")
else()
    message(STATUS "EGL display mode will be unavailable!")
endif()

IF (NOT DEFINED ENABLE_QT)
    message(STATUS "ENABLE_QT not specified - set to 1")
    set(ENABLE_QT 1)
endif()
set(QT_FOUND 0)
if (ENABLE_QT AND QTCORE_FOUND AND QTWIDGETS_FOUND)
    message(STATUS "QTCORE_LINK_LIBRARIES=${QTCORE_LINK_LIBRARIES}")
    message(STATUS "QTCORE_INCLUDE_DIRS=${QTCORE_INCLUDE_DIRS}")
    include_directories(${QTCORE_INCLUDE_DIRS} ${QTWIDGETS_INCLUDE_DIRS})
    set(TARGET_LIBS ${TARGET_LIBS} ${QTCORE_LIBRARIES} ${QTWIDGETS_LIBRARIES})
    # The qt5/QtCore/qvariant.h header throws a warning, so suppress this.
    # Annoyingly there are two different (incompatible) flags for clang < 10
    # and >= 10, so set both, and supress unknown options warnings.
    set_source_files_properties(qt_preview.cpp PROPERTIES COMPILE_FLAGS
                                "-Wno-unknown-warning-option -Wno-deprecated-copy -Wno-deprecated")
    set(SRC ${SRC} qt_preview.cpp)
    set(QT_FOUND 1)
    message(STATUS "QT display mode enabled")
else()
    message(STATUS "QT display mode will be unavailable!")
endif()

add_library(preview null_preview.cpp ${SRC})
set_target_properties(preview PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})
target_link_libraries(preview ${TARGET_LIBS})

target_compile_definitions(preview PUBLIC LIBDRM_PRESENT=${DRM_FOUND})
target_compile_definitions(preview PUBLIC LIBEGL_PRESENT=${EGL_FOUND})
target_compile_definitions(preview PUBLIC QT_PRESENT=${QT_FOUND})

install(TARGETS preview LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

list(APPEND ${PROJECT_NAME}_HEADERS
    preview.hpp
)

install(FILES
    ${${PROJECT_NAME}_HEADERS}
    DESTINATION
    ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/preview
    COMPONENT Devel
)
