cmake_minimum_required(VERSION 3.6)

pkg_check_modules(LIBDRM REQUIRED libdrm)

find_package(X11 REQUIRED)
message(STATUS "${X11_INCLUDE_DIR}")
find_library(EPOXY_LIBRARY libepoxy.so REQUIRED)
message(STATUS "${EPOXY_LIBRARY}")
find_library(DRM_LIBRARY libdrm.so REQUIRED)
message(STATUS "${DRM_LIBRARY}")

include_directories(${LIBDRM_INCLUDE_DIRS})

add_library(preview egl_preview.cpp drm_preview.cpp null_preview.cpp)
target_link_libraries(preview ${DRM_LIBRARY} ${X11_LIBRARIES} ${EPOXY_LIBRARY})

install(TARGETS preview LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

