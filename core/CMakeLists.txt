cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS program_options)

add_custom_target(VersionCpp ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR} -P ${CMAKE_CURRENT_LIST_DIR}/version.cmake)
set_source_files_properties(version.cpp PROPERTIES GENERATED 1)

add_library(libcamera_app libcamera_app.cpp post_processor.cpp version.cpp options.cpp)
add_dependencies(libcamera_app VersionCpp)

set_target_properties(libcamera_app PROPERTIES PREFIX "" IMPORT_PREFIX "" VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})
target_link_libraries(libcamera_app pthread preview ${LIBCAMERA_LINK_LIBRARIES} ${Boost_LIBRARIES} post_processing_stages)

install(TARGETS libcamera_app LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

list(APPEND ${PROJECT_NAME}_HEADERS
    completed_request.hpp
    frame_info.hpp
    libcamera_app.hpp
    libcamera_encoder.hpp
    logging.hpp
    metadata.hpp
    options.hpp
    post_processor.hpp
    still_options.hpp
    stream_info.hpp
    version.hpp
    video_options.hpp
)

install(FILES
    ${${PROJECT_NAME}_HEADERS}
    DESTINATION
    ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/core
    COMPONENT Devel
)
