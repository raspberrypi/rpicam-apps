cmake_minimum_required(VERSION 3.6)

find_package(Boost REQUIRED COMPONENTS program_options)

add_library(libcamera_app libcamera_app.cpp post_processing_stage.cpp post_processor.cpp)
set_target_properties(libcamera_app PROPERTIES PREFIX "" IMPORT_PREFIX "")
target_link_libraries(libcamera_app pthread images preview ${LIBCAMERA_LINK_LIBRARIES} ${Boost_LIBRARIES} -Wl,--whole-archive post_processing_stages -Wl,--no-whole-archive)

install(TARGETS libcamera_app LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
