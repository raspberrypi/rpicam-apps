cmake_minimum_required(VERSION 3.6)

find_library(TIFF_LIBRARY tiff REQUIRED)

project(cinepi-raw)
add_executable(cinepi-raw cinepi_raw.cpp dng_encoder.cpp cinepi_controller.cpp cinepi_state.cpp)
target_link_libraries(cinepi-raw libcamera_app encoders outputs tiff)

# <---------- set c++ standard ------------->
# NOTE: you must build redis-plus-plus and your application code with the same standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# <------------ add hiredis dependency --------------->
find_path(HIREDIS_HEADER hiredis)
target_include_directories(cinepi-raw PUBLIC ${HIREDIS_HEADER})

find_library(HIREDIS_LIB hiredis)
target_link_libraries(cinepi-raw ${HIREDIS_LIB})

# <------------ add redis-plus-plus dependency -------------->
# NOTE: this should be *sw* NOT *redis++*
find_path(REDIS_PLUS_PLUS_HEADER sw)
target_include_directories(cinepi-raw PUBLIC ${REDIS_PLUS_PLUS_HEADER})

find_library(REDIS_PLUS_PLUS_LIB redis++)
target_link_libraries(cinepi-raw ${REDIS_PLUS_PLUS_LIB})

set(EXECUTABLES cinepi-raw)

set(EXECUTABLE_OUTPUT_PATH  ${CMAKE_BINARY_DIR})
install(TARGETS ${EXECUTABLES} RUNTIME DESTINATION bin)
