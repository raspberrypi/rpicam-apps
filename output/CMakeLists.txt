cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

add_library(outputs output.cpp file_output.cpp net_output.cpp circular_output.cpp memcached_output.cpp)

install(TARGETS outputs LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

target_link_libraries(outputs -lmemcached)

# <------------ add hiredis dependency --------------->
find_path(HIREDIS_HEADER hiredis)
target_include_directories(outputs PUBLIC ${HIREDIS_HEADER})

find_library(HIREDIS_LIB hiredis)
target_link_libraries(outputs ${HIREDIS_LIB})

# <------------ add redis-plus-plus dependency -------------->
# NOTE: this should be *sw* NOT *redis++*
find_path(REDIS_PLUS_PLUS_HEADER sw)
target_include_directories(outputs PUBLIC ${REDIS_PLUS_PLUS_HEADER})

find_library(REDIS_PLUS_PLUS_LIB redis++)
target_link_libraries(outputs ${REDIS_PLUS_PLUS_LIB})

