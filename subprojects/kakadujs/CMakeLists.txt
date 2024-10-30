cmake_minimum_required(VERSION 3.20)

project (kakadujs LANGUAGES CXX)

# search for the kakadu source code in extern
#find_path(KAKADU_ROOT NAMES Enabling_HT.txt HINTS ${CMAKE_SOURCE_DIR}/ext/kakadujs/extern/* REQUIRED NO_CACHE)
find_path(KAKADU_ROOT NAMES Enabling_HT.txt HINTS ${CMAKE_SOURCE_DIR}/extern/* REQUIRED NO_CACHE)

# set the build type if not specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}")
endif()

# force KAKADU_THREADING off as we haven't tested it yet.  
SET(KAKADU_THREADING OFF CACHE BOOL "Kakadu Threading has not been tested yet" FORCE)  # TODO: Test with threading enabled

# add the kakadu library from extern
add_subdirectory(extern/kakadu EXCLUDE_FROM_ALL)

# add the library code
add_subdirectory(src)

# c++ native test case
if(NOT EMSCRIPTEN)
  add_subdirectory(test/cpp)
endif()
