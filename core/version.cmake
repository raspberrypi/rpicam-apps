# Script to generate a version string and embed it in the version.cpp source file

if (NOT VERSION_SHA STREQUAL "")
    message("Using user supplied version sha: " ${VERSION_SHA})
endif()

execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/version.py ${VERSION_SHA} OUTPUT_VARIABLE VER)
message("Generating version string: " ${VER})
configure_file(${CMAKE_CURRENT_LIST_DIR}/version.cpp.in version.cpp @ONLY)
