# Script to generate a version string and embed it in the version.cpp source file

execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/version.py OUTPUT_VARIABLE VER)
message("Generating version string: " ${VER})
configure_file(${CMAKE_CURRENT_LIST_DIR}/version.cpp.in version.cpp @ONLY)
