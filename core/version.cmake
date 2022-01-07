# Script to generate a version string and embed it in the version.cpp source file

if (EXISTS ${SOURCE_DIR}/version.gen)
    message("Reading version string from version.gen")
    file(READ ${SOURCE_DIR}/version.gen SHA)
endif()

execute_process(COMMAND ${SOURCE_DIR}/utils/version.py ${SHA}
                WORKING_DIRECTORY ${SOURCE_DIR}
                OUTPUT_VARIABLE VER)

configure_file(${CMAKE_CURRENT_LIST_DIR}/version.cpp.in version.cpp @ONLY)
message("Generating version string: " ${VER})
