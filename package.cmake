# Script to generate a version string and save it to the package source root

execute_process(COMMAND git rev-parse HEAD
                WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                OUTPUT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/version.gen)
