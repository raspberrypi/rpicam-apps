cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

add_library(outputs output.cpp file_output.cpp net_output.cpp circular_output.cpp)
set_target_properties(outputs PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})

install(TARGETS outputs LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

list(APPEND ${PROJECT_NAME}_HEADERS
    circular_output.hpp
    file_output.hpp
    net_output.hpp
    output.hpp
)

install(FILES
    ${${PROJECT_NAME}_HEADERS}
    DESTINATION
    ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/output
    COMPONENT Devel
)
