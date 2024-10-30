if(EMSCRIPTEN)
  add_executable(kakadujs ${SOURCES} jslib.cpp)

  target_link_libraries(kakadujs PRIVATE kakadu kakaduappsupport)

  if(KAKADU_THREADING)
    message(FATAL_ERROR "EMSCRIPTEN builds have not been tested with threading enabled yet")
  endif()

  # Explicitly turn off Kakadu threading to prevent compile errors
  # due to kakadu not knowing about EMSCRIPTEN pthreads.  EMSCRIPTEN
  # does support threads, so this might work but it requires extra
  # deployment work (enabling SharedArrayBuffers) and Kakadujs hasn't
  # been tested with it yet
  add_compile_definitions(KDU_NO_THREADS)

  set_target_properties(
    kakadujs
    PROPERTIES
    LINK_FLAGS "\
        -O3 \
        -lembind \
        -s DISABLE_EXCEPTION_CATCHING=1 \
        -s ASSERTIONS=0 \
        -s NO_EXIT_RUNTIME=1 \
        -s MALLOC=emmalloc \
        -s ALLOW_MEMORY_GROWTH=1 \
        -s INITIAL_MEMORY=50MB \
        -s FILESYSTEM=0 \
        -s EXPORTED_FUNCTIONS=[] \
        -s EXPORTED_RUNTIME_METHODS=[ccall] \
    ")

else() # C++ header only library
  add_library(kakadujs INTERFACE)
  message(STATUS "${SOURCES}")
  target_link_libraries(kakadujs INTERFACE kakaduappsupport kakadu)
  target_include_directories(kakadujs INTERFACE ".")
endif()