
add_executable(cpptest main.cpp)

target_link_libraries(cpptest PRIVATE kakadujs)

target_compile_features(cpptest PRIVATE cxx_std_11)
