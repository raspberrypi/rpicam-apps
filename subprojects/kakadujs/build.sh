#!/bin/bash

# this script does a clean build and is useful for development when modifying cmakefiles
clear
rm -rf build

export CMAKE_BUILD_TYPE=Release # | Debug
#export VERBOSE=1

(cmake -S . -B build -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE) || (exit 1;)
(cmake --build build --  -j) || (exit 1;)
build/test/cpp/cpptest
