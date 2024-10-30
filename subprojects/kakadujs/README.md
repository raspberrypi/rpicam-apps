# kakadujs

Easy to use wrapper for the Kakadu JPEG2000 library

Includes builds for WASM and native C/C++ on Mac/Linux/Windows

## Status

Experimental - use at your own risk

## Building

This code has been developed/tested against v8_4_1 of Kakadu for Mac (x64/ARM), Linux (x64/ARM) and
Windows (x64). You must place a fresh version of the Kakadu library in the extern folder
(e.g. extern/v_8_4_1-02044N). CMake will find your installation of Kakadu for you.

NOTE - The CMake files are setup to automatically build with the processor SIMD optimizations
without making any changes to the default Kakadu source distribution. You do should NOT replace
srclib_ht with altlib_ht_opt or set FC_BLOCK_ENABLED as described in the Kakadu/Enabling_HT.txt file.

### Prerequisites

#### Linux/Mac OS X

- CMake (v3.20)
- C++ Compiler Toolchain (e.g. Ubuntu build-essentials, XCode command line tools v14.0.0)
- Emscripten (v3.1.25)

#### Windows

- Visual Studio 2022 (Community Edition with Desktop development with C++)

### Building the native C++ version with Linux/Mac OS X

```
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
$ cmake --build build
$ build-clang/test/cpp/cpptest
```

The test app in $BUILD_DIR/test/cpp/main.cpp will generate benchmarks for decoding and encoding.

- TPF = Time Per Frame/Image.
- MP/s = Mega pixels / second
- FPS = Frames per second decoding

Numbers from an Apple M1 MacBook Pro running macOS Monterey 12.6.1

```
build-clang/test/cpp/cpptest
NATIVE decode test/fixtures/j2c/CT1.j2c TotalTime: 0.014 s for 20 iterations; TPF=0.692 ms (361.12 MP/s, 1444.46 FPS)
NATIVE decode test/fixtures/j2c/MG1.j2c TotalTime: 0.744 s for 20 iterations; TPF=37.206 ms (374.94 MP/s, 26.88 FPS)
NATIVE encode test/fixtures/raw/CT1.RAW TotalTime: 0.037 s for 20 iterations; TPF=1.846 ms (135.44 MP/s, 541.74 FPS)
```

Numbers from Intel 13900k on Linux

```
>
build-gcc/test/cpp/cpptest
NATIVE decode test/fixtures/j2c/CT1.j2c TotalTime: 0.009 s for 20 iterations; TPF=0.448 ms (558.05 MP/s, 2232.20 FPS)
NATIVE decode test/fixtures/j2c/MG1.j2c TotalTime: 0.564 s for 20 iterations; TPF=28.177 ms (495.09 MP/s, 35.49 FPS)
NATIVE encode test/fixtures/raw/CT1.RAW TotalTime: 0.009 s for 20 iterations; TPF=0.438 ms (570.52 MP/s, 2282.07 FPS)
```

### Building the native C++ version with Windows/Visual Studio 2022

Build the x64-release version. Run cpp test from the project root directory.

- TPF = Time Per Frame/Image.
- MP/s = Mega pixels / second
- FPS = Frames per second decoding

Numbers from an Intel 13900K running Windows 11 Pro

```
C:\Users\chafe\source\repos\kakadujs>out\build\x64-Release\test\cpp\cpptest.exe
NATIVE decode test/fixtures/j2c/CT1.j2c TotalTime: 0.017 s for 20 iterations; TPF=0.850 ms (294.11 MP/s, 1176.43 FPS)
NATIVE decode test/fixtures/j2c/MG1.j2c TotalTime: 0.568 s for 20 iterations; TPF=28.400 ms (491.19 MP/s, 35.21 FPS)
NATIVE encode test/fixtures/raw/CT1.RAW TotalTime: 0.014 s for 20 iterations; TPF=0.700 ms (357.15 MP/s, 1428.61 FPS)
```

### Building WASM version

Install EMSCRIPTEN or launch the docker container using Visual Studio Code Remote Containers. From the terminal:

```
$ emcmake cmake -S . -B build-emscripten -DCMAKE_BUILD_TYPE=Release -DCMAKE_FIND_ROOT_PATH=/
$ (cd build-emscripten; emmake make -j)
$ build-emscripten/test/cpp/cpptest
```

```
$ build-emscripten/test/cpp/cpptest
WASM decode ../fixtures/j2c/CT1.j2c TotalTime: 0.100 s for 20 iterations; TPF=5.001 ms (49.99 MP/s, 199.95 FPS)
WASM decode ../fixtures/j2c/MG1.j2c TotalTime: 4.090 s for 20 iterations; TPF=204.477 ms (68.22 MP/s, 4.89 FPS)
WASM encode ../fixtures/raw/CT1.RAW TotalTime: 0.074 s for 20 iterations; TPF=3.710 ms (67.38 MP/s, 269.52 FPS)
```
