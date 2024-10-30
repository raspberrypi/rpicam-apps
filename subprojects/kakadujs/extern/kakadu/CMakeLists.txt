# Enable SIMD by default
OPTION(KAKADU_SIMD_ACCELERATION "Enable Kakadu's heavily optimized implementation of HTJ2K" ON)

# NOTE - Has not been tested yet
option(KAKADU_THREADING "Build Kakadu with threading" OFF)

# disable threads if not enabled
if(NOT KAKADU_THREADING)
    add_compile_definitions(KDU_NO_THREADS)
endif()

# configure for compiler options and link directories based on SIMD Acceleration enabled or not
if(KAKADU_SIMD_ACCELERATION)
    if(EXISTS "${KAKADU_ROOT}/altlib_ht_opt/${KAKADU_PLATFORM}")
        message("KAKADU SIMD Acceleration ENABLED using altlib_ht_opt")
        set(KAKDU_HT_LIB "altlib_ht_opt") # this has the CPU optimized libraries (see Enabling_HT.txt)
    else()
        message("KAKADU SIMD Acceleration ENABLED using srclib_ht")
        set(KAKDU_HT_LIB "srclib_ht") # this has the CPU optimized libraries (see Enabling_HT.txt)
    endif()
else()
    message("KAKADU SIMD Acceleration DISABLED")
    add_compile_definitions(FBC_NO_ACCELERATION) # disable kakadu HT acceleration (see Enabling_HT.txt)
    set(KAKDU_HT_LIB "srclib_ht") # this has the CPU optimized libraries (see Enabling_HT.txt)
endif()

# do platform specific stuff
if(EMSCRIPTEN)
    SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Shared libraries forced off for EMSCRIPTEN" FORCE) # EMSCRIPTEN does not support shared libraries
    SET(KAKADU_THREADING OFF CACHE BOOL "Kakadu threading forced off for EMSCRIPTEN" FORCE) # EMSCRIPTEN threading has not been tested with kakadu yet
    SET(KAKADU_SIMD_ACCELERATION OFF CACHE BOOL "Kakadu SIMD acceleration forced off for EMSCRIPTEN" FORCE) # Kakadu does not support WASM-SIMD yet

    add_compile_options(-msimd128) # enabled LLVM autovectoring for WASM SIMD
elseif(UNIX AND(NOT EMSCRIPTEN))
    if(BUILD_SHARED_LIBS)
        add_compile_options(-fPIC) # enable position independent code for shared libraries
    endif()

    # figure out what architecture we are running on
    EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
    message(STATUS "ARCHITECURE=${ARCHITECTURE}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Darwin") # Mac
        add_compile_definitions(KDU_MAC_SPEEDUPS) # enables cpu specific optimizations for mac (ARM | x86)

        if(ARCHITECTURE STREQUAL "arm64")
            set(KAKADU_PLATFORM "Mac-arm-64-gcc")
        else()
            add_compile_definitions(KDU_X86_INTRINSICS) # TODO - test removing this
            set(KAKADU_PLATFORM "Mac-x86-64-gcc")
        endif()
    else() # Non mac unix (linux)
        if(ARCHITECTURE STREQUAL "aarch64")
            add_compile_options(-march=armv8.1-a) # TODO - try removing this and see if it builds (this was taken from kakadu makefile)
            add_compile_options(-flax-vector-conversions) # TODO - try compiling without this (this was taken from kakadu makefile)
            set(KAKADU_PLATFORM "Linux-arm-64-gcc")
        else()
            set(KAKADU_PLATFORM "Linux-x86-64-gcc")
        endif()
    endif()
elseif(WIN32)
    set(ARCHITECTURE "x86_64")
    set(KAKADU_PLATFORM "Win-x86-64")
else()
    message(FATAL_ERROR "Platform not supported yet")
endif()

set(PUBLIC_HEADERS
    "${KAKADU_ROOT}/coresys/common"
    "${KAKADU_ROOT}/coresys/shared"
)

set(FBC_HEADERS
    "${KAKADU_ROOT}/coresys/fast_coding/"
)

set(PUBLIC_HEADERS_APP_SUPPORT
    "${KAKADU_ROOT}/apps/support"
    "${KAKADU_ROOT}/apps/compressed_io"
)

set(KAKADUJS_SOURCES
    "ensure_config.cpp" # no code, just verifies that the preprocessor definitions are correct since the defaults changed between v8_3 and v8_4_1
)

set(CODING_SOURCES
    "${KAKADU_ROOT}/coresys/coding/block_coding_common.cpp"
    "${KAKADU_ROOT}/coresys/coding/block_decoder.cpp"
    "${KAKADU_ROOT}/coresys/coding/block_encoder.cpp"
    "${KAKADU_ROOT}/coresys/coding/decoder.cpp"
    "${KAKADU_ROOT}/coresys/coding/encoder.cpp"
    "${KAKADU_ROOT}/coresys/coding/cplex_analysis.cpp"
    "${KAKADU_ROOT}/coresys/coding/mq_decoder.cpp"
    "${KAKADU_ROOT}/coresys/coding/mq_encoder.cpp"
)

set(COMPRESSED_SOURCES
    "${KAKADU_ROOT}/coresys/compressed/blocks.cpp"
    "${KAKADU_ROOT}/coresys/compressed/codestream.cpp"
    "${KAKADU_ROOT}/coresys/compressed/compressed.cpp"
)
set(SHARED_SOURCES
    "${KAKADU_ROOT}/coresys/shared/core_local.cpp"
)

set(KERNEL_SOURCES
    "${KAKADU_ROOT}/coresys/kernels/kernels.cpp"
)

set(MESSAGING_SOURCES
    "${KAKADU_ROOT}/coresys/messaging/messaging.cpp"
)

set(PARAMETERS_SOURCES
    "${KAKADU_ROOT}/coresys/parameters/params.cpp"
)

set(TRANSFORM_SOURCES
    "${KAKADU_ROOT}/coresys/transform/colour.cpp"
    "${KAKADU_ROOT}/coresys/transform/transform.cpp"
    "${KAKADU_ROOT}/coresys/transform/analysis.cpp"
    "${KAKADU_ROOT}/coresys/transform/synthesis.cpp"
    "${KAKADU_ROOT}/coresys/transform/fusion.cpp"
    "${KAKADU_ROOT}/coresys/transform/multi_transform.cpp"
    "${KAKADU_ROOT}/coresys/transform/analysis2.cpp"
    "${KAKADU_ROOT}/coresys/transform/synthesis2.cpp"
    "${KAKADU_ROOT}/coresys/transform/transform2.cpp"
)

set(ROI_SOURCES
    "${KAKADU_ROOT}/coresys/roi/roi.cpp"
)

set(COMMON_SOURCES
    "${KAKADU_ROOT}/coresys/common/kdu_arch.cpp"
    "${KAKADU_ROOT}/coresys/threads/kdu_threads.cpp"
)

set(FASTCODING_SOURCES
    "${KAKADU_ROOT}/coresys/fast_coding/fbc_common.cpp"
    "${KAKADU_ROOT}/coresys/fast_coding/fbc_encoder.cpp"
    "${KAKADU_ROOT}/coresys/fast_coding/fbc_decoder.cpp"
    "${KAKADU_ROOT}/coresys/fast_coding/fbc_encoder_tools.cpp"
    "${KAKADU_ROOT}/coresys/fast_coding/fbc_decoder_tools.cpp"
)

set(APP_SUPPORT_SOURCES
    "${KAKADU_ROOT}/apps/support/kdu_stripe_decompressor.cpp"
    "${KAKADU_ROOT}/apps/support/kdu_stripe_compressor.cpp"
    "${KAKADU_ROOT}/apps/support/supp_local.cpp"
    "${KAKADU_ROOT}/apps/jp2/jp2.cpp"
    "${KAKADU_ROOT}/apps/jp2/jpx.cpp"
    "${KAKADU_ROOT}/apps/client_server/kdu_client_window.cpp"
)

# include platform specific code
if(WIN32)
    ENABLE_LANGUAGE(ASM_MASM)
    set(WIN32_SOURCES
        "${KAKADU_ROOT}/coresys/common/arch_masm64.asm"
    )
endif()

# include SIMD acelerated code
if(KAKADU_SIMD_ACCELERATION)
    if(ARCHITECTURE STREQUAL "x86_64")
        add_compile_definitions(KDU_X86_INTRINSICS) # enable x86 SIMD optimizations

        set(SSSE3_SOURCES
            "${KAKADU_ROOT}/coresys/coding/ssse3_coder_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/ssse3_colour_local.cpp"
            "${KAKADU_ROOT}/coresys/transform//ssse3_dwt_local.cpp"
        )

        set(SSE4_SOURCES
            "${KAKADU_ROOT}/coresys/transform/sse4_multi_transform_local.cpp"
        )

        set(AVX_SOURCES
            "${KAKADU_ROOT}/coresys/coding/avx_coder_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/avx_colour_local.cpp"
        )

        set(AVX2_SOURCES
            "${KAKADU_ROOT}/coresys/coding/avx2_coder_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/avx2_colour_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/avx2_dwt_local.cpp"
        )

        set(AVX2_X64_SOURCES
            "${KAKADU_ROOT}/coresys/transform/avx2_analysis2.cpp"
            "${KAKADU_ROOT}/coresys/transform/avx2_synthesis2.cpp"
        )

        set(APP_SUPPORT_SOURCES_INTEL_SIMD
            "${KAKADU_ROOT}/apps/support/avx2_stripe_transfer.cpp"
            "${KAKADU_ROOT}/apps/support/ssse3_stripe_transfer.cpp"
        )

        if(UNIX)
            add_compile_options(-msse2)
            add_compile_options(-mssse3)
            add_compile_options(-msse4.1)
            add_compile_options(-mavx)
            add_compile_options(-mavx2)
            add_compile_options(-mfma)
            add_compile_options(-mbmi)
            add_compile_options(-mbmi2)
            add_compile_options(-mlzcnt)
            add_compile_options(-m64)
        endif()

    elseif(ARCHITECTURE STREQUAL "arm64")
        add_compile_definitions(KDU_NEON_INTRINSICS) # enable ARM NEON SIMD optimizations

        set(NEON_SOURCES
            "${KAKADU_ROOT}/coresys/coding/neon_coder_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/neon_colour_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/neon_dwt_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/neon_multi_transform_local.cpp"
            "${KAKADU_ROOT}/coresys/transform/neon_synthesis2.cpp"
            "${KAKADU_ROOT}/coresys/transform/neon_analysis2.cpp"
        )

        set(NEON_APP_SUPPORT
            "${KAKADU_ROOT}/apps/support/neon_stripe_transfer.cpp"
            "${KAKADU_ROOT}/apps/support/neon_region_decompressor.cpp"
            "${KAKADU_ROOT}/apps/support/neon_region_compositor.cpp"
            "${KAKADU_ROOT}/apps/image/neon_sample_reorg.cpp"
        )
    endif() # arm64
endif() # KAKADU_SIMD_ACCELERATION

# include non SIMD acelerated code
if(NOT KAKADU_SIMD_ACCELERATION)
    if(ARCHITECTURE STREQUAL "arm64")
        add_compile_definitions("KDU_NO_NEON") # explicitly disable ARM NEON SIMD code as it gets turned on by default
    endif()
endif()

# # Kakadu Library
add_library(kakadu
    ${KAKADUJS_SOURCES}
    ${SHARED_SOURCES}
    ${CODING_SOURCES}
    ${COMPRESSED_SOURCES}
    ${KERNEL_SOURCES}
    ${MESSAGING_SOURCES}
    ${PARAMETERS_SOURCES}
    ${TRANSFORM_SOURCES}
    ${ROI_SOURCES}
    ${COMMON_SOURCES}
    ${FASTCODING_SOURCES}
    ${SSSE3_SOURCES}
    ${SSE4_SOURCES}
    ${AVX_SOURCES}
    ${AVX2_SOURCES}
    ${AVX2_X64_SOURCES}
    ${NEON_SOURCES}
    ${WIN32_SOURCES}
)

target_include_directories(kakadu PUBLIC ${PUBLIC_HEADERS} PRIVATE ${FBC_HEADERS})

# include the platform specific kakadu ht library
if(UNIX AND(NOT EMSCRIPTEN))
    target_link_libraries(kakadu PUBLIC
        ${KAKADU_ROOT}${KAKDU_HT_LIB}/${KAKADU_PLATFORM}/libkdu_ht.a m
    )
elseif(WIN32)
    target_link_libraries(kakadu PUBLIC
        ${KAKADU_ROOT}${KAKDU_HT_LIB}/${KAKADU_PLATFORM}/kdu_ht2019R.lib
    )
endif()

# KakaduAppSupport library
add_library(kakaduappsupport
    ${APP_SUPPORT_SOURCES}
    ${APP_SUPPORT_SOURCES_INTEL_SIMD}
    ${NEON_APP_SUPPORT}
)
target_include_directories(kakaduappsupport PUBLIC ${PUBLIC_HEADERS_APP_SUPPORT})
target_link_libraries(kakaduappsupport PUBLIC kakadu)

# turn off specific compiler warnings depending upon the compiler
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(kakadu PRIVATE -Wno-return-type -Wno-volatile -Wno-deprecated-declarations)
    target_compile_options(kakaduappsupport PRIVATE -Wno-return-type -Wno-volatile -Wno-deprecated-declarations)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    target_compile_options(kakadu PRIVATE -Wno-deprecated-volatile -Wno-return-type)
    target_compile_options(kakaduappsupport PRIVATE -Wno-deprecated-volatile -Wno-return-type)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(kakadu PRIVATE "/wd4244" "/wd4715")
    target_compile_options(kakaduappsupport PRIVATE "/wd4244" "/wd4715")
elseif(EMSCRIPTEN)
    target_compile_options(kakadu PRIVATE -Wno-deprecated-volatile -Wno-return-type -Wno-tautological-constant-out-of-range-compare -Wno-implicit-const-int-float-conversion)
    target_compile_options(kakaduappsupport PRIVATE -Wno-deprecated-volatile -Wno-return-type -Wno-tautological-constant-out-of-range-compare -Wno-implicit-const-int-float-conversion)
endif()
