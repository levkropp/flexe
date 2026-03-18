cmake_minimum_required(VERSION 3.14)
project(xtensa-emulator C)
set(CMAKE_C_STANDARD 17)

# Default to Release build (enables -O3)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Profile-guided optimization (PGO)
# Usage:
#   1. cmake -B build-pgo -DPGO=generate && cmake --build build-pgo
#   2. ./build-pgo/xtensa-emu -q -c 100000000 -s firmware.elf firmware.bin
#   3. cmake -B build-pgo -DPGO=use && cmake --build build-pgo
option(PGO "PGO mode: 'generate' to collect profiles, 'use' to apply them" "")
set(PREDECODE_FLASH_MB "2" CACHE STRING "Pre-decode table flash coverage in MB (0=disabled, 2/4/8/16)")

# LTO and native tuning for Release builds (GCC/Clang only)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT IPO_SUPPORTED)
    if(IPO_SUPPORTED AND NOT PGO)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
    if(NOT MSVC)
        add_compile_options(-march=native)
    endif()
endif()

if(PGO STREQUAL "generate")
    message(STATUS "PGO: generating profile data")
    add_compile_options(-fprofile-generate)
    add_link_options(-fprofile-generate)
    set(PGO_NOWARN "-Wno-error")
    # Disable LTO during profile generation (conflicts with PGO on some GCC versions)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION FALSE)
elseif(PGO STREQUAL "use")
    message(STATUS "PGO: using profile data for optimization")
    add_compile_options(-fprofile-use -fprofile-correction)
    add_link_options(-fprofile-use)
    set(PGO_NOWARN "-Wno-error")
    # Re-enable LTO for PGO use builds
    include(CheckIPOSupported)
    check_ipo_supported(RESULT IPO_SUPPORTED)
    if(IPO_SUPPORTED)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
endif()

# Library (core emulator)
set(LIB_SOURCES
    src/xtensa.c
    src/xtensa_disasm.c
    src/memory.c
    src/loader.c
    src/peripherals.c
    src/rom_stubs.c
    src/elf_symbols.c
    src/freertos_stubs.c
    src/esp_timer_stubs.c
    src/display_stubs.c
    src/touch_stubs.c
    src/sdcard_stubs.c
    src/wifi_stubs.c
    src/bt_stubs.c
    src/sha_stubs.c
    src/aes_stubs.c
    src/mpi_stubs.c
    src/flexe_session.c
    src/savestate.c
    src/hierarchical_trace.c
)

if(MSVC)
    list(APPEND LIB_SOURCES src/msvc_compat.c)
endif()

add_library(xtensa-emu-lib STATIC ${LIB_SOURCES})
target_include_directories(xtensa-emu-lib PUBLIC src/)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

if(MSVC)
    find_package(ZLIB REQUIRED)
    target_link_libraries(xtensa-emu-lib PUBLIC
        OpenSSL::SSL OpenSSL::Crypto ZLIB::ZLIB Threads::Threads)
    target_compile_options(xtensa-emu-lib PRIVATE /W4 /wd4100 /wd4244 /wd4267 /MP)
    target_compile_definitions(xtensa-emu-lib PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_definitions(xtensa-emu-lib PUBLIC PREDECODE_FLASH_MB=${PREDECODE_FLASH_MB})
else()
    target_link_libraries(xtensa-emu-lib PUBLIC
        m z Threads::Threads OpenSSL::SSL OpenSSL::Crypto)
    target_compile_options(xtensa-emu-lib PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})
    target_compile_definitions(xtensa-emu-lib PUBLIC
        PREDECODE_FLASH_MB=${PREDECODE_FLASH_MB})

    # Disable stack protector on hot-path files (saves ~3-4% from canary checks)
    set_source_files_properties(src/xtensa.c src/memory.c
        PROPERTIES COMPILE_OPTIONS "-fno-stack-protector")
endif()

# Main executable
add_executable(xtensa-emu src/main.c)
target_link_libraries(xtensa-emu PRIVATE xtensa-emu-lib)
if(MSVC)
    target_compile_options(xtensa-emu PRIVATE /W4 /wd4100 /wd4244 /wd4267)
    target_compile_definitions(xtensa-emu PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(xtensa-emu PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})
endif()

# Test runner
add_executable(xtensa-tests tests/test_main.c)
target_link_libraries(xtensa-tests PRIVATE xtensa-emu-lib)
if(MSVC)
    target_compile_options(xtensa-tests PRIVATE /W4 /wd4100 /wd4244 /wd4267)
    target_compile_definitions(xtensa-tests PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(xtensa-tests PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})
endif()

# Disassembler tool
add_executable(xt-dis tools/xt-dis.c)
target_link_libraries(xt-dis PRIVATE xtensa-emu-lib)
if(MSVC)
    target_compile_options(xt-dis PRIVATE /W4 /wd4100)
    target_compile_definitions(xt-dis PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(xt-dis PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})
endif()

# Trace filter tool
add_executable(trace-filter tools/trace-filter.c)
if(MSVC)
    target_compile_options(trace-filter PRIVATE /W4 /wd4100)
    target_compile_definitions(trace-filter PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(trace-filter PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})
endif()
