cmake_minimum_required(VERSION 3.14)
project(xtensa-emulator C)
set(CMAKE_C_STANDARD 17)

# Default to Release build (enables -O3)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# LTO and native tuning for Release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT IPO_SUPPORTED)
    if(IPO_SUPPORTED)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
    if(NOT MSVC)
        add_compile_options(-march=native)
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
find_package(ZLIB REQUIRED)
find_package(pthreads REQUIRED)

if(MSVC)
    target_link_libraries(xtensa-emu-lib PUBLIC OpenSSL::SSL OpenSSL::Crypto ZLIB::ZLIB PThreads4W::PThreads4W)
    target_compile_options(xtensa-emu-lib PRIVATE /W4 /wd4100 /MP)
    target_compile_definitions(xtensa-emu-lib PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_link_libraries(xtensa-emu-lib PUBLIC m z pthread OpenSSL::SSL OpenSSL::Crypto)
    target_compile_options(xtensa-emu-lib PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
    # Disable stack protector on hot-path files (saves ~3-4% from canary checks per instruction)
    set_source_files_properties(src/xtensa.c src/memory.c
        PROPERTIES COMPILE_OPTIONS "-fno-stack-protector")
endif()

# Main executable
add_executable(xtensa-emu src/main.c)
target_link_libraries(xtensa-emu PRIVATE xtensa-emu-lib)
if(MSVC)
    target_compile_options(xtensa-emu PRIVATE /W4 /wd4100)
else()
    target_compile_options(xtensa-emu PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
endif()

# Test runner
add_executable(xtensa-tests tests/test_main.c)
target_link_libraries(xtensa-tests PRIVATE xtensa-emu-lib)
if(MSVC)
    target_compile_options(xtensa-tests PRIVATE /W4 /wd4100)
else()
    target_compile_options(xtensa-tests PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
endif()

# Disassembler tool
add_executable(xt-dis tools/xt-dis.c)
target_link_libraries(xt-dis PRIVATE xtensa-emu-lib)
if(MSVC)
    target_compile_options(xt-dis PRIVATE /W4 /wd4100)
else()
    target_compile_options(xt-dis PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
endif()

# Trace filter tool
add_executable(trace-filter tools/trace-filter.c)
if(MSVC)
    target_compile_options(trace-filter PRIVATE /W4 /wd4100)
else()
    target_compile_options(trace-filter PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
endif()
