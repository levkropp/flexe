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
set(PREDECODE_FLASH_MB "4" CACHE STRING "Pre-decode table flash coverage in MB (0=disabled, 2/4/8/16)")
# -march=native bakes the building machine's CPU into the binary, which is
# what you want locally and not what you want on a shared CI runner: the
# result is unreproducible, and a runner whose feature set the compiler
# dislikes fails the build outright under -Werror for reasons that have
# nothing to do with the code.
option(NATIVE_ARCH "Tune Release builds for the building machine's CPU" ON)

# LTO and native tuning for Release builds (GCC/Clang only)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT IPO_SUPPORTED)
    if(IPO_SUPPORTED AND NOT PGO)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
    if(NOT MSVC AND NATIVE_ARCH)
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
    src/spi_display.c
    src/rom_stubs.c
    src/elf_symbols.c
    src/freertos_stubs.c
    src/esp_timer_stubs.c
    src/display_stubs.c
    src/touch_stubs.c
    src/sdcard_stubs.c
    src/guest_call.c
    src/wifi_stubs.c
    src/vfs_stubs.c
    src/bt_stubs.c
    src/sha_stubs.c
    src/aes_stubs.c
    src/mpi_stubs.c
    src/flexe_session.c
    src/savestate.c
    src/hierarchical_trace.c
    src/sandbox_events.c
    src/aot.c
)

# JIT compiler: x86-64 and ARM64, non-MSVC
if(NOT MSVC AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|arm64|aarch64")
    list(APPEND LIB_SOURCES src/jit.c)
    message(STATUS "JIT compiler enabled (${CMAKE_SYSTEM_PROCESSOR})")
    set(FLEXE_HAS_JIT TRUE)
else()
    list(APPEND LIB_SOURCES src/jit_stub.c)
    message(STATUS "JIT compiler disabled (requires x86-64/ARM64, non-MSVC) — using no-op stubs")
endif()

if(MSVC)
    list(APPEND LIB_SOURCES src/msvc_compat.c)
endif()

if(FLEXE_HAS_JIT)
    add_compile_definitions(FLEXE_HAS_JIT=1)
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
    set_source_files_properties(src/xtensa.c src/memory.c src/jit.c
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
# AOT dlopen plugins call back into mem_read*/mem_write*; export them.
if(APPLE)
    target_link_options(xtensa-emu PRIVATE -Wl,-export_dynamic)
elseif(NOT MSVC)
    target_link_options(xtensa-emu PRIVATE -rdynamic)
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

# Optional POSIX external-ROM integration runner. The ROM images themselves
# are supplied at runtime and are not part of the normal unit-test fixtures.
if(UNIX)
    add_executable(flexe-stock-rom-test tools/stock_rom_test.c)
    target_link_libraries(flexe-stock-rom-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-stock-rom-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    # Optional compiled-firmware gates. Their .ino fixtures are built by the
    # shell runners so Arduino CLI remains outside the core build.
    add_executable(flexe-i2c-wire-test tools/i2c_wire_test.c)
    target_link_libraries(flexe-i2c-wire-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-i2c-wire-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-rtc-i2c-test tools/rtc_i2c_test.c)
    target_link_libraries(flexe-rtc-i2c-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-rtc-i2c-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-analog-dac-test tools/analog_dac_test.c)
    target_link_libraries(flexe-analog-dac-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-analog-dac-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-sigmadelta-test tools/sigmadelta_test.c)
    target_link_libraries(flexe-sigmadelta-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-sigmadelta-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-i2s-dma-test tools/i2s_dma_test.c)
    target_link_libraries(flexe-i2s-dma-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-i2s-dma-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-rmt-tx-test tools/rmt_tx_test.c)
    target_link_libraries(flexe-rmt-tx-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-rmt-tx-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-ledc-pwm-test tools/ledc_pwm_test.c)
    target_link_libraries(flexe-ledc-pwm-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-ledc-pwm-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-bench-compute tools/bench_compute.c)
    target_link_libraries(flexe-bench-compute PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-bench-compute PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-gpio-isr-test tools/gpio_isr_test.c)
    target_link_libraries(flexe-gpio-isr-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-gpio-isr-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-scheduler-test tools/scheduler_test.c)
    target_link_libraries(flexe-scheduler-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-scheduler-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-task-queue-test tools/task_queue_test.c)
    target_link_libraries(flexe-task-queue-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-task-queue-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-event-group-test tools/event_group_test.c)
    target_link_libraries(flexe-event-group-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-event-group-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-frt-timer-test tools/frt_timer_test.c)
    target_link_libraries(flexe-frt-timer-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-frt-timer-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-esp-timer-test tools/esp_timer_test.c)
    target_link_libraries(flexe-esp-timer-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-esp-timer-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-spi-master-test tools/spi_master_test.c)
    target_link_libraries(flexe-spi-master-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-spi-master-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-pcnt-pulse-test tools/pcnt_pulse_test.c)
    target_link_libraries(flexe-pcnt-pulse-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-pcnt-pulse-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-mcpwm-motor-test tools/mcpwm_motor_test.c)
    target_link_libraries(flexe-mcpwm-motor-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-mcpwm-motor-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-timer-group-test tools/timer_group_test.c)
    target_link_libraries(flexe-timer-group-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-timer-group-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-frc-timer-test tools/frc_timer_test.c)
    target_link_libraries(flexe-frc-timer-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-frc-timer-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-uhci-dma-test tools/uhci_dma_test.c)
    target_link_libraries(flexe-uhci-dma-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-uhci-dma-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-sdmmc-host-test tools/sdmmc_host_test.c)
    target_link_libraries(flexe-sdmmc-host-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-sdmmc-host-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-sdio-slave-test tools/sdio_slave_test.c)
    target_link_libraries(flexe-sdio-slave-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-sdio-slave-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-twai-bus-test tools/twai_bus_test.c)
    target_link_libraries(flexe-twai-bus-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-twai-bus-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})

    add_executable(flexe-emac-hal-test tools/emac_hal_test.c)
    target_link_libraries(flexe-emac-hal-test PRIVATE xtensa-emu-lib)
    target_compile_options(flexe-emac-hal-test PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})
endif()
