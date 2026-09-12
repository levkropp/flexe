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
# Interpreter sampling profiler. Off by default: the runtime flag check costs
# ~3% in the dispatch loop even when the profiler is disabled.
option(FLEXE_PROFILE "Build the interpreter sampling profiler" OFF)
if(FLEXE_PROFILE)
    add_compile_definitions(FLEXE_PROFILE_BUILD=1)
endif()

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
    src/target.c
    src/xtensa.c
    src/window_accel.c
    src/profile.c
    src/xtensa_disasm.c
    src/memory.c
    src/flash_mmu.c
    src/esp32s3_extmem.c
    src/efuse.c
    src/gpio.c
    src/io_mux.c
    src/rtc_cntl.c
    src/regi2c.c
    src/sens.c
    src/sensitive_memprot.c
    src/system_clock.c
    src/systimer.c
    src/timer_group.c
    src/usb_serial_jtag.c
    src/spi_mem.c
    src/firmware_scan.c
    src/loader.c
    src/rom_elf.c
    src/peripherals.c
    src/spi_display.c
    src/sx127x.c
    src/axp192.c
    src/ublox_gps.c
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
    set_source_files_properties(src/xtensa.c src/window_accel.c src/memory.c
        src/jit.c
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

# Optional POSIX integration runners. Firmware images and Arduino CLI stay
# outside the core build; these targets provide their host-side harnesses.
if(UNIX)
    function(add_flexe_runner target source)
        add_executable(${target} tools/${source})
        target_link_libraries(${target} PRIVATE xtensa-emu-lib)
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Werror -Wno-unused-parameter ${PGO_NOWARN})
    endfunction()

    add_flexe_runner(flexe-stock-rom-test stock_rom_test.c)
    add_flexe_runner(flexe-generic-rom-test generic_rom_test.c)
    add_flexe_runner(flexe-bench-compute bench_compute.c)

    add_flexe_runner(flexe-analog-dac-test analog_dac_test.c)
    add_flexe_runner(flexe-deep-sleep-test deep_sleep_test.c)
    add_flexe_runner(flexe-emac-hal-test emac_hal_test.c)
    add_flexe_runner(flexe-esp-timer-test esp_timer_test.c)
    add_flexe_runner(flexe-event-group-test event_group_test.c)
    add_flexe_runner(flexe-frc-timer-test frc_timer_test.c)
    add_flexe_runner(flexe-frt-timer-test frt_timer_test.c)
    add_flexe_runner(flexe-gpio-isr-test gpio_isr_test.c)
    add_flexe_runner(flexe-guest-clock-test guest_clock_test.c)
    add_flexe_runner(flexe-i2c-slave-test i2c_slave_test.c)
    add_flexe_runner(flexe-i2c-wire-test i2c_wire_test.c)
    add_flexe_runner(flexe-i2s-dma-test i2s_dma_test.c)
    add_flexe_runner(flexe-ledc-pwm-test ledc_pwm_test.c)
    add_flexe_runner(flexe-mcpwm-motor-test mcpwm_motor_test.c)
    add_flexe_runner(flexe-pcnt-pulse-test pcnt_pulse_test.c)
    add_flexe_runner(flexe-rmt-tx-test rmt_tx_test.c)
    add_flexe_runner(flexe-rtc-gpio-test rtc_gpio_test.c)
    add_flexe_runner(flexe-rtc-i2c-test rtc_i2c_test.c)
    add_flexe_runner(flexe-scheduler-test scheduler_test.c)
    add_flexe_runner(flexe-sdio-slave-test sdio_slave_test.c)
    add_flexe_runner(flexe-sdmmc-host-test sdmmc_host_test.c)
    add_flexe_runner(flexe-sigmadelta-test sigmadelta_test.c)
    add_flexe_runner(flexe-spi-master-test spi_master_test.c)
    add_flexe_runner(flexe-task-queue-test task_queue_test.c)
    add_flexe_runner(flexe-task-wdt-test task_wdt_test.c)
    add_flexe_runner(flexe-timer-group-test timer_group_test.c)
    add_flexe_runner(flexe-touch-pad-test touch_pad_test.c)
    add_flexe_runner(flexe-twai-bus-test twai_bus_test.c)
    add_flexe_runner(flexe-uhci-dma-test uhci_dma_test.c)
    add_flexe_runner(flexe-wifi-client-test wifi_client_test.c)
endif()
