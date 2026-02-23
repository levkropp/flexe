cmake_minimum_required(VERSION 3.14)
project(xtensa-emulator C)
set(CMAKE_C_STANDARD 17)

# Library (core emulator)
add_library(xtensa-emu-lib STATIC
    src/xtensa.c
    src/xtensa_disasm.c
    src/memory.c
    src/loader.c
    src/peripherals.c
    src/rom_stubs.c
)
target_include_directories(xtensa-emu-lib PUBLIC src/)
target_link_libraries(xtensa-emu-lib PUBLIC m)
target_compile_options(xtensa-emu-lib PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)

# Main executable
add_executable(xtensa-emu src/main.c)
target_link_libraries(xtensa-emu PRIVATE xtensa-emu-lib)
target_compile_options(xtensa-emu PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)

# Test runner
add_executable(xtensa-tests tests/test_main.c)
target_link_libraries(xtensa-tests PRIVATE xtensa-emu-lib)
target_compile_options(xtensa-tests PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)

# Disassembler tool
add_executable(xt-dis tools/xt-dis.c)
target_link_libraries(xt-dis PRIVATE xtensa-emu-lib)
target_compile_options(xt-dis PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
