cmake_minimum_required(VERSION 3.13)

# The terminal-explorer kit (firmware/kit submodule): board setup,
# SDK import, core firmware library, tt_extension().
include(kit/firmware/kit.cmake)

project(tt_um_brainfck_host C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# The kit's command firmware plus the BF extension.
tt_extension(tt_host
    src/bf_ext.c
    src/bf_run.c
    src/spi_ram.c
)
target_include_directories(tt_host PRIVATE include)
target_link_libraries(tt_host pico_multicore)
