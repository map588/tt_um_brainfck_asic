cmake_minimum_required(VERSION 3.13)

# The terminal-explorer kit (firmware/kit submodule) provides the
# board setup, the SDK import, and the core firmware library.
include(kit/firmware/preamble.cmake)

project(tt_um_brainfck_host C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

include(kit/firmware/core.cmake)

set(BF_SRCS
    src/bf_run.c
    src/spi_ram.c
)

# tt_host: the kit's command firmware plus the BF extension.
tt_extension(tt_host src/bf_ext.c ${BF_SRCS})
target_include_directories(tt_host PRIVATE include)
target_link_libraries(tt_host pico_multicore)

# bf_host: standalone BF paste-and-run firmware. It replaces the
# kit main, so it uses add_executable directly.
add_executable(bf_host src/bf_main.c ${BF_SRCS})
target_include_directories(bf_host PRIVATE include)
target_link_libraries(bf_host tt_core pico_multicore)
pico_enable_stdio_usb(bf_host 1)
pico_enable_stdio_uart(bf_host 0)
pico_add_extra_outputs(bf_host)
