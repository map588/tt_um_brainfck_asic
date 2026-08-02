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

# bf_host: standalone BF paste-and-run firmware (own main, no
# command protocol).
add_executable(bf_host src/bf_main.c ${BF_SRCS})

# tt_host: the kit's command firmware plus the BF extension.
add_executable(tt_host ${TT_CORE_MAIN} src/bf_ext.c ${BF_SRCS})

foreach(target bf_host tt_host)
    target_include_directories(${target} PRIVATE include)
    target_link_libraries(${target}
        tt_core
        pico_multicore
    )
    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)
    pico_add_extra_outputs(${target})
endforeach()
