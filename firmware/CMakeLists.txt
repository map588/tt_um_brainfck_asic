cmake_minimum_required(VERSION 3.13)

# Tiny Tapeout demo board v3 carries an RP2350B; see boards/tt_dbv3.h.
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
set(PICO_BOARD tt_dbv3 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")

include(pico_sdk_import.cmake)

project(tt_host_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

set(SHARED_SRCS
    src/bf_run.c
    src/board.c
    src/clock.c
    src/spi_ram.c
)

# bf_host: standalone BF paste-and-run firmware.
add_executable(bf_host src/bf_main.c ${SHARED_SRCS})

# tt_host: command-protocol firmware for the explorer TUI.
add_executable(tt_host src/main.c src/commands.c ${SHARED_SRCS})

foreach(target bf_host tt_host)
    target_include_directories(${target} PRIVATE include)
    target_link_libraries(${target}
        pico_stdlib
        pico_multicore
        hardware_pwm
    )
    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)
    pico_add_extra_outputs(${target})
endforeach()
