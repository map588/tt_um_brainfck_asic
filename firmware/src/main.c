/*
 * RP2350 host firmware for the Tiny Tapeout demo board v3.
 *
 * Core 0 runs a line-based command protocol over USB CDC (commands.c):
 * select a shuttle design, control or single-step the project clock
 * (clock.c), peek/poke the ui/uo/uio pins, and run interactive BF
 * sessions on tt_um_brainfck_asic (bf_run.c).
 * Core 1 (spi_ram.c) emulates the SPI RAM tape for the BF design.
 */
#include <stdio.h>

#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "board.h"
#include "clock.h"
#include "commands.h"
#include "spi_ram.h"

#define BOOT_CLK_HZ 1000000u
#define WAIT_FOR_USB 1

int main(void) {
    stdio_init_all();
    board_pins_init();

#if WAIT_FOR_USB
    while (!stdio_usb_connected())
        sleep_ms(100);
#endif

    asic_clk_set_hz(BOOT_CLK_HZ, NULL);
    multicore_launch_core1(spi_ram_core1_entry);

    printf("# tt-explorer host — type 'help'\n");
    command_loop();
}
