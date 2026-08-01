/*
 * Standalone BF host (target: bf_host).
 *
 * Boots straight into the paste-a-program loop: open the serial
 * port, paste BF source, end with '!'. No command protocol — for
 * that flash tt_host instead.
 *
 * The clock is fixed at 200 kHz: measured on ttsky25b silicon
 * (2026-08-01), the ASIC->MCU serial link bit-slips at >= 500 kHz
 * and is fully reliable at <= 200 kHz.
 */
#include <stdio.h>

#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "bf_run.h"
#include "board.h"
#include "clock.h"
#include "spi_ram.h"

#define BF_CLK_HZ 200000u
#define WAIT_FOR_USB 1

int main(void) {
    stdio_init_all();
    board_pins_init();

#if WAIT_FOR_USB
    while (!stdio_usb_connected())
        sleep_ms(100);
#endif

    printf("\n== tt_um_brainfck_asic host ==\n");
    printf("ASIC clock %u Hz, design #%u\n", BF_CLK_HZ, BF_DESIGN_ADDR);

    asic_clk_set_hz(BF_CLK_HZ, NULL);
    tt_select_design(BF_DESIGN_ADDR);
    pins_bf();
    multicore_launch_core1(spi_ram_core1_entry);

    for (;;) {
        const char *err = bf_run_session();
        if (err)
            printf("!! %s\n", err);
    }
}
