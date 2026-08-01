#include "pico/stdlib.h"

#include "board.h"
#include "spi_ram.h"
#include "tt_pins.h"

void board_pins_init(void) {
    static const uint pins[] = {TT_PIN_CTRL_ENA, TT_PIN_CTRL_NRST,
                                TT_PIN_CTRL_INC, TT_PIN_PROJ_NRST,
                                TT_PIN_LED};
    for (uint i = 0; i < count_of(pins); i++) {
        gpio_init(pins[i]);
        gpio_put(pins[i], 0);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
    pins_safe();
}

/* The CS pull-up stays on in every profile: it is weak, it cannot
 * fight a driven pad, and it parks core 1 when the BF design is not
 * routed (see spi_ram.c). */
void pins_safe(void) {
    spi_ram_set_enabled(false);
    for (uint i = 0; i < 8; i++) {
        uint p = TT_GPIO_UIO_BASE + i;
        gpio_init(p); /* SIO function, input */
        gpio_disable_pulls(p);
    }
    gpio_pull_up(PIN_SPI_CS);
    for (uint i = 0; i < 8; i++) {
        uint p = TT_GPIO_UI_BASE + i;
        gpio_init(p);
        gpio_put(p, 0);
        gpio_set_dir(p, GPIO_OUT);
    }
    for (uint i = 0; i < 8; i++)
        gpio_init(TT_GPIO_UO_BASE + i);
}

/* The 8 ui pins are already MCU outputs (instr, instr_valid, rx
 * clk/bit, inspect_sel) and only MISO changes on the uio side. */
void pins_bf(void) {
    pins_safe();
    gpio_put(PIN_SPI_MISO, 0);
    gpio_set_dir(PIN_SPI_MISO, GPIO_OUT);
    spi_ram_set_enabled(true);
}

/* Mux sequence per tt-micropython-firmware project_mux.py. Harmless on
 * the FPGA ASIC-sim carrier, required on real silicon. */
void tt_select_design(unsigned n) {
    gpio_put(TT_PIN_CTRL_INC, 0);
    gpio_put(TT_PIN_CTRL_NRST, 0);
    gpio_put(TT_PIN_CTRL_ENA, 0);
    sleep_ms(10);
    gpio_put(TT_PIN_CTRL_NRST, 1);
    sleep_ms(10);
    for (uint i = 0; i < n; i++) {
        gpio_put(TT_PIN_CTRL_INC, 1);
        sleep_ms(1);
        gpio_put(TT_PIN_CTRL_INC, 0);
        sleep_ms(1);
    }
    gpio_put(TT_PIN_CTRL_ENA, 1);
    sleep_ms(1);
}
