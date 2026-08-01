/*
 * Core 1: bit-banged SPI RAM slave ("the tape").
 *
 * The ASIC's spi_master expects a 23LC1024-style device: CS low, command
 * byte (0x02 write / 0x03 read), 16-bit address, then a 5-byte sequential
 * burst, all MSB first.  SCK runs at ASIC_CLK/8, the master drives MOSI at
 * the SCK rising edge and samples MISO at the falling edge, and SCK
 * free-runs between transactions, so CS is the only framing signal.
 *
 * Quirk compensated for here: spi_master.v never forces SCK low when CS
 * asserts.  If SCK happens to be high at the CS falling edge, the master's
 * bit counter consumes one slot before MOSI is first driven, so the command
 * byte arrives as only 7 bits (its MSB is never put on the wire).  Both
 * commands have bit7 = 0, so when SCK is high at CS-fall we collect 7
 * command bits and imply the MSB.  A real SPI RAM chip cannot apply this
 * fix — see firmware/README.md for the suggested one-line RTL change.
 *
 * Timing: at the default 1 MHz ASIC clock, SCK is 125 kHz (4 us per half
 * period) while this loop polls at ~10 ns granularity, so pure GPIO
 * polling is comfortable.  Core 1 runs nothing else and takes no
 * interrupts.
 */
#include <string.h>

#include "pico/stdlib.h"
#include "spi_ram.h"
#include "tt_pins.h"

#define CMD_WRITE 0x02u
#define CMD_READ 0x03u

/* The ASIC addresses a 1024-byte tape but puts 16-bit addresses on the
 * wire; a full 64 KiB backing store makes address handling unconditional. */
static uint8_t tape[65536];

static inline bool cs_high(void) { return gpio_get(PIN_SPI_CS); }

/* False parks core 1 in its outer wait so it never drives MISO while a
 * non-BF design owns the uio pins. The flag is only examined between
 * transactions; the CS pull-up keeps the wait condition true when the
 * BF design is not routed to the pads. */
static volatile bool enabled;

void spi_ram_set_enabled(bool on) { enabled = on; }

/* Called from core 0 while the ASIC is held in reset (CS deasserted, so
 * core 1 is parked in its CS-high wait and never touches the array). */
void spi_ram_clear(void) { memset(tape, 0, sizeof(tape)); }

/* Shift in n bits, sampling MOSI just after each SCK rising edge.
 * Returns false (transaction over) if CS deasserts while waiting. */
static bool __not_in_flash_func(rx_bits)(uint n, uint32_t *out) {
    uint32_t v = 0;
    while (n--) {
        while (gpio_get(PIN_SPI_SCK)) { /* wait for low phase */
            if (cs_high()) return false;
        }
        while (!gpio_get(PIN_SPI_SCK)) { /* wait for rising edge */
            if (cs_high()) return false;
        }
        v = (v << 1) | gpio_get(PIN_SPI_MOSI);
    }
    *out = v;
    return true;
}

/* Shift out one byte, updating MISO just after each SCK falling edge;
 * the master latches MISO at the *next* falling edge. */
static bool __not_in_flash_func(tx_byte)(uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        while (gpio_get(PIN_SPI_SCK)) { /* wait for falling edge */
            if (cs_high()) return false;
        }
        gpio_put(PIN_SPI_MISO, (b >> i) & 1);
        while (!gpio_get(PIN_SPI_SCK)) { /* hold through rising edge */
            if (cs_high()) return false;
        }
    }
    return true;
}

void __not_in_flash_func(spi_ram_core1_entry)(void) {
    for (;;) {
        while (!enabled || cs_high())
            tight_loop_contents();

        /* SCK level at CS-fall tells us whether the master will eat the
         * first bit slot (see file header).  CS-fall to first SCK edge is
         * 4 ASIC clocks, ample time for this read. */
        uint cmd_bits = gpio_get(PIN_SPI_SCK) ? 7 : 8;

        uint32_t cmd, addr_h, addr_l;
        if (!rx_bits(cmd_bits, &cmd) || !rx_bits(8, &addr_h) ||
            !rx_bits(8, &addr_l))
            continue;

        uint16_t addr = (uint16_t)((addr_h << 8) | addr_l);

        if (cmd == CMD_READ) {
            while (!cs_high()) {
                if (!tx_byte(tape[addr]))
                    break;
                addr++;
            }
            gpio_put(PIN_SPI_MISO, 0);
        } else if (cmd == CMD_WRITE) {
            uint32_t b;
            while (rx_bits(8, &b))
                tape[addr++] = (uint8_t)b;
        }

        while (!cs_high()) /* drain unknown commands / trailing clocks */
            tight_loop_contents();
    }
}
