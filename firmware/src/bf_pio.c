/*
 * PIO receiver for the ASIC->MCU serial link, hand-encoded like the
 * kit's clock generator (no pioasm step). It lives on pio1 with GPIO
 * base 16, so pins 16..47 are reachable (the kit clock owns pio0 at
 * base 0).
 *
 * Only the receive side runs on PIO. Receiving is the timing-
 * critical direction: the ASIC clocks the frame out at half the
 * project clock, and CPU-polled sampling starts to slip bits near
 * 500 kHz. The send side stays bit-banged in bf_run.c: the MCU
 * paces that link itself with phases of several ASIC clocks, so
 * CPU timing gives it margin by construction.
 *
 * The machine captures a transfer PAIR per pass, because the
 * phantom repeat (silicon bug 2) follows the real frame with no
 * idle-high gap: serial_tx goes DONE -> IDLE with the stale start
 * strobe still set, so the clock stays low straight into the
 * phantom. After the phantom the strobe is clear and the clock
 * does idle high. A machine that treats frames uniformly cannot
 * sit in that asymmetry: it either misses the gapless phantom
 * start or samples the return-to-idle rise as a bit, and every
 * later frame shifts.
 *
 *    0: wait 1 pin 1               ; idle between transfer pairs
 *    1: wait 0 pin 1               ; announce of the real frame
 *    2: set x, 9
 *    3: wait 1 pin 1               ; bit stable from rising edge
 *    4: in pins, 1
 *    5: wait 0 pin 1
 *    6: jmp x--, 3                 ; real frame pushed at bit 10
 *    7: set x, 9                   ; phantom: gapless, no idle wait
 *    8: wait 1 pin 1
 *    9: in pins, 1
 *   10: wait 0 pin 1
 *   11: jmp x--, 8                 ; phantom pushed, wrap to 0
 *
 * The wrap to the idle wait consumes the return-to-idle rise
 * without sampling it. The machine runs at full system clock;
 * sampling lands ~3 system clocks after each edge, inside the
 * stable window at any ASIC clock.
 */
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "bf_pins.h"
#include "bf_pio.h"

#define BF_PIO pio1
#define SM_RECV 0u
#define OFF_RECV 0u

static bool pio_ready;

static void bf_pio_init(void) {
    if (pio_ready)
        return;
    pio_set_gpio_base(BF_PIO, 16);
    pio_sm_claim(BF_PIO, SM_RECV);

    BF_PIO->instr_mem[OFF_RECV + 0] = pio_encode_wait_pin(true, 1);
    BF_PIO->instr_mem[OFF_RECV + 1] = pio_encode_wait_pin(false, 1);
    BF_PIO->instr_mem[OFF_RECV + 2] = pio_encode_set(pio_x, 9);
    BF_PIO->instr_mem[OFF_RECV + 3] = pio_encode_wait_pin(true, 1);
    BF_PIO->instr_mem[OFF_RECV + 4] = pio_encode_in(pio_pins, 1);
    BF_PIO->instr_mem[OFF_RECV + 5] = pio_encode_wait_pin(false, 1);
    BF_PIO->instr_mem[OFF_RECV + 6] = pio_encode_jmp_x_dec(OFF_RECV + 3);
    BF_PIO->instr_mem[OFF_RECV + 7] = pio_encode_set(pio_x, 9);
    BF_PIO->instr_mem[OFF_RECV + 8] = pio_encode_wait_pin(true, 1);
    BF_PIO->instr_mem[OFF_RECV + 9] = pio_encode_in(pio_pins, 1);
    BF_PIO->instr_mem[OFF_RECV + 10] = pio_encode_wait_pin(false, 1);
    BF_PIO->instr_mem[OFF_RECV + 11] = pio_encode_jmp_x_dec(OFF_RECV + 8);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, PIN_TX_BIT); /* wait pin 1 = TX_CLK */
    sm_config_set_in_shift(&c, false, true, 10); /* MSB first, autopush */
    sm_config_set_wrap(&c, OFF_RECV, OFF_RECV + 11);
    pio_sm_init(BF_PIO, SM_RECV, OFF_RECV, &c);

    pio_ready = true;
}

/* Reset the machine to its entry with empty FIFO and shift state,
 * then enable. Call at session start; a machine left mid-frame by
 * a died session realigns here. */
void bf_pio_arm(void) {
    bf_pio_init();
    pio_sm_set_enabled(BF_PIO, SM_RECV, false);
    pio_sm_clear_fifos(BF_PIO, SM_RECV);
    pio_sm_restart(BF_PIO, SM_RECV);
    pio_sm_exec(BF_PIO, SM_RECV, pio_encode_jmp(OFF_RECV));
    pio_sm_set_enabled(BF_PIO, SM_RECV, true);
}

void bf_pio_idle(void) {
    if (!pio_ready)
        return;
    pio_sm_set_enabled(BF_PIO, SM_RECV, false);
}

int bf_pio_recv_frame(uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    while (pio_sm_is_rx_fifo_empty(BF_PIO, SM_RECV)) {
        if (time_reached(dl))
            return -1;
    }
    return (int)(pio_sm_get(BF_PIO, SM_RECV) & 0x3FF);
}
