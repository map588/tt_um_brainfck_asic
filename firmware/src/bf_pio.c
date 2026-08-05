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
 *
 * Feed (instr_valid on one rising edge), one pulse per FIFO word:
 *
 *   12: pull block                ; wait for a go word
 *   13: wait 1 gpio PROJ_CLK
 *   14: wait 0 gpio PROJ_CLK      ; start from a falling edge
 *   15: set pins, 1               ; raise instr_valid
 *   16: wait 1 gpio PROJ_CLK      ; the ASIC executes here
 *   17: wait 0 gpio PROJ_CLK
 *   18: set pins, 0               ; drop it
 *   19: push block                ; completion token, wrap to 12
 *
 * The raise lands ~3 system clocks after the synchronized falling
 * edge and the drop mirrors it, so the pulse spans exactly one
 * rising edge with near half-period margin on both sides at any
 * reachable clock. The CPU version could jitter by an ASIC clock
 * under bus contention; this cannot.
 */
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "bf_pins.h"
#include "bf_pio.h"

#define BF_PIO pio1
#define SM_RECV 0u
#define SM_FEED 1u
#define OFF_RECV 0u
#define OFF_FEED 12u

/* A pulse is two clock edges. 200 ms covers it down to far below
 * the usable clock range; past that the clock is stopped. */
#define FEED_TIMEOUT_US 200000u

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

    pio_sm_claim(BF_PIO, SM_FEED);
    BF_PIO->instr_mem[OFF_FEED + 0] = pio_encode_pull(false, true);
    BF_PIO->instr_mem[OFF_FEED + 1] =
        pio_encode_wait_gpio(true, TT_PIN_PROJ_CLK - 16);
    BF_PIO->instr_mem[OFF_FEED + 2] =
        pio_encode_wait_gpio(false, TT_PIN_PROJ_CLK - 16);
    BF_PIO->instr_mem[OFF_FEED + 3] = pio_encode_set(pio_pins, 1);
    BF_PIO->instr_mem[OFF_FEED + 4] =
        pio_encode_wait_gpio(true, TT_PIN_PROJ_CLK - 16);
    BF_PIO->instr_mem[OFF_FEED + 5] =
        pio_encode_wait_gpio(false, TT_PIN_PROJ_CLK - 16);
    BF_PIO->instr_mem[OFF_FEED + 6] = pio_encode_set(pio_pins, 0);
    BF_PIO->instr_mem[OFF_FEED + 7] = pio_encode_push(false, true);

    c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, PIN_INSTR_VALID, 1);
    sm_config_set_wrap(&c, OFF_FEED, OFF_FEED + 7);
    pio_sm_init(BF_PIO, SM_FEED, OFF_FEED, &c);
    pio_sm_set_consecutive_pindirs(BF_PIO, SM_FEED, PIN_INSTR_VALID, 1,
                                   true);
    pio_sm_exec(BF_PIO, SM_FEED, pio_encode_set(pio_pins, 0));

    pio_ready = true;
}

/* Reset one machine to its entry with empty FIFOs and shift state,
 * then enable. A machine left mid-pass by a died session realigns
 * here. */
static void sm_reset(uint sm, uint entry) {
    pio_sm_set_enabled(BF_PIO, sm, false);
    pio_sm_clear_fifos(BF_PIO, sm);
    pio_sm_restart(BF_PIO, sm);
    if (sm == SM_FEED) /* restart keeps pin latches: force it low */
        pio_sm_exec(BF_PIO, sm, pio_encode_set(pio_pins, 0));
    pio_sm_exec(BF_PIO, sm, pio_encode_jmp(entry));
    pio_sm_set_enabled(BF_PIO, sm, true);
}

void bf_pio_arm(void) {
    bf_pio_init();
    /* pins_safe() gave instr_valid back to SIO; take it again. The
     * machine's pin latch is low, so the pad stays low across the
     * handover. */
    pio_gpio_init(BF_PIO, PIN_INSTR_VALID);
    sm_reset(SM_RECV, OFF_RECV);
    sm_reset(SM_FEED, OFF_FEED);
}

void bf_pio_idle(void) {
    if (!pio_ready)
        return;
    pio_sm_set_enabled(BF_PIO, SM_RECV, false);
    pio_sm_set_enabled(BF_PIO, SM_FEED, false);
    pio_sm_exec(BF_PIO, SM_FEED, pio_encode_set(pio_pins, 0));
}

int bf_pio_recv_frame(uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    while (pio_sm_is_rx_fifo_empty(BF_PIO, SM_RECV)) {
        if (time_reached(dl))
            return -1;
    }
    return (int)(pio_sm_get(BF_PIO, SM_RECV) & 0x3FF);
}

void bf_pio_feed(void) {
    pio_sm_put(BF_PIO, SM_FEED, 1);
    absolute_time_t dl = make_timeout_time_us(FEED_TIMEOUT_US);
    while (pio_sm_is_rx_fifo_empty(BF_PIO, SM_FEED)) {
        if (time_reached(dl)) {
            sm_reset(SM_FEED, OFF_FEED);
            return;
        }
    }
    (void)pio_sm_get(BF_PIO, SM_FEED); /* completion token */
}
