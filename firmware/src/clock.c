#include "clock.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "tt_pins.h"

uint32_t clk_hz = 1000000u;
clk_mode_t clk_mode = CLK_STEP; /* main() calls asic_clk_set_hz() at boot */
uint32_t rx_settle_us;
uint32_t serial_half_us;

/*
 * PIO square-wave generator, hand-encoded (6 instructions, so no
 * pioasm step). One side-set bit drives TT_PIN_PROJ_CLK. Two
 * programs share the state machine:
 *
 * counter (offsets 0..3), period = 2*(y+2) cycles, y is 32 bits:
 *   0: mov x, y     side 0     ; low phase:  y+2 cycles
 *   1: jmp x--, 1   side 0
 *   2: mov x, y     side 1     ; high phase: y+2 cycles
 *   3: jmp x--, 3   side 1
 *
 * fast (offsets 4..5), period = N cycles for N = 2..32, exact,
 * via instruction delay slots:
 *   4: nop [hi-1]   side 1
 *   5: nop [lo-1]   side 0
 *
 * Frequency = clk_sys / period. The period rounds UP so the true
 * frequency never exceeds the request.
 */

#define CLK_PIO pio0
#define CLK_SM 0u

#define OFF_COUNTER 0u
#define OFF_FAST 4u
#define FAST_MAX_PERIOD 32u

static bool pio_ready;

static inline uint16_t side(uint v) {
    return (uint16_t)pio_encode_sideset(1, v);
}

static void clk_pio_init(void) {
    if (pio_ready)
        return;
    pio_sm_claim(CLK_PIO, CLK_SM);

    CLK_PIO->instr_mem[OFF_COUNTER + 0] =
        pio_encode_mov(pio_x, pio_y) | side(0);
    CLK_PIO->instr_mem[OFF_COUNTER + 1] =
        pio_encode_jmp_x_dec(OFF_COUNTER + 1) | side(0);
    CLK_PIO->instr_mem[OFF_COUNTER + 2] =
        pio_encode_mov(pio_x, pio_y) | side(1);
    CLK_PIO->instr_mem[OFF_COUNTER + 3] =
        pio_encode_jmp_x_dec(OFF_COUNTER + 3) | side(1);
    /* fast program slots are rewritten per frequency */

    pio_gpio_init(CLK_PIO, TT_PIN_PROJ_CLK);
    gpio_set_input_enabled(TT_PIN_PROJ_CLK, true); /* feed_instr and
                                                      asic_clk_stop
                                                      read it back */
    pio_sm_set_consecutive_pindirs(CLK_PIO, CLK_SM, TT_PIN_PROJ_CLK, 1,
                                   true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, TT_PIN_PROJ_CLK);
    sm_config_set_set_pins(&c, TT_PIN_PROJ_CLK, 1); /* for step mode */
    pio_sm_init(CLK_PIO, CLK_SM, OFF_COUNTER, &c);
    pio_ready = true;
}

uint32_t asic_clks_us(uint32_t n) {
    return (n * 1000000u + clk_hz - 1) / clk_hz;
}

bool asic_clk_set_hz(uint32_t hz, uint32_t *actual) {
    uint32_t sys = clock_get_hz(clk_sys);
    if (hz < CLK_HZ_MIN || hz > sys / 2)
        return false;
    clk_pio_init();

    /* Period in sys-clock cycles, rounded up (never overshoot). */
    uint64_t period = ((uint64_t)sys + hz - 1) / hz;

    pio_sm_set_enabled(CLK_PIO, CLK_SM, false);
    pio_sm_clear_fifos(CLK_PIO, CLK_SM);
    pio_sm_restart(CLK_PIO, CLK_SM);

    uint entry;
    if (period <= FAST_MAX_PERIOD) {
        uint hi = (uint)period / 2, lo = (uint)period - hi;
        CLK_PIO->instr_mem[OFF_FAST + 0] =
            pio_encode_nop() | side(1) | pio_encode_delay(hi - 1);
        CLK_PIO->instr_mem[OFF_FAST + 1] =
            pio_encode_nop() | side(0) | pio_encode_delay(lo - 1);
        pio_sm_set_wrap(CLK_PIO, CLK_SM, OFF_FAST, OFF_FAST + 1);
        entry = OFF_FAST + 1; /* start with the low phase */
    } else {
        /* period = 2*(y+2); round the half-period up */
        uint64_t half = (period + 1) / 2;
        if (half - 2 > 0xFFFFFFFFull)
            half = 0xFFFFFFFFull + 2ull; /* not reachable for hz >= 1 */
        uint32_t y = (uint32_t)(half - 2);
        period = 2 * half;
        pio_sm_put(CLK_PIO, CLK_SM, y);
        pio_sm_exec(CLK_PIO, CLK_SM, pio_encode_pull(false, true) | side(0));
        pio_sm_exec(CLK_PIO, CLK_SM,
                    pio_encode_out(pio_y, 32) | side(0));
        pio_sm_set_wrap(CLK_PIO, CLK_SM, OFF_COUNTER, OFF_COUNTER + 3);
        entry = OFF_COUNTER; /* starts with the low phase */
    }
    pio_sm_exec(CLK_PIO, CLK_SM, pio_encode_jmp(entry) | side(0));
    pio_sm_set_enabled(CLK_PIO, CLK_SM, true);

    clk_hz = hz;
    clk_mode = CLK_RUN;
    rx_settle_us = asic_clks_us(32u);
    serial_half_us = asic_clks_us(6u);
    if (actual) /* true output frequency, nearest integer */
        *actual = (uint32_t)(((uint64_t)sys + period / 2) / period);
    return true;
}

void asic_clk_stop(void) {
    if (clk_mode == CLK_STEP)
        return;

    /* Freeze the state machine only while the output is low. A
     * freeze in the high phase, or too near the falling edge, would
     * make a runt pulse when the pin is then parked low. The
     * interrupts-off window is only the read-back plus one register
     * write. */
    for (;;) {
        while (!gpio_get(TT_PIN_PROJ_CLK))
            tight_loop_contents();
        while (gpio_get(TT_PIN_PROJ_CLK)) /* falling edge */
            tight_loop_contents();
        uint32_t save = save_and_disable_interrupts();
        if (!gpio_get(TT_PIN_PROJ_CLK)) {
            pio_sm_set_enabled(CLK_PIO, CLK_SM, false);
            restore_interrupts(save);
            break;
        }
        restore_interrupts(save); /* an IRQ delayed us a full phase; retry */
    }

    /* Park low. The exec runs even while the SM is disabled. */
    pio_sm_exec(CLK_PIO, CLK_SM, pio_encode_set(pio_pins, 0) | side(0));
    clk_mode = CLK_STEP;
}

bool asic_clk_step(uint32_t n) {
    if (clk_mode != CLK_STEP)
        return false;
    while (n--) {
        pio_sm_exec(CLK_PIO, CLK_SM, pio_encode_set(pio_pins, 1) | side(1));
        busy_wait_us(10);
        pio_sm_exec(CLK_PIO, CLK_SM, pio_encode_set(pio_pins, 0) | side(0));
        busy_wait_us(10);
    }
    return true;
}

void asic_clk_resume(void) {
    if (clk_mode == CLK_RUN)
        return;
    asic_clk_set_hz(clk_hz, NULL);
}
