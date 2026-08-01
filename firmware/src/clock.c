#include "clock.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "tt_pins.h"

uint32_t clk_hz = 1000000u;
clk_mode_t clk_mode = CLK_STEP; /* main() calls asic_clk_set_hz() at boot */
uint32_t rx_settle_us;
uint32_t serial_half_us;

uint32_t asic_clks_us(uint32_t n) {
    return (n * 1000000u + clk_hz - 1) / clk_hz;
}

bool asic_clk_set_hz(uint32_t hz, uint32_t *actual) {
    if (hz < CLK_HZ_MIN || hz > CLK_HZ_MAX)
        return false;

    /* The PWM wrap counter is 16 bits. Frequencies below
     * clk_sys / 65536 (~2.3 kHz) need the clock divider. */
    uint32_t sys = clock_get_hz(clk_sys);
    uint32_t div = (uint32_t)(sys / ((uint64_t)hz * 65536u)) + 1u;
    if (div > 255u)
        return false; /* not reachable for hz >= CLK_HZ_MIN */
    uint32_t wrap = sys / (div * hz);

    uint slice = pwm_gpio_to_slice_num(TT_PIN_PROJ_CLK);
    pwm_set_enabled(slice, false);
    pwm_set_clkdiv_int_frac(slice, (uint8_t)div, 0);
    pwm_set_wrap(slice, (uint16_t)(wrap - 1));
    pwm_set_gpio_level(TT_PIN_PROJ_CLK, (uint16_t)(wrap / 2));
    pwm_set_counter(slice, 0);
    gpio_set_function(TT_PIN_PROJ_CLK, GPIO_FUNC_PWM);
    gpio_set_input_enabled(TT_PIN_PROJ_CLK, true); /* feed_instr and
                                                      asic_clk_stop read it */
    pwm_set_enabled(slice, true);

    clk_hz = hz;
    clk_mode = CLK_RUN;
    rx_settle_us = asic_clks_us(32u);
    serial_half_us = asic_clks_us(6u);
    if (actual)
        *actual = sys / (div * wrap);
    return true;
}

void asic_clk_stop(void) {
    if (clk_mode == CLK_STEP)
        return;
    uint slice = pwm_gpio_to_slice_num(TT_PIN_PROJ_CLK);

    /* Freeze the PWM only while the output is low. A freeze in the high
     * phase, or too near the falling edge, would make a runt pulse when
     * the pin then goes to GPIO-low. The interrupts-off window is only
     * the read-back plus one register write. */
    for (;;) {
        while (!gpio_get(TT_PIN_PROJ_CLK))
            tight_loop_contents();
        while (gpio_get(TT_PIN_PROJ_CLK)) /* falling edge */
            tight_loop_contents();
        uint32_t save = save_and_disable_interrupts();
        if (!gpio_get(TT_PIN_PROJ_CLK)) {
            pwm_set_enabled(slice, false);
            restore_interrupts(save);
            break;
        }
        restore_interrupts(save); /* an IRQ delayed us a full phase; retry */
    }

    gpio_put(TT_PIN_PROJ_CLK, 0);
    gpio_set_dir(TT_PIN_PROJ_CLK, GPIO_OUT);
    gpio_set_function(TT_PIN_PROJ_CLK, GPIO_FUNC_SIO);
    gpio_set_input_enabled(TT_PIN_PROJ_CLK, true);
    clk_mode = CLK_STEP;
}

bool asic_clk_step(uint32_t n) {
    if (clk_mode != CLK_STEP)
        return false;
    while (n--) {
        gpio_put(TT_PIN_PROJ_CLK, 1);
        busy_wait_us(10);
        gpio_put(TT_PIN_PROJ_CLK, 0);
        busy_wait_us(10);
    }
    return true;
}

void asic_clk_resume(void) {
    if (clk_mode == CLK_RUN)
        return;
    asic_clk_set_hz(clk_hz, NULL);
}
