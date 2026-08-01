/*
 * Runtime ASIC clock control (PIO square-wave generator).
 *
 * Two modes:
 *   CLK_RUN  — a PIO state machine makes a free-running clock on
 *              TT_PIN_PROJ_CLK, exact to one sys-clock cycle.
 *   CLK_STEP — the state machine is off, the pin is parked low, and
 *              asic_clk_step() makes single clean pulses.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CLK_HZ_MIN 1u /* the PIO cycle counter is 32 bits */
/* Upper limit is clk_sys/2 (~75 MHz), checked at run time. The BF
 * engine has its own, much lower limits (see commands.c). */
#define CLK_STEP_MAX 65535u

typedef enum { CLK_RUN, CLK_STEP } clk_mode_t;

/* Read-only outside clock.c. */
extern uint32_t clk_hz;    /* last programmed frequency */
extern clk_mode_t clk_mode;

/* Timings that follow the clock; asic_clk_set_hz() recomputes them so the
 * hot serial loops in bf_run.c read plain variables. */
extern uint32_t rx_settle_us;
extern uint32_t serial_half_us;

/* Microseconds that n ASIC clocks take at the current frequency. */
uint32_t asic_clks_us(uint32_t n);

/* Program the PWM and enter CLK_RUN. Returns false if hz is out of
 * range. Writes the true output frequency to *actual when non-NULL. */
bool asic_clk_set_hz(uint32_t hz, uint32_t *actual);

/* Park the clock low and enter CLK_STEP. Safe against runt pulses. */
void asic_clk_stop(void);

/* Make n clock pulses. Returns false unless in CLK_STEP. */
bool asic_clk_step(uint32_t n);

/* Return to CLK_RUN at the last programmed frequency. */
void asic_clk_resume(void);
