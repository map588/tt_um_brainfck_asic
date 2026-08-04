/*
 * PIO receiver for the ASIC->MCU serial link (pio1). The machine
 * samples TX_BIT on TX_CLK edges, so CPU timing cannot slip a bit.
 * The engine polls the FIFO; no interrupts are used. The MCU->ASIC
 * direction stays bit-banged in bf_run.c: the MCU paces it with
 * phases of several ASIC clocks, so it has margin by construction.
 */
#pragma once

#include <stdint.h>

/* Reset the receive machine and enable it. Call at session start
 * (pins_bf). Safe to call again at any idle point. */
void bf_pio_arm(void);

/* Disable the receive machine. Call at session end. */
void bf_pio_idle(void);

/* One received 10-bit frame, or -1 on timeout. The caller handles
 * the phantom repeat and the disarm wait. */
int bf_pio_recv_frame(uint32_t timeout_us);
