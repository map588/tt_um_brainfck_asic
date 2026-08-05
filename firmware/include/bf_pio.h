/*
 * PIO machines for the timing-critical ASIC handshakes (pio1).
 *
 * Receive: samples TX_BIT on TX_CLK edges, so CPU timing cannot
 * slip a bit. Feed: pulses instr_valid across exactly one rising
 * clock edge, so a CPU stall cannot stretch or miss the pulse.
 * The engine polls the FIFOs; no interrupts are used. The MCU->ASIC
 * serial link stays bit-banged in bf_run.c: the MCU paces it with
 * phases of several ASIC clocks, so it has margin by construction.
 */
#pragma once

#include <stdint.h>

/* Reset both machines and enable them. Call at session start
 * (pins_bf). Safe to call again at any idle point. */
void bf_pio_arm(void);

/* Park instr_valid low and disable both machines. Call at session
 * end. */
void bf_pio_idle(void);

/* One received 10-bit frame, or -1 on timeout. The caller handles
 * the phantom repeat and the disarm wait. */
int bf_pio_recv_frame(uint32_t timeout_us);

/* Pulse instr_valid across one rising clock edge. The instruction
 * pins must already be set. Blocks until the pulse is done; a
 * wedged machine (stopped clock) is parked and reset silently, and
 * the caller's handshake timeout then reports the failure. */
void bf_pio_feed(void);
