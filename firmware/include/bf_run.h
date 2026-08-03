#pragma once

/* Core 1 entry point: the BF execution engine. It waits for a start
 * word from bf_session.c, runs the session, and reports events
 * through the bf_link.h mailbox. */
void bf_core1_main(void);

/* BF pin roles on top of the safe profile. */
void pins_bf(void);

/* Recompute the serial-handshake delays from the current ASIC clock.
 * Call it whenever the clock changes (the ext_clock_changed hook). */
void bf_timing_update(void);
