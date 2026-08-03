#pragma once

/* BF pin roles on top of the safe profile; enables the core-1 SPI
 * RAM emulator. */
void pins_bf(void);

/* Recompute the serial-handshake delays from the current ASIC clock.
 * Call it whenever the clock changes (the ext_clock_changed hook). */
void bf_timing_update(void);

/* Run one interactive BF session over USB CDC: read a program, run it
 * on the ASIC, bridge ','/'.'. In the byte stream, Ctrl-D is end of
 * input (',' reads 0) and Ctrl-C stops the session. Returns NULL on
 * success or an error token ("bad-program", "run-fail", "stopped").
 * The caller must first arm the BF pin profile and make sure the
 * clock free-runs. */
const char *bf_run_session(void);

/* Like bf_run_session(), but the host paces execution one instruction
 * at a time: 'n' steps, 'c' continues, 'q' stops. A "# dbg ..." state
 * line (pc, next op, pointer, data, bracket stack) follows each step. */
const char *bf_debug_session(void);
