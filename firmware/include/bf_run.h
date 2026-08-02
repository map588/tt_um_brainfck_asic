#pragma once

/* BF pin roles on top of the safe profile; enables the core-1 SPI
 * RAM emulator. */
void pins_bf(void);

/* Run one interactive BF session over USB CDC: read a program, run it
 * on the ASIC, bridge ','/'.'. Returns NULL on success or a protocol
 * error token ("bad-program", "run-fail"). The caller must first arm
 * the BF pin profile and make sure the clock free-runs. */
const char *bf_run_session(void);

/* Like bf_run_session(), but the host paces execution one instruction
 * at a time: 'n' steps, 'c' continues, 'q' stops. A "# dbg ..." state
 * line (pc, next op, pointer, data, bracket stack) follows each step. */
const char *bf_debug_session(void);
