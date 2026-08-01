#pragma once

/* Run one interactive BF session over USB CDC: read a program, run it
 * on the ASIC, bridge ','/'.'. Returns NULL on success or a protocol
 * error token ("bad-program", "run-fail"). The caller must first arm
 * the BF pin profile and make sure the clock free-runs. */
const char *bf_run_session(void);
