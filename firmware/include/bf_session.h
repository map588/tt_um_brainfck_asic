#pragma once

#include <stdbool.h>

/* Run one BF session on core 0: parse the program from USB, drive
 * the core-1 engine, pump input and output. In the byte stream,
 * Ctrl-D is end of input (',' reads 0) and Ctrl-C stops the session.
 * Returns NULL on success or an error token ("bad-program",
 * "run-fail", "stopped", "host-lost"). The caller must first arm the
 * BF pin profile and make sure the clock free-runs. */
const char *bf_session(bool debug);
