/*
 * BF extension of the terminal-explorer kit firmware.
 *
 * The whole BF integration is this file: it defines the kit's
 * extension hooks (kit/firmware/include/ext.h) to add the `bf` and
 * `bfdbg` commands, launch the execution engine on core 1, follow
 * clock and design changes, and report the BF design address in
 * hello/status. bf_run.c is the core-1 engine, bf_session.c the
 * core-0 session pump. The kit core is untouched.
 */
#include <stdio.h>
#include <string.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "bf_pins.h"
#include "bf_run.h"
#include "bf_session.h"
#include "clock.h"
#include "ext.h"

#define BF_MIN_HZ 50000u   /* below this, feed_instr's interrupts-off \
                              window starves USB and the 200 ms       \
                              handshake timeouts trip */
#define BF_MAX_HZ 2000000u /* hard cap: the CPU cannot pace the        \
                              instr_valid pulse at all above this.     \
                              The receive link is PIO-sampled and no   \
                              longer limits; the CPU-paced feed is     \
                              solid at 200 kHz and starts to fail      \
                              around 500 kHz (measured 2026-08-04) */

static int bf_design = -1; /* tracked via ext_design_changed */
static bool bf_armed;

static const char *bf_precheck(void) {
    if (bf_design != (int)BF_DESIGN_ADDR)
        return "not-bf-design";
    if (clk_mode != CLK_RUN)
        return "need-clock";
    if (clk_hz < BF_MIN_HZ)
        return "too-slow";
    if (clk_hz > BF_MAX_HZ)
        return "too-fast";
    return NULL;
}

static const char *bf_common(const char *banner, bool debug) {
    const char *err = bf_precheck();
    if (err)
        return err;
    pins_bf();
    bf_armed = true;
    printf("ok %s\n", banner);
    err = bf_session(debug);
    /* Drain input the program did not consume (e.g. ',' bytes left
     * over after an error) so it cannot pollute the next command. */
    while (getchar_timeout_us(100000) != PICO_ERROR_TIMEOUT)
        ;
    if (err)
        return err;
    strcpy(tt_reply, "done");
    return NULL;
}

static const char *cmd_bf(int argc, char **argv) {
    (void)argc;
    (void)argv;
    return bf_common("bf", false);
}

static const char *cmd_bfdbg(int argc, char **argv) {
    (void)argc;
    (void)argv;
    return bf_common("bfdbg", true);
}

static const struct cmd bf_cmds[] = {
    {"bf", cmd_bf, "bf                 run a BF session (BF design only)"},
    {"bfdbg", cmd_bfdbg, "bfdbg              BF debugger: n=step c=run q=quit"},
};

/* ---- kit hooks ---- */

const struct cmd *ext_commands(size_t *count) {
    *count = count_of(bf_cmds);
    return bf_cmds;
}

void ext_init(void) {
    multicore_launch_core1(bf_core1_main);
}

void ext_hello(char *out, size_t cap) {
    snprintf(out, cap, "bf=%u", BF_DESIGN_ADDR);
}

void ext_status(char *out, size_t cap) {
    snprintf(out, cap, "bf=%d", bf_armed ? 1 : 0);
}

void ext_design_changed(unsigned addr) {
    bf_design = (int)addr;
    bf_armed = false; /* the safe profile disarmed the BF pins */
}

void ext_clock_changed(uint32_t hz) {
    (void)hz;
    bf_timing_update();
}
