/*
 * Core-0 side of a BF session: parse the program from USB, start the
 * core-1 engine (bf_run.c), then pump — USB bytes to the input ring
 * and control flags, engine events to text. All printf lives here,
 * so the wire protocol is defined in this one file.
 *
 * Session output ('.' bytes) goes out raw, without the CRLF
 * translation that protocol lines get: the ASIC is a byte machine
 * and a 0x0A cell must not become 0x0D 0x0A.
 */
#include <stdio.h>

#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "bf_link.h"
#include "bf_session.h"

/* Read one byte from the host, or -1 when the host disconnects.
 * A plain getchar() blocks forever after the port closes, and the
 * stuck session then eats the next connection's commands as
 * session input. */
static int host_getchar(void) {
    for (;;) {
        int c = getchar_timeout_us(100000);
        if (c != PICO_ERROR_TIMEOUT)
            return c;
        if (!stdio_usb_connected())
            return -1;
    }
}

/* Read BF source from USB CDC until '!' (or Ctrl-D).  Everything that
 * isn't one of the eight ops is a comment.  Anything after the '!' stays
 * in the stream and becomes ',' input for the program. */
static int op_of(int c) {
    switch (c) {
    case '-': return OP_SUB;
    case '+': return OP_ADD;
    case '<': return OP_LEFT;
    case '>': return OP_RIGHT;
    case '[': return OP_OPEN;
    case ']': return OP_CLOSE;
    case ',': return OP_IN;
    case '.': return OP_OUT;
    default: return -1;
    }
}

static bool read_program(void) {
    printf("# paste program, end with '!'\n");
    bf_n_ops = 0;
    bool overflow = false;
    for (;;) {
        int c = host_getchar();
        if (c < 0)
            return false; /* host gone, abort the session */
        if (c == CH_STOP)
            return false; /* host stop during the paste */
        if (c == '!' || c == CH_EOF)
            break;
        int op = op_of(c);
        if (op < 0)
            continue;
        if (bf_n_ops == BF_MAX_OPS) {
            overflow = true; /* keep draining until the terminator */
            continue;
        }
        bf_ops[bf_n_ops++] = (uint8_t)op;
        putchar(c); /* echo accepted ops so a paste is visible */
    }
    putchar('\n');
    if (overflow) {
        printf("# program exceeds %u ops (ASIC PC is 10 bits) — discarded\n",
               BF_MAX_OPS);
        return false;
    }
    if (bf_n_ops == 0)
        return false;
    printf("# %u ops\n", bf_n_ops);
    return true;
}

static bool build_jump_table(void) {
    uint16_t stack[BF_MAX_OPS];
    uint sp = 0, max_depth = 0;
    for (uint16_t i = 0; i < bf_n_ops; i++) {
        if (bf_ops[i] == OP_OPEN) {
            stack[sp++] = i;
            if (sp > max_depth)
                max_depth = sp;
        } else if (bf_ops[i] == OP_CLOSE) {
            if (sp == 0) {
                printf("# unmatched ']' at op %u\n", i);
                return false;
            }
            uint16_t j = stack[--sp];
            bf_match[j] = i;
            bf_match[i] = j;
        }
    }
    if (sp != 0) {
        printf("# unmatched '[' at op %u\n", stack[sp - 1]);
        return false;
    }
    /* The silicon bracket stack has 8 slots but its push guard stops
     * at 7, so 7 entries are usable. */
    if (max_depth > 7)
        printf("# nesting depth %u exceeds the silicon's 7 usable "
               "bracket-stack slots — deeper loops will misbehave\n",
               max_depth);
    return true;
}

static void print_died(void) {
    static const char *sel_name[4] = {"data", "ptr", "pc", "bstack"};
    printf("\n# !! %s at pc=%u ('%c')\n", bf_shared.died.why,
           bf_shared.died.pc, bf_shared.died.op);
    for (uint sel = 0; sel < 4; sel++)
        printf("#    inspect %-6s = 0x%02x\n", sel_name[sel],
               bf_shared.died.inspect[sel]);
    printf("#    irq_jump=%d irq_io=%d tx_clk=%d spi_cs=%d\n",
           bf_shared.died.irq_jump, bf_shared.died.irq_io,
           bf_shared.died.tx_clk, bf_shared.died.spi_cs);
}

static void print_dbg(void) {
    printf("# dbg pc=%u op=%c vptr=%u data=%02x bstk=%02x exec=%lu\n",
           bf_shared.dbg.pc, bf_shared.dbg.op, bf_shared.dbg.vptr,
           bf_shared.dbg.data, bf_shared.dbg.bstk,
           (unsigned long)bf_shared.dbg.exec);
    stdio_flush();
}

static const char *token_of(uint32_t res) {
    switch (res) {
    case BF_RES_OK: return NULL;
    case BF_RES_STOPPED: return "stopped";
    case BF_RES_HOST_LOST: return "host-lost";
    default: return "run-fail";
    }
}

/* Move USB bytes to the engine and engine events to text until the
 * engine reports the end of the session. */
static const char *pump(void) {
    for (;;) {
        while (multicore_fifo_rvalid()) {
            uint32_t w = multicore_fifo_pop_blocking();
            __dmb();
            uint32_t arg = BF_EV_ARG(w);
            switch (BF_EV_TAG(w)) {
            case BF_EV_OUT:
                putchar_raw((int)(arg & 0xFF));
                stdio_flush();
                break;
            case BF_EV_DBG:
                print_dbg();
                break;
            case BF_EV_DBG_READY:
                printf("# dbg ready: n=step c=continue q=quit "
                       "b<n>=breakpoint\n");
                break;
            case BF_EV_BREAK:
                printf("# dbg break at %lu\n", (unsigned long)arg);
                break;
            case BF_EV_BP_SET:
                printf("# dbg bp set %lu\n", (unsigned long)arg);
                break;
            case BF_EV_BP_CLEAR:
                printf("# dbg bp clear %lu\n", (unsigned long)arg);
                break;
            case BF_EV_BP_ERR:
                printf("# dbg bp out of range\n");
                break;
            case BF_EV_DESYNC:
                printf("# !! ']' at %u jumped to %ld, expected %u "
                       "(bracket stack desync?)\n",
                       bf_shared.desync.pc, (long)bf_shared.desync.tgt,
                       bf_shared.desync.expect);
                break;
            case BF_EV_DIED:
                print_died();
                break;
            case BF_EV_STOPPED:
                printf("\n# stopped by host\n");
                break;
            case BF_EV_DBG_QUIT:
                printf("# dbg stopped by host\n");
                break;
            case BF_EV_IN_WAIT:
                /* Own line: '.' output may sit mid-line before it. */
                printf("\n# input?\n");
                break;
            case BF_EV_HALTED:
                printf("\n# halted: %lu instructions executed\n",
                       (unsigned long)bf_shared.executed);
                break;
            case BF_EV_END:
                stdio_flush();
                return token_of(arg);
            }
            stdio_flush();
        }
        int c = getchar_timeout_us(1000);
        if (c == CH_STOP)
            bf_shared.stop_req = true;
        else if (c >= 0)
            bf_in_put((uint8_t)c); /* full ring drops: see bf_link.h */
        if (!stdio_usb_connected())
            bf_shared.host_lost = true;
    }
}

const char *bf_session(bool debug) {
    if (!read_program())
        return "bad-program";
    if (!build_jump_table())
        return "bad-program";

    bf_shared.stop_req = false;
    bf_shared.host_lost = false;
    bf_shared.in_r = bf_shared.in_w = 0;
    multicore_fifo_drain();
    __dmb();
    multicore_fifo_push_blocking(debug ? BF_CMD_DEBUG : BF_CMD_RUN);
    return pump();
}
