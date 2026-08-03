/*
 * Mailbox between core 0 (USB host) and core 1 (BF engine).
 *
 * Channels, one owner per field:
 *  - start:   core 0 pushes one SIO FIFO word (BF_CMD_*) down.
 *  - input:   core 0 produces into the byte ring, the engine
 *             consumes for ',' and debugger keys.
 *  - control: core 0 sets stop_req / host_lost, the engine polls
 *             them every instruction and in every blocking wait.
 *  - events:  the engine pushes SIO FIFO words (BF_EV_*) up. Multi-
 *             word payloads sit in this struct, written before the
 *             event word; __dmb() orders the writes on both sides.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware/sync.h"

#define BF_MAX_OPS 1024u /* ASIC PC is 10 bits */

/* The instruction encoding, as the ASIC's 3-bit opcodes. */
enum { OP_SUB, OP_ADD, OP_LEFT, OP_RIGHT, OP_OPEN, OP_CLOSE, OP_IN, OP_OUT };

#define CH_STOP 0x03 /* Ctrl-C: stop the session */
#define CH_EOF 0x04  /* Ctrl-D: end of input, ',' reads 0 */

/* core 0 -> core 1 start words */
#define BF_CMD_RUN 1u
#define BF_CMD_DEBUG 2u

/* core 1 -> core 0 event words: tag in bits 31..24, arg in 23..0 */
#define BF_EV_WORD(t, a) (((uint32_t)(t) << 24) | ((uint32_t)(a) & 0xFFFFFFu))
#define BF_EV_TAG(w) ((w) >> 24)
#define BF_EV_ARG(w) ((w) & 0xFFFFFFu)

enum {
    BF_EV_OUT = 1,   /* arg = one output byte from '.' */
    BF_EV_DBG,       /* dbg snapshot valid */
    BF_EV_DBG_READY, /* debugger banner */
    BF_EV_BREAK,     /* arg = pc of the breakpoint hit */
    BF_EV_BP_SET,    /* arg = breakpoint index */
    BF_EV_BP_CLEAR,  /* arg = breakpoint index */
    BF_EV_BP_ERR,    /* bp index missing or out of range */
    BF_EV_DESYNC,    /* desync snapshot valid */
    BF_EV_DIED,      /* die snapshot valid */
    BF_EV_STOPPED,   /* run ended by CH_STOP */
    BF_EV_DBG_QUIT,  /* debugger ended by 'q' or CH_STOP */
    BF_EV_HALTED,    /* program ran to the end, executed valid */
    BF_EV_END,       /* session over, arg = bf_result_t */
};

typedef enum {
    BF_RES_OK = 0,
    BF_RES_RUN_FAIL,
    BF_RES_STOPPED,
    BF_RES_HOST_LOST,
} bf_result_t;

#define BF_INQ_CAP 1024u

typedef struct {
    /* core 0 -> core 1 control */
    bool stop_req;
    bool host_lost;

    /* core 0 -> core 1 input ring (single producer, single consumer) */
    uint16_t in_w, in_r;
    uint8_t in[BF_INQ_CAP];

    /* core 1 -> core 0 payloads */
    struct {
        const char *why; /* string literal, valid on both cores */
        uint16_t pc;
        char op;
        uint8_t inspect[4]; /* data, ptr, pc, bstack */
        uint8_t irq_jump, irq_io, tx_clk, spi_cs;
    } died;
    struct {
        uint16_t pc, vptr;
        char op;
        uint8_t data, bstk;
        uint32_t exec;
    } dbg;
    struct {
        uint16_t pc, expect;
        int32_t tgt;
    } desync;
    uint32_t executed;
} bf_shared_t;

extern volatile bf_shared_t bf_shared;

/* The program, written by core 0 before the start word, read by the
 * engine. */
extern uint8_t bf_ops[BF_MAX_OPS];
extern uint16_t bf_match[BF_MAX_OPS];
extern uint16_t bf_n_ops;

/* Input ring. Core 0 puts, the engine gets. A full ring drops the
 * byte: more than 1 KiB of unread type-ahead means the program does
 * not read input, and dropping keeps the CH_STOP scan alive. */
static inline bool bf_in_put(uint8_t b) {
    uint16_t w = bf_shared.in_w;
    uint16_t n = (uint16_t)((w + 1) % BF_INQ_CAP);
    if (n == bf_shared.in_r)
        return false;
    bf_shared.in[w] = b;
    __dmb();
    bf_shared.in_w = n;
    return true;
}

static inline int bf_in_get(void) {
    uint16_t r = bf_shared.in_r;
    if (r == bf_shared.in_w)
        return -1;
    uint8_t b = bf_shared.in[r];
    __dmb();
    bf_shared.in_r = (uint16_t)((r + 1) % BF_INQ_CAP);
    return b;
}
