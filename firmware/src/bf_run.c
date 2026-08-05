/*
 * BF execution engine for tt_um_brainfck_asic. Runs on CORE 1.
 *
 * Core 1 takes no interrupts, so every bit-banged handshake below
 * runs with deterministic timing and no masking. The two edge-
 * critical handshakes run on PIO machines (bf_pio.c): the ASIC->MCU
 * serial link is sampled there, and the instr_valid pulse is
 * generated there, so neither depends on CPU timing at all. Core 0
 * owns USB and all text output; this file talks to it only through
 * the mailbox in bf_link.h (input ring in, SIO FIFO events out).
 * bf_session.c is the core-0 side.
 *
 * Handshake model
 * ---------------
 * The ASIC has no program memory: we hold the whole program and must feed
 * the instruction at the ASIC's PC.  inspect_sel is parked on PC so the
 * low PC byte (uo[7:4] | uio[7:4]) acts as an "instruction retired"
 * signal — it also covers the invisible SPI cache-refill stalls, which
 * raise no interrupt.  Bracket and I/O ops are then completed via
 * interrupt_jump / interrupt_io and the two 10-bit serial links:
 *
 *   '[' data==0: interrupt_jump, ASIC waits on RX -> send PC of matching ']'
 *   ']' data!=0: interrupt_jump, ASIC transmits jump target on TX
 *   '.'        : interrupt_io,   ASIC transmits {2'b00, data} on TX
 *   ','        : interrupt_io,   ASIC waits on RX -> send {2'b00, byte}
 *
 * instr_valid is sampled every ASIC clock edge, so a sloppy pulse executes
 * an instruction twice.  A PIO machine pulses it from falling edge to
 * falling edge — exactly one rising edge sees it high.
 */
#include <setjmp.h>
#include <string.h>

#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "bf_link.h"
#include "bf_pins.h"
#include "bf_pio.h"
#include "bf_run.h"
#include "board.h"
#include "clock.h"

#define PC_TIMEOUT_US 200000u
#define SERIAL_TIMEOUT_US 200000u

/* Clock edges to wait after an ASIC->MCU transfer (real frame plus
 * phantom). After the phantom's last bit, serial_tx still passes
 * DONE (done=1 for one cycle) and IDLE. A TX op fed into that
 * window samples the stale tx_done in WAIT_IO/WAIT_JUMP and
 * completes without ever transmitting. Counted in edges, not time:
 * a time delay aliases against the clock phase. Derived window is
 * 3 cycles; 6 gives margin. */
#define TX_DISARM_EDGES 6u

static const char OP_CHAR[8] = {'-', '+', '<', '>', '[', ']', ',', '.'};

volatile bf_shared_t bf_shared;
uint8_t bf_ops[BF_MAX_OPS];
uint16_t bf_match[BF_MAX_OPS];
uint16_t bf_n_ops;

static jmp_buf bf_err; /* die()/stop jump here; set per session */
enum { JMP_DIE = 1, JMP_STOP = 2, JMP_HOST_LOST = 3 };

/* Timings that follow the ASIC clock. Core 0 writes them from the
 * ext_clock_changed hook, always between sessions. */
static volatile uint32_t rx_settle_us;
static volatile uint32_t serial_half_us;

void bf_timing_update(void) {
    rx_settle_us = asic_clks_us(32u);
    serial_half_us = asic_clks_us(6u);
}

/* BF pin roles on top of the safe profile. The 8 ui pins are already
 * MCU outputs (instr, instr_valid, rx clk/bit, inspect_sel). MISO is
 * driven low so the ASIC's SPI input does not float. The PIO
 * receiver is armed fresh for the session. */
void pins_bf(void) {
    pins_safe();
    gpio_put(PIN_SPI_MISO, 0);
    gpio_set_dir(PIN_SPI_MISO, GPIO_OUT);
    bf_pio_arm();
}

/* ---- mailbox ---- */

/* Publish payload writes, then the event word. */
static void ev_push(uint8_t tag, uint32_t arg) {
    __dmb();
    multicore_fifo_push_blocking(BF_EV_WORD(tag, arg));
}

static void __attribute__((noreturn)) stop_run(void);
static void __attribute__((noreturn)) lost_host(void);

/* One input byte for ',' or a debugger key. Blocks until a byte
 * arrives, the host asks for a stop, or the host disconnects. */
static int in_wait(void) {
    for (;;) {
        if (bf_shared.stop_req)
            return -1;
        if (bf_shared.host_lost)
            return -2;
        int c = bf_in_get();
        if (c >= 0)
            return c;
        tight_loop_contents();
    }
}

/* ---- low-level pin access ---- */

static inline uint64_t gpio_snapshot(void) {
    return ((uint64_t)sio_hw->gpio_hi_in << 32) | sio_hw->gpio_in;
}

static inline uint8_t inspect_from(uint64_t snap) {
    uint8_t lo = (snap >> (TT_GPIO_UO_BASE + 4)) & 0xF;
    uint8_t hi = (snap >> (TT_GPIO_UIO_BASE + 4)) & 0xF;
    return (uint8_t)((hi << 4) | lo);
}

static void set_inspect_sel(uint sel) {
    gpio_put(PIN_SEL0, sel & 1);
    gpio_put(PIN_SEL1, (sel >> 1) & 1);
}

/* ---- instruction feed ---- */

/* Assert instr_valid across exactly one rising edge of the ASIC
 * clock. The pulse itself runs on a PIO machine (bf_pio.c); the
 * instruction pins are plain outputs and settle long before the
 * machine reaches the edge. */
static void feed_instr(uint8_t op) {
    gpio_put(PIN_INSTR0, op & 1);
    gpio_put(PIN_INSTR1, (op >> 1) & 1);
    gpio_put(PIN_INSTR2, (op >> 2) & 1);
    bf_pio_feed();
}

/* ---- completion / interrupt polling ---- */

typedef enum { EV_PC_ADVANCED, EV_IRQ, EV_TIMEOUT } event_t;

/* The pads switch on the ASIC's rising clock edge and our poll is
 * asynchronous, so a single read can catch a mid-transition value
 * (some pads switched, others not). Accept a match only when a
 * second read ~200 ns later agrees: pad-to-pad skew resolves well
 * inside that, so a transient cannot survive the confirmation. */
static bool __not_in_flash_func(inspect_is)(uint8_t expect) {
    if (inspect_from(gpio_snapshot()) != expect)
        return false;
    busy_wait_at_least_cycles(32);
    return inspect_from(gpio_snapshot()) == expect;
}

static event_t await_pc_or_irq(uint8_t expect_pc, uint irq_pin,
                               uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    for (;;) {
        if (gpio_get(irq_pin))
            return EV_IRQ;
        if (inspect_is(expect_pc))
            return EV_PC_ADVANCED;
        if (time_reached(dl))
            return EV_TIMEOUT;
    }
}

static bool wait_pc_low(uint8_t expect, uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    while (!inspect_is(expect)) {
        if (time_reached(dl))
            return false;
    }
    return true;
}

static bool wait_gpio_low(uint pin, uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    while (gpio_get(pin)) {
        if (time_reached(dl))
            return false;
    }
    return true;
}

/* ---- 10-bit serial links ---- */

/* MCU->ASIC: serial_rx samples RX_BIT on the (synchronized) rising edge of
 * RX_CLK; each phase is held for several ASIC clocks to clear the two-flop
 * synchronizer.  MSB first. The MCU paces this link itself, so it needs no
 * PIO: every phase has multi-clock margin by construction. */
static void __not_in_flash_func(send10)(uint16_t v) {
    for (int i = 9; i >= 0; i--) {
        gpio_put(PIN_RX_BIT, (v >> i) & 1);
        busy_wait_us(serial_half_us);
        gpio_put(PIN_RX_CLK, 1);
        busy_wait_us(serial_half_us);
        gpio_put(PIN_RX_CLK, 0);
        busy_wait_us(serial_half_us);
    }
}

/* ASIC->MCU: receive one frame and swallow its phantom repeat
 * (silicon bug 2: serial_tx re-arms on a stale start strobe, so
 * every frame repeats once). The PIO receive machine (bf_pio.c)
 * pushes each frame as one FIFO word. Returns the first frame's
 * value, or -1 on timeout. */
static int recv10(uint32_t timeout_us) {
    int v = bf_pio_recv_frame(timeout_us);
    if (v < 0)
        return -1;
    /* The phantom start comes within a few ASIC clocks and its
     * frame takes a few tens more; bound the wait so a genuinely
     * missing phantom fails fast. */
    if (bf_pio_recv_frame(asic_clks_us(192u)) < 0)
        return -1;
    /* The machine pushes the phantom word at its 10th rising edge,
     * before the frame's last low phase runs out. Wait for TX_CLK
     * to idle high again so the disarm count starts where the
     * bit-banged receiver started it: after the frame ended. */
    absolute_time_t dl = make_timeout_time_us(asic_clks_us(64u));
    while (!gpio_get(PIN_TX_CLK)) {
        if (time_reached(dl))
            break;
    }
    for (uint n = TX_DISARM_EDGES; n--;) {
        while (!gpio_get(TT_PIN_PROJ_CLK))
            tight_loop_contents();
        while (gpio_get(TT_PIN_PROJ_CLK))
            tight_loop_contents();
    }
    return v;
}

/* Read one inspect value with the mux settled. */
static uint8_t inspect_read(uint sel) {
    set_inspect_sel(sel);
    busy_wait_us(asic_clks_us(4u));
    return inspect_from(gpio_snapshot());
}

/* Fresh machine for each program. VERIFIED: pulse reset, then read
 * PC and DATA back, and pulse again until both are zero. An
 * unverified pulse intermittently leaves stale state behind. */
static bool asic_reset(void) {
    for (int tries = 0; tries < 8; tries++) {
        gpio_put(TT_PIN_PROJ_NRST, 0);
        busy_wait_us(asic_clks_us(64u));
        gpio_put(TT_PIN_PROJ_NRST, 1);
        busy_wait_us(asic_clks_us(64u));
        uint8_t pc = inspect_read(INSPECT_PC);
        uint8_t data = inspect_read(INSPECT_DATA);
        set_inspect_sel(INSPECT_PC);
        busy_wait_us(asic_clks_us(4u));
        if (pc == 0 && data == 0) {
            /* Settle before the first feed. A run started hard
             * against the verified reset showed rare first-op
             * misexecution (same footprint as the stale-reset
             * silicon bug); a few quiet clocks close it. */
            busy_wait_us(asic_clks_us(8u));
            return true;
        }
    }
    return false;
}

/* ---- error handling ---- */

/* Snapshot diagnostics for core 0, put the feed pins back to idle,
 * and jump out of the run so the engine loop stays alive. */
static void __attribute__((noreturn)) die(const char *why, uint16_t pc) {
    bf_shared.died.why = why;
    bf_shared.died.pc = pc;
    bf_shared.died.op = pc < bf_n_ops ? OP_CHAR[bf_ops[pc]] : '?';
    for (uint sel = 0; sel < 4; sel++) {
        set_inspect_sel(sel);
        busy_wait_us(asic_clks_us(4u));
        bf_shared.died.inspect[sel] = inspect_from(gpio_snapshot());
    }
    bf_shared.died.irq_jump = (uint8_t)gpio_get(PIN_IRQ_JUMP);
    bf_shared.died.irq_io = (uint8_t)gpio_get(PIN_IRQ_IO);
    bf_shared.died.tx_clk = (uint8_t)gpio_get(PIN_TX_CLK);
    bf_shared.died.spi_cs = (uint8_t)gpio_get(PIN_SPI_CS);
    /* instr_valid belongs to the PIO; bf_core1_main parks it via
     * bf_pio_idle() right after the session ends. */
    gpio_put(PIN_RX_CLK, 0);
    gpio_put(PIN_RX_BIT, 0);
    set_inspect_sel(INSPECT_PC);
    ev_push(BF_EV_DIED, 0);
    longjmp(bf_err, JMP_DIE);
}

static void __attribute__((noreturn)) leave_run(int code) {
    gpio_put(PIN_RX_CLK, 0);
    gpio_put(PIN_RX_BIT, 0);
    set_inspect_sel(INSPECT_PC);
    longjmp(bf_err, code);
}

static void __attribute__((noreturn)) stop_run(void) { leave_run(JMP_STOP); }
static void __attribute__((noreturn)) lost_host(void) {
    leave_run(JMP_HOST_LOST);
}

/* ---- execution ----
 *
 * Silicon bugs that shape this loop (confirmed by step-mode traces,
 * 2026-08-01; see README "Known issues"):
 *
 * 1. serial_tx re-arms on a stale start strobe: every ASIC->MCU frame
 *    is followed by an identical phantom frame. recv10() consumes it.
 *
 * 2. The SPI cache-refill FSM re-issues its transaction forever (the
 *    transfer_done exit is unreachable). The engine therefore never
 *    feeds a pointer move that would leave the ASIC's cache window
 *    (physical cells 0..8): it virtualizes such moves through the
 *    working '.'/',' serial paths and keeps the full 1024-cell tape
 *    in vtape. Synthetic ops advance the ASIC's PC, so the engine
 *    models it in `apc`, separate from the program index `pc`.
 *
 * 3. Physical cell 4 (tape_base) has no backing store in the RTL:
 *    the cache mux returns data_current for offset 0 and the save
 *    path drops the value when the pointer leaves. Reads alias the
 *    value carried in; writes are lost. The engine therefore never
 *    parks on physical cell 4 — logical cell L maps to physical L
 *    for L < 4 and L + 1 for L >= 4, and the logical 3<->4 move
 *    feeds two physical moves (transit through cell 4 is safe).
 */

#define NATIVE_LIMIT 7u /* logical cells 0..7 live in the ASIC */

static inline uint phys_of(uint16_t l) { return l < 4 ? l : l + 1u; }

static uint8_t vtape[1024];

/* Feed a synthetic '.' and return data_current (the current cell). */
static uint8_t synth_out(uint8_t apc_next, uint16_t pc) {
    feed_instr(OP_OUT);
    if (await_pc_or_irq(apc_next, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
        die("no interrupt_io for synthetic '.'", pc);
    int v = recv10(SERIAL_TIMEOUT_US);
    if (v < 0)
        die("no data for synthetic '.'", pc);
    if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
        die("interrupt_io stuck after synthetic '.'", pc);
    if (!wait_pc_low(apc_next, PC_TIMEOUT_US))
        die("synthetic '.' never retired", pc);
    return (uint8_t)(v & 0xFF);
}

/* Feed a synthetic ',' loading `byte` into data_current. */
static void synth_in(uint8_t byte, uint8_t apc_next, uint16_t pc) {
    feed_instr(OP_IN);
    if (await_pc_or_irq(apc_next, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
        die("no interrupt_io for synthetic ','", pc);
    busy_wait_us(rx_settle_us);
    send10(byte);
    if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
        die("interrupt_io stuck after synthetic ','", pc);
    if (!wait_pc_low(apc_next, PC_TIMEOUT_US))
        die("synthetic ',' never retired", pc);
}

typedef struct {
    uint16_t pc;         /* program index */
    uint16_t apc;        /* model of the ASIC's PC (drifts from pc) */
    uint16_t vptr;       /* logical tape pointer */
    uint16_t abstack[8]; /* ASIC PC values it pushed for '[' */
    uint sp;
    uint32_t executed;
} bf_state_t;

static void state_init(bf_state_t *s) {
    memset(s, 0, sizeof *s);
    memset(vtape, 0, sizeof vtape);
    set_inspect_sel(INSPECT_PC);
}

/* Execute the one instruction at s->pc. Calls die() (longjmp) on a
 * handshake failure. Inline so the run loop pays no call cost. */
static inline void exec_one(bf_state_t *s) {
    {
        uint8_t op = bf_ops[s->pc];
        uint8_t anext = (uint8_t)(s->apc + 1); /* ASIC PC low byte after op */

        switch (op) {
        case OP_SUB:
        case OP_ADD:
            feed_instr(op);
            s->executed++;
            if (!wait_pc_low(anext, PC_TIMEOUT_US))
                die("timeout waiting for op to retire", s->pc);
            s->apc++;
            s->pc++;
            break;

        case OP_LEFT:
        case OP_RIGHT: {
            uint16_t nv = s->vptr;
            if (op == OP_RIGHT) {
                if (s->vptr < 1023)
                    nv = s->vptr + 1;
            } else {
                if (s->vptr > 0)
                    nv = s->vptr - 1;
            }
            if (nv == s->vptr) { /* blocked at a tape edge: pure no-op */
                s->pc++;
                break;
            }
            if (s->vptr <= NATIVE_LIMIT && nv <= NATIVE_LIMIT) {
                /* Native move inside the cache window. Crossing
                 * logical 3<->4 takes two physical moves (transit
                 * through the broken physical cell 4). */
                uint pa = phys_of(s->vptr), pb = phys_of(nv);
                uint moves = pb > pa ? pb - pa : pa - pb;
                while (moves--) {
                    feed_instr(op);
                    s->executed++;
                    if (!wait_pc_low((uint8_t)(s->apc + 1), PC_TIMEOUT_US))
                        die("timeout waiting for op to retire", s->pc);
                    s->apc++;
                }
            } else {
                /* Virtual move: save the current cell, load the target
                 * cell. The ASIC's ptr stays parked at the window edge. */
                vtape[s->vptr] = synth_out(anext, s->pc);
                s->apc++;
                synth_in(vtape[nv], (uint8_t)(s->apc + 1), s->pc);
                s->apc++;
                s->executed += 2;
            }
            s->vptr = nv;
            s->pc++;
            break;
        }

        case OP_OPEN: {
            feed_instr(op);
            s->executed++;
            event_t ev = await_pc_or_irq(anext, PIN_IRQ_JUMP, PC_TIMEOUT_US);
            if (ev == EV_PC_ADVANCED) { /* data!=0: entered loop */
                if (s->sp < count_of(s->abstack))
                    s->abstack[s->sp] = s->apc;
                s->sp++;
                s->apc++;
                s->pc++;
            } else if (ev == EV_IRQ) { /* data==0: ASIC wants skip target */
                /* The ASIC pushed s->apc. Send s->apc back so its PC does not
                 * move, then run the matching ']' (data==0: pops). */
                busy_wait_us(rx_settle_us);
                send10(s->apc & 0x3FF);
                if (!wait_gpio_low(PIN_IRQ_JUMP, SERIAL_TIMEOUT_US))
                    die("interrupt_jump stuck after skip", s->pc);
                if (!wait_pc_low((uint8_t)s->apc, PC_TIMEOUT_US))
                    die("PC never reached skip target", s->pc);
                if (s->sp < count_of(s->abstack))
                    s->abstack[s->sp] = s->apc;
                s->sp++;
                s->pc = bf_match[s->pc];
            } else {
                die("no response to '['", s->pc);
            }
            break;
        }

        case OP_CLOSE: {
            feed_instr(op);
            s->executed++;
            event_t ev = await_pc_or_irq(anext, PIN_IRQ_JUMP, PC_TIMEOUT_US);
            if (ev == EV_PC_ADVANCED) { /* data==0: exited loop */
                if (s->sp)
                    s->sp--;
                s->apc++;
                s->pc++;
            } else if (ev == EV_IRQ) { /* data!=0: ASIC transmits target */
                int tgt = recv10(SERIAL_TIMEOUT_US);
                if (tgt < 0)
                    die("no jump target transmitted for ']'", s->pc);
                if (!wait_gpio_low(PIN_IRQ_JUMP, SERIAL_TIMEOUT_US))
                    die("interrupt_jump stuck after ']'", s->pc);
                if (s->sp && (uint16_t)tgt != (s->abstack[s->sp - 1] & 0x3FF)) {
                    bf_shared.desync.pc = s->pc;
                    bf_shared.desync.tgt = tgt;
                    bf_shared.desync.expect = s->abstack[s->sp - 1] & 0x3FF;
                    ev_push(BF_EV_DESYNC, 0);
                }
                uint16_t back;
                if (s->sp >= 2) {
                    /* Silicon: the WAIT_JUMP start branch fires twice
                     * before tx_busy rises, so the ASIC pops TWO
                     * entries and its PC ends at the second one. */
                    s->sp -= 2;
                    back = s->abstack[s->sp];
                    if (!wait_pc_low((uint8_t)back, PC_TIMEOUT_US))
                        die("PC never reached jump target", s->pc);
                    s->apc = back;
                    /* Restore the lost entry with a synthetic '[' —
                     * data is not zero here, so it pushes and advances. */
                    feed_instr(OP_OPEN);
                    s->executed++;
                    if (await_pc_or_irq((uint8_t)(s->apc + 1), PIN_IRQ_JUMP,
                                        PC_TIMEOUT_US) != EV_PC_ADVANCED)
                        die("synthetic '[' did not push", s->pc);
                    s->abstack[s->sp++] = s->apc;
                    s->apc++;
                } else {
                    back = s->sp ? s->abstack[--s->sp] : 0;
                    if (!wait_pc_low((uint8_t)back, PC_TIMEOUT_US))
                        die("PC never reached jump target", s->pc);
                    s->apc = back;
                }
                s->pc = bf_match[s->pc]; /* the matching '[' re-executes */
            } else {
                die("no response to ']'", s->pc);
            }
            break;
        }

        case OP_IN: {
            feed_instr(op);
            s->executed++;
            if (await_pc_or_irq(anext, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
                die("no interrupt_io for ','", s->pc);
            int c = in_wait();
            if (c == -1)
                stop_run();
            if (c == -2)
                lost_host();
            if (c == CH_EOF)
                c = 0; /* end of input: ',' reads 0 */
            busy_wait_us(rx_settle_us);
            send10((uint16_t)(c & 0xFF));
            if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
                die("interrupt_io stuck after ','", s->pc);
            if (!wait_pc_low(anext, PC_TIMEOUT_US))
                die("',' never retired", s->pc);
            s->apc++;
            s->pc++;
            break;
        }

        case OP_OUT: {
            feed_instr(op);
            s->executed++;
            if (await_pc_or_irq(anext, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
                die("no interrupt_io for '.'", s->pc);
            int v = recv10(SERIAL_TIMEOUT_US);
            if (v < 0)
                die("no data transmitted for '.'", s->pc);
            if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
                die("interrupt_io stuck after '.'", s->pc);
            if (!wait_pc_low(anext, PC_TIMEOUT_US))
                die("'.' never retired", s->pc);
            /* Emit after the op retires: a full FIFO then stalls the
             * engine between instructions, never mid-handshake. */
            ev_push(BF_EV_OUT, (uint32_t)(v & 0xFF));
            s->apc++;
            s->pc++;
            break;
        }
        }
    }
}

/* One check per instruction: stop and disconnect end the run at the
 * next instruction boundary. */
static void check_host(void) {
    if (bf_shared.stop_req)
        stop_run();
    if (bf_shared.host_lost)
        lost_host();
}

static bf_result_t engine_run(void) {
    gpio_put(TT_PIN_LED, 1);
    bf_state_t st;
    bf_result_t res;
    switch (setjmp(bf_err)) {
    case 0:
        if (!asic_reset())
            die("reset did not clear the design", 0);
        state_init(&st);
        while (st.pc < bf_n_ops) {
            check_host();
            exec_one(&st);
        }
        bf_shared.executed = st.executed;
        ev_push(BF_EV_HALTED, 0);
        res = BF_RES_OK;
        break;
    case JMP_STOP:
        ev_push(BF_EV_STOPPED, 0);
        res = BF_RES_STOPPED;
        break;
    case JMP_HOST_LOST:
        res = BF_RES_HOST_LOST;
        break;
    default:
        res = BF_RES_RUN_FAIL; /* die() already pushed BF_EV_DIED */
        break;
    }
    gpio_put(TT_PIN_LED, 0);
    return res;
}

/* ---- instruction-level debugger ---- */

/* Snapshot one state line per step. inspect_sel returns to PC
 * afterward: the retirement handshake depends on it. */
static void dbg_state(const bf_state_t *s) {
    bf_shared.dbg.data = inspect_read(INSPECT_DATA);
    bf_shared.dbg.bstk = inspect_read(INSPECT_BSTACK);
    set_inspect_sel(INSPECT_PC);
    busy_wait_us(asic_clks_us(4u));
    bf_shared.dbg.pc = s->pc;
    bf_shared.dbg.op = s->pc < bf_n_ops ? OP_CHAR[bf_ops[s->pc]] : '-';
    bf_shared.dbg.vptr = s->vptr;
    bf_shared.dbg.exec = s->executed;
    ev_push(BF_EV_DBG, 0);
}

/* Breakpoints on program indexes. Cleared at each session start;
 * the host re-sends its set. */
static uint8_t bp[BF_MAX_OPS / 8];

static inline bool bp_get(uint16_t i) {
    return (bp[i >> 3] >> (i & 7)) & 1;
}

/* 'b' followed by a decimal index toggles a breakpoint. The digits
 * end at the first non-digit byte (the host sends a newline). */
static void bp_command(void) {
    uint32_t v = 0;
    bool any = false;
    for (;;) {
        absolute_time_t dl = make_timeout_time_us(200000);
        int c;
        while ((c = bf_in_get()) < 0) {
            if (time_reached(dl) || bf_shared.stop_req ||
                bf_shared.host_lost)
                break;
        }
        if (c < '0' || c > '9')
            break;
        v = v * 10 + (uint32_t)(c - '0');
        any = true;
    }
    if (!any || v >= bf_n_ops) {
        ev_push(BF_EV_BP_ERR, 0);
        return;
    }
    bp[v >> 3] ^= (uint8_t)(1u << (v & 7));
    ev_push(bp_get((uint16_t)v) ? BF_EV_BP_SET : BF_EV_BP_CLEAR, v);
}

/* Like engine_run(), but the host paces execution: 'n' runs one
 * instruction, 'c' runs to the next breakpoint or the end, 'q' (or
 * CH_STOP at the prompt) ends the session. */
static bf_result_t engine_debug(void) {
    gpio_put(TT_PIN_LED, 1);
    bf_state_t st;
    int j = setjmp(bf_err);
    if (j) {
        gpio_put(TT_PIN_LED, 0);
        if (j == JMP_STOP) {
            ev_push(BF_EV_STOPPED, 0);
            return BF_RES_STOPPED;
        }
        return j == JMP_HOST_LOST ? BF_RES_HOST_LOST : BF_RES_RUN_FAIL;
    }
    if (!asic_reset())
        die("reset did not clear the design", 0);
    state_init(&st);
    memset(bp, 0, sizeof bp);
    ev_push(BF_EV_DBG_READY, 0);
    dbg_state(&st);
    while (st.pc < bf_n_ops) {
        int ch = in_wait();
        if (ch == -2) {
            gpio_put(TT_PIN_LED, 0);
            return BF_RES_HOST_LOST;
        }
        if (ch == 'n') {
            exec_one(&st);
            dbg_state(&st);
        } else if (ch == 'c') {
            /* at least one instruction, so a continue from a
             * breakpoint does not stop on the same spot */
            do {
                check_host();
                exec_one(&st);
            } while (st.pc < bf_n_ops && !bp_get(st.pc));
            if (st.pc < bf_n_ops) {
                ev_push(BF_EV_BREAK, st.pc);
                dbg_state(&st);
            }
        } else if (ch == 'b') {
            bp_command();
        } else if (ch == 'q' || ch == -1) {
            gpio_put(TT_PIN_LED, 0);
            ev_push(BF_EV_DBG_QUIT, 0);
            return BF_RES_OK;
        }
        /* other bytes (line ends and so on) are ignored */
    }
    bf_shared.executed = st.executed;
    ev_push(BF_EV_HALTED, 0);
    gpio_put(TT_PIN_LED, 0);
    return BF_RES_OK;
}

/* ---- core 1 entry ---- */

void bf_core1_main(void) {
    for (;;) {
        uint32_t cmd = multicore_fifo_pop_blocking();
        __dmb();
        /* Interrupts stay off for the whole session. The SDK arms
         * FIFO/lockout handlers on this core at launch, and one
         * stray interrupt mid-pulse stretches instr_valid across
         * two rising edges: the instruction executes twice. The
         * engine only polls (pins, ring, FIFO, timer), so it needs
         * no interrupts. */
        uint32_t save = save_and_disable_interrupts();
        bf_result_t res =
            cmd == BF_CMD_DEBUG ? engine_debug() : engine_run();
        bf_pio_idle();
        restore_interrupts(save);
        ev_push(BF_EV_END, (uint32_t)res);
    }
}
