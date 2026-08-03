/*
 * BF execution engine for tt_um_brainfck_asic.
 *
 * Drives the ASIC clock handshake, feeds instructions, mirrors the
 * program counter, resolves bracket jumps against a precomputed match
 * table, and services ','/'.' over USB CDC.
 * Core 1 (spi_ram.c) emulates the SPI RAM tape.
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
 * an instruction twice.  The clock module generates the ASIC clock (PWM)
 * and we pulse instr_valid from falling edge to falling edge — exactly one
 * rising edge sees it high.
 */
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "bf_pins.h"
#include "bf_run.h"
#include "board.h"
#include "clock.h"
#include "spi_ram.h"

#define MAX_OPS 1024u /* ASIC PC is 10 bits */
#define PC_TIMEOUT_US 200000u
#define SERIAL_TIMEOUT_US 200000u

#define CH_STOP 0x03 /* Ctrl-C: stop the session */
#define CH_EOF 0x04  /* Ctrl-D: end of input, ',' reads 0 */

enum { OP_SUB, OP_ADD, OP_LEFT, OP_RIGHT, OP_OPEN, OP_CLOSE, OP_IN, OP_OUT };
static const char OP_CHAR[8] = {'-', '+', '<', '>', '[', ']', ',', '.'};

static uint8_t ops[MAX_OPS];
static uint16_t match[MAX_OPS];
static uint16_t n_ops;

static jmp_buf bf_err; /* die() jumps here; set in bf_run_session() */

/* Timings that follow the ASIC clock. The kit calls the hook on
 * every clock change, so the hot serial loops read plain variables. */
static uint32_t rx_settle_us;
static uint32_t serial_half_us;

/* One pushed-back byte from the stop poll: it is pending ',' input
 * and must not be lost. */
static int pushback = -1;

/* Read one byte from the host, or -1 when the host disconnects.
 * A plain getchar() blocks forever after the port closes, and the
 * stuck session then eats the next connection's commands as
 * session input. */
static int host_getchar(void) {
    if (pushback >= 0) {
        int c = pushback;
        pushback = -1;
        return c;
    }
    for (;;) {
        int c = getchar_timeout_us(100000);
        if (c != PICO_ERROR_TIMEOUT)
            return c;
        if (!stdio_usb_connected())
            return -1;
    }
}

/* Non-blocking check for CH_STOP between instructions. Any other
 * byte is pending ',' input and goes into the pushback. */
static bool host_wants_stop(void) {
    if (pushback >= 0)
        return false;
    int c = getchar_timeout_us(0);
    if (c == CH_STOP)
        return true;
    if (c >= 0)
        pushback = c;
    return false;
}

void bf_timing_update(void) {
    rx_settle_us = asic_clks_us(32u);
    serial_half_us = asic_clks_us(6u);
}

/* BF pin roles on top of the safe profile; enables the core-1 SPI
 * RAM. The 8 ui pins are already MCU outputs (instr, instr_valid,
 * rx clk/bit, inspect_sel) and only MISO changes on the uio side. */
void pins_bf(void) {
    pins_safe();
    gpio_put(PIN_SPI_MISO, 0);
    gpio_set_dir(PIN_SPI_MISO, GPIO_OUT);
    spi_ram_set_enabled(true);
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

/* Fresh machine for each program: ASIC registers and the emulated tape
 * both back to zero.  The tape is cleared mid-reset, while CS is
 * guaranteed deasserted and core 1 is parked. */
static void asic_reset(void) {
    gpio_put(TT_PIN_PROJ_NRST, 0);
    busy_wait_us(asic_clks_us(64u));
    spi_ram_clear();
    gpio_put(TT_PIN_PROJ_NRST, 1);
    busy_wait_us(asic_clks_us(64u));
}

/* ---- instruction feed ---- */

/* Assert instr_valid across exactly one rising edge of the ASIC clock:
 * raise it just after a falling edge, drop it just after the next one.
 * The caller must hold interrupts off. */
static void __not_in_flash_func(feed_instr_raw)(uint8_t op) {
    gpio_put(PIN_INSTR0, op & 1);
    gpio_put(PIN_INSTR1, (op >> 1) & 1);
    gpio_put(PIN_INSTR2, (op >> 2) & 1);

    while (!gpio_get(TT_PIN_PROJ_CLK))
        tight_loop_contents();
    while (gpio_get(TT_PIN_PROJ_CLK)) /* falling edge */
        tight_loop_contents();
    gpio_put(PIN_INSTR_VALID, 1);
    while (!gpio_get(TT_PIN_PROJ_CLK)) /* rising edge: ASIC executes */
        tight_loop_contents();
    while (gpio_get(TT_PIN_PROJ_CLK)) /* falling edge */
        tight_loop_contents();
    gpio_put(PIN_INSTR_VALID, 0);
}

/* feed_instr_raw for ops that transmit nothing back. */
static void feed_instr(uint8_t op) {
    uint32_t save = save_and_disable_interrupts();
    feed_instr_raw(op);
    restore_interrupts(save);
}

/* ---- completion / interrupt polling ---- */

typedef enum { EV_PC_ADVANCED, EV_IRQ, EV_TIMEOUT } event_t;

static event_t await_pc_or_irq(uint8_t expect_pc, uint irq_pin,
                               uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    for (;;) {
        uint64_t snap = gpio_snapshot();
        if ((snap >> irq_pin) & 1)
            return EV_IRQ;
        if (inspect_from(snap) == expect_pc)
            return EV_PC_ADVANCED;
        if (time_reached(dl))
            return EV_TIMEOUT;
    }
}

static bool wait_pc_low(uint8_t expect, uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    while (inspect_from(gpio_snapshot()) != expect) {
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
 * synchronizer.  MSB first. */
static void send10(uint16_t v) {
    for (int i = 9; i >= 0; i--) {
        gpio_put(PIN_RX_BIT, (v >> i) & 1);
        busy_wait_us(serial_half_us);
        gpio_put(PIN_RX_CLK, 1);
        busy_wait_us(serial_half_us);
        gpio_put(PIN_RX_CLK, 0);
        busy_wait_us(serial_half_us);
    }
}

/* ASIC->MCU: TX_CLK idles high, drops to announce a transfer, then each of
 * the 10 bits (MSB first) is stable from one rising edge to the next.
 * Sample one frame. TX_CLK must already be low (start seen) and the
 * caller must hold interrupts off: the bit period is only 2 ASIC
 * clocks. */
static bool sample10(uint16_t *out, absolute_time_t dl) {
    uint16_t v = 0;
    for (int i = 0; i < 10; i++) {
        while (!gpio_get(PIN_TX_CLK)) {
            if (time_reached(dl))
                return false;
        }
        v = (uint16_t)((v << 1) | gpio_get(PIN_TX_BIT));
        while (gpio_get(PIN_TX_CLK)) {
            if (time_reached(dl))
                return false;
        }
    }
    *out = v;
    return true;
}

/* Receive one frame and swallow its phantom repeat (silicon bug 2:
 * serial_tx re-arms on a stale start strobe, so every frame repeats
 * once). The caller must hold interrupts off, from BEFORE the fed
 * instruction can start the transfer: a USB interrupt anywhere from
 * the feed to the end of the phantom outlasts the frames, and the
 * data is then gone. Returns the first frame's value, or -1 on
 * timeout. */
static int recv10_raw(uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    while (gpio_get(PIN_TX_CLK)) { /* wait for start (clock drop) */
        if (time_reached(dl))
            return -1;
    }
    uint16_t v = 0, junk = 0;
    if (!sample10(&v, dl))
        return -1;
    /* The phantom start comes within a few ASIC clocks. Bound the
     * wait so a genuinely missing phantom cannot spin for the
     * caller's full timeout. */
    absolute_time_t pdl = make_timeout_time_us(asic_clks_us(64u));
    while (!time_reached(pdl)) {
        if (!gpio_get(PIN_TX_CLK))
            return sample10(&junk, dl) ? v : -1;
    }
    return -1;
}

/* ---- program load ---- */

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

/* Read BF source from USB CDC until '!' (or Ctrl-D).  Everything that
 * isn't one of the eight ops is a comment.  Anything after the '!' stays
 * in the stream and becomes ',' input for the program. */
static bool read_program(void) {
    printf("# paste program, end with '!'\n");
    n_ops = 0;
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
        if (n_ops == MAX_OPS) {
            overflow = true; /* keep draining until the terminator */
            continue;
        }
        ops[n_ops++] = (uint8_t)op;
        putchar(c); /* echo accepted ops so a paste is visible */
    }
    putchar('\n');
    if (overflow) {
        printf("# program exceeds %u ops (ASIC PC is 10 bits) — discarded\n",
               MAX_OPS);
        return false;
    }
    if (n_ops == 0)
        return false;
    printf("# %u ops\n", n_ops);
    return true;
}

static bool build_jump_table(void) {
    uint16_t stack[MAX_OPS];
    uint sp = 0, max_depth = 0;
    for (uint16_t i = 0; i < n_ops; i++) {
        if (ops[i] == OP_OPEN) {
            stack[sp++] = i;
            if (sp > max_depth)
                max_depth = sp;
        } else if (ops[i] == OP_CLOSE) {
            if (sp == 0) {
                printf("# unmatched ']' at op %u\n", i);
                return false;
            }
            uint16_t j = stack[--sp];
            match[j] = i;
            match[i] = j;
        }
    }
    if (sp != 0) {
        printf("# unmatched '[' at op %u\n", stack[sp - 1]);
        return false;
    }
    if (max_depth > 8)
        printf("# nesting depth %u exceeds the ASIC's 8-deep bracket "
               "stack — deeper loops will misbehave\n",
               max_depth);
    return true;
}

/* ---- error handling ---- */

/* Dump diagnostics, put the feed pins back to idle, and jump out of the
 * run so the command loop stays alive. */
static void __attribute__((noreturn)) die(const char *why, uint16_t pc) {
    printf("\n# !! %s at pc=%u ('%c')\n", why, pc,
           pc < n_ops ? OP_CHAR[ops[pc]] : '?');
    static const char *sel_name[4] = {"data", "ptr", "pc", "bstack"};
    for (uint sel = 0; sel < 4; sel++) {
        set_inspect_sel(sel);
        busy_wait_us(asic_clks_us(4u));
        printf("#    inspect %-6s = 0x%02x\n", sel_name[sel],
               inspect_from(gpio_snapshot()));
    }
    printf("#    irq_jump=%d irq_io=%d tx_clk=%d spi_cs=%d\n",
           gpio_get(PIN_IRQ_JUMP), gpio_get(PIN_IRQ_IO), gpio_get(PIN_TX_CLK),
           gpio_get(PIN_SPI_CS));
    gpio_put(PIN_INSTR_VALID, 0);
    gpio_put(PIN_RX_CLK, 0);
    gpio_put(PIN_RX_BIT, 0);
    set_inspect_sel(INSPECT_PC);
    longjmp(bf_err, 1);
}

/* Host stop (CH_STOP): put the feed pins back to idle and leave the
 * run without the diagnostic dump. The next session's reset gives
 * the ASIC a fresh state. */
static void __attribute__((noreturn)) stop_run(void) {
    gpio_put(PIN_INSTR_VALID, 0);
    gpio_put(PIN_RX_CLK, 0);
    gpio_put(PIN_RX_BIT, 0);
    set_inspect_sel(INSPECT_PC);
    longjmp(bf_err, 2);
}

/* ---- execution ----
 *
 * Two silicon bugs shape this loop (confirmed by step-mode traces,
 * 2026-08-01; see README "Known issues"):
 *
 * 1. serial_tx re-arms on a stale start strobe: every ASIC->MCU frame
 *    is followed by an identical phantom frame. recv10() consumes it
 *    in the same interrupts-off window so it cannot collide with the
 *    next handshake.
 *
 * 2. The SPI cache-refill FSM re-issues its transaction forever (the
 *    transfer_done exit is unreachable). The host therefore never
 *    feeds a pointer move that would leave the ASIC's cache window
 *    (physical cells 0..8): it virtualizes such moves through the
 *    working '.'/',' serial paths and keeps the full 1024-cell tape
 *    in vtape. Synthetic ops advance the ASIC's PC, so the host
 *    models it in `apc`, separate from the program index `pc`.
 *
 * 3. Physical cell 4 (tape_base) has no backing store in the RTL:
 *    the cache mux returns data_current for offset 0 and the save
 *    path drops the value when the pointer leaves. Reads alias the
 *    value carried in; writes are lost. The host therefore never
 *    parks on physical cell 4 — logical cell L maps to physical L
 *    for L < 4 and L + 1 for L >= 4, and the logical 3<->4 move
 *    feeds two physical moves (transit through cell 4 is safe).
 */

#define NATIVE_LIMIT 7u /* logical cells 0..7 live in the ASIC */

static inline uint phys_of(uint16_t l) { return l < 4 ? l : l + 1u; }

static uint8_t vtape[1024];

/* Feed a synthetic '.' and return data_current (the current cell). */
static uint8_t synth_out(uint8_t apc_next, uint16_t pc) {
    uint32_t save = save_and_disable_interrupts();
    feed_instr_raw(OP_OUT);
    event_t ev = await_pc_or_irq(apc_next, PIN_IRQ_IO, PC_TIMEOUT_US);
    int v = ev == EV_IRQ ? recv10_raw(SERIAL_TIMEOUT_US) : -1;
    restore_interrupts(save);
    if (ev != EV_IRQ)
        die("no interrupt_io for synthetic '.'", pc);
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
        uint8_t op = ops[s->pc];
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
                s->pc = match[s->pc];
            } else {
                die("no response to '['", s->pc);
            }
            break;
        }

        case OP_CLOSE: {
            /* Interrupts off from the feed through the TX frames:
             * a USB interrupt in that span outlasts the frames and
             * the jump target is lost. */
            uint32_t save = save_and_disable_interrupts();
            feed_instr_raw(op);
            s->executed++;
            event_t ev = await_pc_or_irq(anext, PIN_IRQ_JUMP, PC_TIMEOUT_US);
            int tgt = ev == EV_IRQ ? recv10_raw(SERIAL_TIMEOUT_US) : -1;
            restore_interrupts(save);
            if (ev == EV_PC_ADVANCED) { /* data==0: exited loop */
                if (s->sp)
                    s->sp--;
                s->apc++;
                s->pc++;
            } else if (ev == EV_IRQ) { /* data!=0: ASIC transmits target */
                if (tgt < 0)
                    die("no jump target transmitted for ']'", s->pc);
                if (!wait_gpio_low(PIN_IRQ_JUMP, SERIAL_TIMEOUT_US))
                    die("interrupt_jump stuck after ']'", s->pc);
                if (s->sp && (uint16_t)tgt != (s->abstack[s->sp - 1] & 0x3FF))
                    printf("# !! ']' at %u jumped to %d, expected %u "
                           "(bracket stack desync?)\n",
                           s->pc, tgt, s->abstack[s->sp - 1] & 0x3FF);
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
                s->pc = match[s->pc]; /* the matching '[' re-executes */
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
            int c = host_getchar();
            if (c < 0)
                die("host disconnected during ','", s->pc);
            if (c == CH_STOP)
                stop_run();
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
            uint32_t save = save_and_disable_interrupts();
            feed_instr_raw(op);
            s->executed++;
            event_t ev = await_pc_or_irq(anext, PIN_IRQ_IO, PC_TIMEOUT_US);
            int v = ev == EV_IRQ ? recv10_raw(SERIAL_TIMEOUT_US) : -1;
            restore_interrupts(save);
            if (ev != EV_IRQ)
                die("no interrupt_io for '.'", s->pc);
            if (v < 0)
                die("no data transmitted for '.'", s->pc);
            putchar(v & 0xFF);
            stdio_flush();
            if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
                die("interrupt_io stuck after '.'", s->pc);
            if (!wait_pc_low(anext, PC_TIMEOUT_US))
                die("'.' never retired", s->pc);
            s->apc++;
            s->pc++;
            break;
        }
        }
    }
}

static void run(bf_state_t *s) {
    uint32_t n = 0;
    while (s->pc < n_ops) {
        exec_one(s);
        if ((++n & 0xFFu) == 0 && host_wants_stop())
            stop_run();
    }
    printf("\n# halted: %lu instructions executed\n",
           (unsigned long)s->executed);
}

const char *bf_run_session(void) {
    pushback = -1;
    if (!read_program())
        return "bad-program";
    if (!build_jump_table())
        return "bad-program";
    asic_reset();

    gpio_put(TT_PIN_LED, 1);
    bf_state_t st;
    state_init(&st);
    const char *result = NULL;
    switch (setjmp(bf_err)) {
    case 0:
        run(&st);
        break;
    case 2:
        printf("\n# stopped by host\n");
        result = "stopped";
        break;
    default:
        result = "run-fail";
        break;
    }
    gpio_put(TT_PIN_LED, 0);
    return result;
}

/* ---- instruction-level debugger ---- */

/* Read one inspect value with the mux settled. */
static uint8_t inspect_read(uint sel) {
    set_inspect_sel(sel);
    busy_wait_us(asic_clks_us(4u));
    return inspect_from(gpio_snapshot());
}

/* One machine-parseable state line per step. inspect_sel returns to
 * PC afterward: the retirement handshake depends on it. */
static void print_dbg_state(const bf_state_t *s) {
    uint8_t data = inspect_read(INSPECT_DATA);
    uint8_t bstk = inspect_read(INSPECT_BSTACK);
    set_inspect_sel(INSPECT_PC);
    busy_wait_us(asic_clks_us(4u));
    printf("# dbg pc=%u op=%c vptr=%u data=%02x bstk=%02x exec=%lu\n",
           s->pc, s->pc < n_ops ? OP_CHAR[ops[s->pc]] : '-', s->vptr, data,
           bstk, (unsigned long)s->executed);
    stdio_flush();
}

/* Breakpoints on program indexes. Cleared at each session start;
 * the host re-sends its set. */
static uint8_t bp[MAX_OPS / 8];

static inline bool bp_get(uint16_t i) {
    return (bp[i >> 3] >> (i & 7)) & 1;
}

/* 'b' followed by a decimal index toggles a breakpoint. The digits
 * end at the first non-digit byte (the host sends a newline). */
static void bp_command(void) {
    uint32_t v = 0;
    bool any = false;
    for (;;) {
        int c = getchar_timeout_us(200000);
        if (c < '0' || c > '9')
            break;
        v = v * 10 + (uint32_t)(c - '0');
        any = true;
    }
    if (!any || v >= n_ops) {
        printf("# dbg bp out of range\n");
        return;
    }
    bp[v >> 3] ^= (uint8_t)(1u << (v & 7));
    printf("# dbg bp %s %lu\n", bp_get((uint16_t)v) ? "set" : "clear",
           (unsigned long)v);
    stdio_flush();
}

/* Like bf_run_session(), but the host paces execution: 'n' runs one
 * instruction, 'c' runs to the next breakpoint or the end, 'q' stops
 * the session. */
const char *bf_debug_session(void) {
    pushback = -1;
    if (!read_program())
        return "bad-program";
    if (!build_jump_table())
        return "bad-program";
    asic_reset();

    gpio_put(TT_PIN_LED, 1);
    bf_state_t st;
    state_init(&st);
    memset(bp, 0, sizeof bp);
    int j = setjmp(bf_err);
    if (j) {
        gpio_put(TT_PIN_LED, 0);
        return j == 2 ? "stopped" : "run-fail";
    }
    printf("# dbg ready: n=step c=continue q=quit b<n>=breakpoint\n");
    print_dbg_state(&st);
    while (st.pc < n_ops) {
        int ch = host_getchar();
        if (ch < 0) {
            gpio_put(TT_PIN_LED, 0);
            return "host-lost";
        }
        if (ch == 'n') {
            exec_one(&st);
            print_dbg_state(&st);
        } else if (ch == 'c') {
            /* at least one instruction, so a continue from a
             * breakpoint does not stop on the same spot */
            uint32_t n = 0;
            do {
                exec_one(&st);
                if ((++n & 0xFFu) == 0 && host_wants_stop())
                    stop_run();
            } while (st.pc < n_ops && !bp_get(st.pc));
            if (st.pc < n_ops) {
                printf("# dbg break at %u\n", st.pc);
                print_dbg_state(&st);
            }
        } else if (ch == 'b') {
            bp_command();
        } else if (ch == 'q' || ch == CH_STOP) {
            gpio_put(TT_PIN_LED, 0);
            printf("# dbg stopped by host\n");
            return NULL;
        }
        /* other bytes (line ends and so on) are ignored */
    }
    if (st.pc >= n_ops)
        printf("\n# halted: %lu instructions executed\n",
               (unsigned long)st.executed);
    gpio_put(TT_PIN_LED, 0);
    return NULL;
}
