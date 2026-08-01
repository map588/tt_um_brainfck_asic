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
#include "pico/stdlib.h"

#include "bf_run.h"
#include "clock.h"
#include "spi_ram.h"
#include "tt_pins.h"

#define MAX_OPS 1024u /* ASIC PC is 10 bits */
#define PC_TIMEOUT_US 200000u
#define SERIAL_TIMEOUT_US 200000u

enum { OP_SUB, OP_ADD, OP_LEFT, OP_RIGHT, OP_OPEN, OP_CLOSE, OP_IN, OP_OUT };
static const char OP_CHAR[8] = {'-', '+', '<', '>', '[', ']', ',', '.'};

static uint8_t ops[MAX_OPS];
static uint16_t match[MAX_OPS];
static uint16_t n_ops;

static jmp_buf bf_err; /* die() jumps here; set in bf_run_session() */

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
 * raise it just after a falling edge, drop it just after the next one. */
static void __not_in_flash_func(feed_instr)(uint8_t op) {
    gpio_put(PIN_INSTR0, op & 1);
    gpio_put(PIN_INSTR1, (op >> 1) & 1);
    gpio_put(PIN_INSTR2, (op >> 2) & 1);

    uint32_t save = save_and_disable_interrupts();
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
 * the 10 bits (MSB first) is stable from one rising edge to the next.  The
 * bit period is only 2 ASIC clocks, so interrupts stay off while sampling
 * (USB can afford ~50 us). */
static int recv10(uint32_t timeout_us) {
    absolute_time_t dl = make_timeout_time_us(timeout_us);
    while (gpio_get(PIN_TX_CLK)) { /* wait for start (clock drop) */
        if (time_reached(dl))
            return -1;
    }
    uint32_t save = save_and_disable_interrupts();
    uint16_t v = 0;
    for (int i = 0; i < 10; i++) {
        while (!gpio_get(PIN_TX_CLK)) {
            if (time_reached(dl)) {
                restore_interrupts(save);
                return -1;
            }
        }
        v = (uint16_t)((v << 1) | gpio_get(PIN_TX_BIT));
        while (gpio_get(PIN_TX_CLK)) {
            if (time_reached(dl)) {
                restore_interrupts(save);
                return -1;
            }
        }
    }
    restore_interrupts(save);
    return v;
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
        int c = getchar();
        if (c == '!' || c == 0x04)
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

/* ---- execution ----
 *
 * Two silicon bugs shape this loop (confirmed by step-mode traces,
 * 2026-08-01; see README "Known issues"):
 *
 * 1. serial_tx re-arms on a stale start strobe: every ASIC->MCU frame
 *    is followed by an identical phantom frame. drain_phantom()
 *    consumes it so it cannot collide with the next handshake.
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

static void drain_phantom(uint16_t pc) {
    if (recv10(SERIAL_TIMEOUT_US) < 0)
        die("phantom TX frame missing", pc);
}

/* Feed a synthetic '.' and return data_current (the current cell). */
static uint8_t synth_out(uint8_t apc_next, uint16_t pc) {
    feed_instr(OP_OUT);
    if (await_pc_or_irq(apc_next, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
        die("no interrupt_io for synthetic '.'", pc);
    int v = recv10(SERIAL_TIMEOUT_US);
    if (v < 0)
        die("no data for synthetic '.'", pc);
    drain_phantom(pc);
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

static void run(void) {
    uint16_t pc = 0;     /* program index */
    uint16_t apc = 0;    /* model of the ASIC's PC (drifts from pc) */
    uint16_t vptr = 0;   /* logical tape pointer */
    uint16_t abstack[8]; /* ASIC PC values it pushed for '[' */
    uint sp = 0;
    uint32_t executed = 0;

    memset(vtape, 0, sizeof vtape);
    set_inspect_sel(INSPECT_PC);

    while (pc < n_ops) {
        uint8_t op = ops[pc];
        uint8_t anext = (uint8_t)(apc + 1); /* ASIC PC low byte after op */

        switch (op) {
        case OP_SUB:
        case OP_ADD:
            feed_instr(op);
            executed++;
            if (!wait_pc_low(anext, PC_TIMEOUT_US))
                die("timeout waiting for op to retire", pc);
            apc++;
            pc++;
            break;

        case OP_LEFT:
        case OP_RIGHT: {
            uint16_t nv = vptr;
            if (op == OP_RIGHT) {
                if (vptr < 1023)
                    nv = vptr + 1;
            } else {
                if (vptr > 0)
                    nv = vptr - 1;
            }
            if (nv == vptr) { /* blocked at a tape edge: pure no-op */
                pc++;
                break;
            }
            if (vptr <= NATIVE_LIMIT && nv <= NATIVE_LIMIT) {
                /* Native move inside the cache window. Crossing
                 * logical 3<->4 takes two physical moves (transit
                 * through the broken physical cell 4). */
                uint pa = phys_of(vptr), pb = phys_of(nv);
                uint moves = pb > pa ? pb - pa : pa - pb;
                while (moves--) {
                    feed_instr(op);
                    executed++;
                    if (!wait_pc_low((uint8_t)(apc + 1), PC_TIMEOUT_US))
                        die("timeout waiting for op to retire", pc);
                    apc++;
                }
            } else {
                /* Virtual move: save the current cell, load the target
                 * cell. The ASIC's ptr stays parked at the window edge. */
                vtape[vptr] = synth_out(anext, pc);
                apc++;
                synth_in(vtape[nv], (uint8_t)(apc + 1), pc);
                apc++;
                executed += 2;
            }
            vptr = nv;
            pc++;
            break;
        }

        case OP_OPEN: {
            feed_instr(op);
            executed++;
            event_t ev = await_pc_or_irq(anext, PIN_IRQ_JUMP, PC_TIMEOUT_US);
            if (ev == EV_PC_ADVANCED) { /* data!=0: entered loop */
                if (sp < count_of(abstack))
                    abstack[sp] = apc;
                sp++;
                apc++;
                pc++;
            } else if (ev == EV_IRQ) { /* data==0: ASIC wants skip target */
                /* The ASIC pushed apc. Send apc back so its PC does not
                 * move, then run the matching ']' (data==0: pops). */
                busy_wait_us(rx_settle_us);
                send10(apc & 0x3FF);
                if (!wait_gpio_low(PIN_IRQ_JUMP, SERIAL_TIMEOUT_US))
                    die("interrupt_jump stuck after skip", pc);
                if (!wait_pc_low((uint8_t)apc, PC_TIMEOUT_US))
                    die("PC never reached skip target", pc);
                if (sp < count_of(abstack))
                    abstack[sp] = apc;
                sp++;
                pc = match[pc];
            } else {
                die("no response to '['", pc);
            }
            break;
        }

        case OP_CLOSE: {
            feed_instr(op);
            executed++;
            event_t ev = await_pc_or_irq(anext, PIN_IRQ_JUMP, PC_TIMEOUT_US);
            if (ev == EV_PC_ADVANCED) { /* data==0: exited loop */
                if (sp)
                    sp--;
                apc++;
                pc++;
            } else if (ev == EV_IRQ) { /* data!=0: ASIC transmits target */
                int tgt = recv10(SERIAL_TIMEOUT_US);
                if (tgt < 0)
                    die("no jump target transmitted for ']'", pc);
                drain_phantom(pc);
                if (!wait_gpio_low(PIN_IRQ_JUMP, SERIAL_TIMEOUT_US))
                    die("interrupt_jump stuck after ']'", pc);
                if (sp && (uint16_t)tgt != (abstack[sp - 1] & 0x3FF))
                    printf("# !! ']' at %u jumped to %d, expected %u "
                           "(bracket stack desync?)\n",
                           pc, tgt, abstack[sp - 1] & 0x3FF);
                uint16_t back;
                if (sp >= 2) {
                    /* Silicon: the WAIT_JUMP start branch fires twice
                     * before tx_busy rises, so the ASIC pops TWO
                     * entries and its PC ends at the second one. */
                    sp -= 2;
                    back = abstack[sp];
                    if (!wait_pc_low((uint8_t)back, PC_TIMEOUT_US))
                        die("PC never reached jump target", pc);
                    apc = back;
                    /* Restore the lost entry with a synthetic '[' —
                     * data is not zero here, so it pushes and advances. */
                    feed_instr(OP_OPEN);
                    executed++;
                    if (await_pc_or_irq((uint8_t)(apc + 1), PIN_IRQ_JUMP,
                                        PC_TIMEOUT_US) != EV_PC_ADVANCED)
                        die("synthetic '[' did not push", pc);
                    abstack[sp++] = apc;
                    apc++;
                } else {
                    back = sp ? abstack[--sp] : 0;
                    if (!wait_pc_low((uint8_t)back, PC_TIMEOUT_US))
                        die("PC never reached jump target", pc);
                    apc = back;
                }
                pc = match[pc]; /* the matching '[' re-executes */
            } else {
                die("no response to ']'", pc);
            }
            break;
        }

        case OP_IN: {
            feed_instr(op);
            executed++;
            if (await_pc_or_irq(anext, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
                die("no interrupt_io for ','", pc);
            int c = getchar(); /* blocks on USB CDC */
            busy_wait_us(rx_settle_us);
            send10((uint16_t)(c & 0xFF));
            if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
                die("interrupt_io stuck after ','", pc);
            if (!wait_pc_low(anext, PC_TIMEOUT_US))
                die("',' never retired", pc);
            apc++;
            pc++;
            break;
        }

        case OP_OUT: {
            feed_instr(op);
            executed++;
            if (await_pc_or_irq(anext, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
                die("no interrupt_io for '.'", pc);
            int v = recv10(SERIAL_TIMEOUT_US);
            if (v < 0)
                die("no data transmitted for '.'", pc);
            drain_phantom(pc);
            putchar(v & 0xFF);
            stdio_flush();
            if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
                die("interrupt_io stuck after '.'", pc);
            if (!wait_pc_low(anext, PC_TIMEOUT_US))
                die("'.' never retired", pc);
            apc++;
            pc++;
            break;
        }
        }
    }

    printf("\n# halted: %lu instructions executed\n", (unsigned long)executed);
}

const char *bf_run_session(void) {
    if (!read_program())
        return "bad-program";
    if (!build_jump_table())
        return "bad-program";
    asic_reset();

    gpio_put(TT_PIN_LED, 1);
    const char *result = NULL;
    if (setjmp(bf_err))
        result = "run-fail";
    else
        run();
    gpio_put(TT_PIN_LED, 0);
    return result;
}
