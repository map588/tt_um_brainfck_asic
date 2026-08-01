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

/* ---- execution ---- */

static void run(void) {
    uint16_t pc = 0;
    uint32_t executed = 0;

    set_inspect_sel(INSPECT_PC);

    while (pc < n_ops) {
        uint8_t op = ops[pc];
        uint8_t next = (uint8_t)(pc + 1); /* PC low byte after this op */
        feed_instr(op);
        executed++;

        switch (op) {
        case OP_SUB:
        case OP_ADD:
        case OP_LEFT:
        case OP_RIGHT:
            /* '<'/'>' may stall for an SPI cache refill; PC advance is the
             * only completion signal either way. */
            if (!wait_pc_low(next, PC_TIMEOUT_US))
                die("timeout waiting for op to retire", pc);
            pc++;
            break;

        case OP_OPEN: {
            event_t ev = await_pc_or_irq(next, PIN_IRQ_JUMP, PC_TIMEOUT_US);
            if (ev == EV_PC_ADVANCED) { /* data!=0: entered loop */
                pc++;
            } else if (ev == EV_IRQ) { /* data==0: ASIC wants skip target */
                uint16_t tgt = match[pc];
                busy_wait_us(rx_settle_us);
                send10(tgt);
                if (!wait_gpio_low(PIN_IRQ_JUMP, SERIAL_TIMEOUT_US))
                    die("interrupt_jump stuck after skip", pc);
                if (!wait_pc_low((uint8_t)tgt, PC_TIMEOUT_US))
                    die("PC never reached skip target", pc);
                pc = tgt; /* ASIC now executes the matching ']' (data==0:
                             pops the bracket it pushed while waiting) */
            } else {
                die("no response to '['", pc);
            }
            break;
        }

        case OP_CLOSE: {
            event_t ev = await_pc_or_irq(next, PIN_IRQ_JUMP, PC_TIMEOUT_US);
            if (ev == EV_PC_ADVANCED) { /* data==0: exited loop */
                pc++;
            } else if (ev == EV_IRQ) { /* data!=0: ASIC transmits target */
                int tgt = recv10(SERIAL_TIMEOUT_US);
                if (tgt < 0)
                    die("no jump target transmitted for ']'", pc);
                if (!wait_gpio_low(PIN_IRQ_JUMP, SERIAL_TIMEOUT_US))
                    die("interrupt_jump stuck after ']'", pc);
                if ((uint16_t)tgt != match[pc])
                    printf("# !! ']' at %u jumped to %d, expected %u "
                           "(bracket stack desync?)\n",
                           pc, tgt, match[pc]);
                pc = (uint16_t)tgt; /* the matching '[' re-executes */
            } else {
                die("no response to ']'", pc);
            }
            break;
        }

        case OP_IN: {
            if (await_pc_or_irq(next, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
                die("no interrupt_io for ','", pc);
            int c = getchar(); /* blocks on USB CDC */
            busy_wait_us(rx_settle_us);
            send10((uint16_t)(c & 0xFF));
            if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
                die("interrupt_io stuck after ','", pc);
            if (!wait_pc_low(next, PC_TIMEOUT_US))
                die("',' never retired", pc);
            pc++;
            break;
        }

        case OP_OUT: {
            if (await_pc_or_irq(next, PIN_IRQ_IO, PC_TIMEOUT_US) != EV_IRQ)
                die("no interrupt_io for '.'", pc);
            int v = recv10(SERIAL_TIMEOUT_US);
            if (v < 0)
                die("no data transmitted for '.'", pc);
            putchar(v & 0xFF);
            stdio_flush();
            if (!wait_gpio_low(PIN_IRQ_IO, SERIAL_TIMEOUT_US))
                die("interrupt_io stuck after '.'", pc);
            if (!wait_pc_low(next, PC_TIMEOUT_US))
                die("'.' never retired", pc);
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
