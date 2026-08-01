/*
 * Line-based command protocol over USB CDC.
 *
 * One command per line. Each command gets exactly one reply line:
 * "ok [payload]" or "err <token>". Informational output is prefixed
 * with "# ". Hex arguments are two hex digits, no "0x". The `bf`
 * command is the one exception: it replies "ok bf", runs an
 * interactive BF session, then gives the final "ok done" / "err ...".
 *
 * Pin ownership: `design` always applies the safe profile first (all
 * uio pins released to inputs, ui pins driven 0). Only `bf` arms the
 * BF pin roles, and only when the BF design is selected.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"

#include "bf_run.h"
#include "board.h"
#include "clock.h"
#include "commands.h"
#include "tt_pins.h"

#define PROTO_VERSION 1u
#define BF_MIN_HZ 50000u   /* below this, feed_instr's interrupts-off \
                              window starves USB and the 200 ms       \
                              handshake timeouts trip */
#define BF_MAX_HZ 2000000u /* above this, the MCU cannot bit-bang the \
                              instr_valid pulse and serial links      \
                              (and >= 500 kHz already bit-slips) */

static int current_design = -1; /* -1 = none selected since boot */
static bool bf_armed;
static bool ui_driven = true; /* false: ui pins released for DIP/PMOD */
static uint8_t ui_value;
static uint8_t uio_dir_mask; /* 1 = MCU drives the pad */
static uint8_t uio_out_value;

static char reply[96]; /* handlers put their "ok" payload here */

/* pins_safe() in board.c does not know this file's mirror state;
 * reset it together whenever the safe profile is applied. */
static void apply_safe_profile(void) {
    pins_safe();
    bf_armed = false;
    ui_driven = true; /* pins_safe drives the ui pins low */
    ui_value = 0;
    uio_dir_mask = 0;
    uio_out_value = 0;
}

/* ---- argument parsing ---- */

static bool parse_u32(const char *s, uint32_t *out) {
    char *end;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end)
        return false;
    *out = (uint32_t)v;
    return true;
}

static bool parse_hex8(const char *s, uint8_t *out) {
    char *end;
    unsigned long v = strtoul(s, &end, 16);
    if (end == s || *end || v > 255)
        return false;
    *out = (uint8_t)v;
    return true;
}

static uint8_t read_byte(uint base) {
    uint8_t v = 0;
    for (uint i = 0; i < 8; i++)
        v |= (uint8_t)(gpio_get(base + i) << i);
    return v;
}

/* ---- command handlers ---- */
/* A handler returns NULL for success (payload, if any, in reply[]) or
 * an error token. */

static const char *cmd_hello(int argc, char **argv) {
    (void)argc;
    (void)argv;
    sprintf(reply, "tt-explorer %u bf=%u", PROTO_VERSION, BF_DESIGN_ADDR);
    return NULL;
}

static const char *cmd_status(int argc, char **argv) {
    (void)argc;
    (void)argv;
    sprintf(reply,
            "design=%d mode=%s freq=%lu ui=%02x uidrv=%d uiod=%02x bf=%d",
            current_design, clk_mode == CLK_RUN ? "run" : "step",
            (unsigned long)clk_hz, ui_value, ui_driven ? 1 : 0,
            uio_dir_mask, bf_armed ? 1 : 0);
    return NULL;
}

static const char *cmd_freq(int argc, char **argv) {
    uint32_t hz, actual;
    if (argc != 2 || !parse_u32(argv[1], &hz))
        return "bad-arg";
    if (!asic_clk_set_hz(hz, &actual))
        return "range";
    sprintf(reply, "%lu", (unsigned long)actual);
    return NULL;
}

static const char *cmd_stop(int argc, char **argv) {
    (void)argc;
    (void)argv;
    asic_clk_stop();
    return NULL;
}

static const char *cmd_step(int argc, char **argv) {
    uint32_t n = 1;
    if (argc > 2 || (argc == 2 && !parse_u32(argv[1], &n)))
        return "bad-arg";
    if (n < 1 || n > CLK_STEP_MAX)
        return "range";
    if (!asic_clk_step(n))
        return "mode";
    sprintf(reply, "%lu", (unsigned long)n);
    return NULL;
}

static const char *cmd_resume(int argc, char **argv) {
    (void)argc;
    (void)argv;
    asic_clk_resume();
    sprintf(reply, "%lu", (unsigned long)clk_hz);
    return NULL;
}

static const char *cmd_design(int argc, char **argv) {
    uint32_t n;
    if (argc != 2 || !parse_u32(argv[1], &n))
        return "bad-arg";
    if (n > 1023)
        return "range";
    apply_safe_profile();
    tt_select_design(n);
    gpio_put(TT_PIN_PROJ_NRST, 0);
    sleep_ms(2);
    gpio_put(TT_PIN_PROJ_NRST, 1);
    current_design = (int)n;
    sprintf(reply, "%lu", (unsigned long)n);
    return NULL;
}

static const char *cmd_reset(int argc, char **argv) {
    if (argc == 1) { /* pulse */
        gpio_put(TT_PIN_PROJ_NRST, 0);
        sleep_ms(2);
        gpio_put(TT_PIN_PROJ_NRST, 1);
        return NULL;
    }
    if (argc == 2 && !strcmp(argv[1], "1")) { /* assert: NRST low */
        gpio_put(TT_PIN_PROJ_NRST, 0);
        return NULL;
    }
    if (argc == 2 && !strcmp(argv[1], "0")) { /* release */
        gpio_put(TT_PIN_PROJ_NRST, 1);
        return NULL;
    }
    return "bad-arg";
}

/* The board wires the DIP switches and the PMOD to the same nets as
 * the MCU's ui pins. `ui off` releases the pins so those sources can
 * drive (the official firmware calls this ASIC_MANUAL_INPUTS). */
static const char *cmd_ui(int argc, char **argv) {
    if (argc == 1) { /* read the pad levels (useful when released) */
        sprintf(reply, "%02x", read_byte(TT_GPIO_UI_BASE));
        return NULL;
    }
    if (argc == 2 && !strcmp(argv[1], "off")) {
        for (uint i = 0; i < 8; i++)
            gpio_set_dir(TT_GPIO_UI_BASE + i, GPIO_IN);
        ui_driven = false;
        return NULL;
    }
    uint8_t v;
    if (argc != 2 || !parse_hex8(argv[1], &v))
        return "bad-arg";
    for (uint i = 0; i < 8; i++) {
        uint p = TT_GPIO_UI_BASE + i;
        gpio_put(p, (v >> i) & 1);
        gpio_set_dir(p, GPIO_OUT);
    }
    ui_driven = true;
    ui_value = v;
    return NULL;
}

static const char *cmd_uo(int argc, char **argv) {
    (void)argc;
    (void)argv;
    sprintf(reply, "%02x", read_byte(TT_GPIO_UO_BASE));
    return NULL;
}

static const char *cmd_uio(int argc, char **argv) {
    (void)argc;
    (void)argv;
    sprintf(reply, "%02x", read_byte(TT_GPIO_UIO_BASE));
    return NULL;
}

static const char *cmd_uiod(int argc, char **argv) {
    if (argc == 2) {
        uint8_t m;
        if (!parse_hex8(argv[1], &m))
            return "bad-arg";
        for (uint i = 0; i < 8; i++) {
            uint p = TT_GPIO_UIO_BASE + i;
            if ((m >> i) & 1) {
                gpio_put(p, (uio_out_value >> i) & 1);
                gpio_set_dir(p, GPIO_OUT);
            } else {
                gpio_set_dir(p, GPIO_IN);
            }
        }
        uio_dir_mask = m;
    } else if (argc != 1) {
        return "bad-arg";
    }
    sprintf(reply, "%02x", uio_dir_mask);
    return NULL;
}

static const char *cmd_uiow(int argc, char **argv) {
    uint8_t v;
    if (argc != 2 || !parse_hex8(argv[1], &v))
        return "bad-arg";
    uio_out_value = v;
    for (uint i = 0; i < 8; i++) {
        if ((uio_dir_mask >> i) & 1)
            gpio_put(TT_GPIO_UIO_BASE + i, (v >> i) & 1);
    }
    return NULL;
}

static const char *cmd_bf(int argc, char **argv) {
    (void)argc;
    (void)argv;
    if (current_design != (int)BF_DESIGN_ADDR)
        return "not-bf-design";
    if (clk_mode != CLK_RUN)
        return "need-clock";
    if (clk_hz < BF_MIN_HZ)
        return "too-slow";
    if (clk_hz > BF_MAX_HZ)
        return "too-fast";
    pins_bf();
    bf_armed = true;
    ui_driven = true; /* pins_bf drives the ui pins */
    printf("ok bf\n");
    const char *err = bf_run_session();
    /* Drain input the program did not consume (e.g. ',' bytes left
     * over after an error) so it cannot pollute the next command. */
    while (getchar_timeout_us(100000) != PICO_ERROR_TIMEOUT)
        ;
    if (err)
        return err;
    strcpy(reply, "done");
    return NULL;
}

static const char *cmd_help(int argc, char **argv);

static const struct cmd {
    const char *name;
    const char *(*fn)(int argc, char **argv);
    const char *help;
} cmds[] = {
    {"hello", cmd_hello, "hello              -> ok tt-explorer <ver> bf=<addr>"},
    {"status", cmd_status, "status             -> ok design= mode= freq= ui= uiod= bf="},
    {"freq", cmd_freq, "freq <hz>          set clock, 10 Hz .. clk_sys/2"},
    {"stop", cmd_stop, "stop               park clock low, step mode"},
    {"step", cmd_step, "step [n]           n clock pulses (step mode only)"},
    {"resume", cmd_resume, "resume             back to run mode at last freq"},
    {"design", cmd_design, "design <n>         select mux design 0..1023 + reset"},
    {"reset", cmd_reset, "reset [1|0]        pulse, or assert(1)/release(0) NRST"},
    {"ui", cmd_ui, "ui <hh>|off|(none) drive ui_in, release for DIP/PMOD, read"},
    {"uo", cmd_uo, "uo                 read uo_out byte"},
    {"uio", cmd_uio, "uio                read uio pad levels"},
    {"uiod", cmd_uiod, "uiod [hh]          set/get uio dir mask, 1=MCU drives"},
    {"uiow", cmd_uiow, "uiow <hh>          write uio output latch"},
    {"bf", cmd_bf, "bf                 run a BF session (BF design only)"},
    {"help", cmd_help, "help               this list"},
};

static const char *cmd_help(int argc, char **argv) {
    (void)argc;
    (void)argv;
    for (uint i = 0; i < count_of(cmds); i++)
        printf("# %s\n", cmds[i].help);
    return NULL;
}

/* ---- line reader / dispatch ---- */

/* Echo input so a human in a bare terminal sees what they type. '\r'
 * and '\n' both end a line, so CRLF costs one ignored empty line. */
static void read_line(char *buf, size_t cap) {
    size_t n = 0;
    for (;;) {
        int c = getchar();
        if (c == '\r' || c == '\n') {
            if (n == 0)
                continue;
            putchar('\n');
            buf[n] = 0;
            return;
        }
        if (c == 0x08 || c == 0x7f) { /* backspace / DEL */
            if (n) {
                n--;
                printf("\b \b");
            }
            continue;
        }
        if (c < 0x20 || c > 0x7e)
            continue;
        if (n + 1 < cap) {
            buf[n++] = (char)c;
            putchar(c);
        }
    }
}

void command_loop(void) {
    char line[128];
    for (;;) {
        read_line(line, sizeof line);

        char *argv[4];
        int argc = 0;
        for (char *t = strtok(line, " "); t && argc < 4;
             t = strtok(NULL, " "))
            argv[argc++] = t;
        if (argc == 0)
            continue;

        const struct cmd *c = NULL;
        for (uint i = 0; i < count_of(cmds); i++) {
            if (!strcmp(argv[0], cmds[i].name)) {
                c = &cmds[i];
                break;
            }
        }
        if (!c) {
            printf("err unknown\n");
            continue;
        }
        reply[0] = 0;
        const char *err = c->fn(argc, argv);
        if (err)
            printf("err %s\n", err);
        else if (reply[0])
            printf("ok %s\n", reply);
        else
            printf("ok\n");
        stdio_flush();
    }
}
