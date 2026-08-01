# RP2350 host firmware

Runs on the Tiny Tapeout demo board v3 (RP2350B) and drives
`tt_um_brainfck_asic` — either real silicon or the ICE40UP5K ASIC-sim
carrier built by the `fpga` workflow.

## What it does

- **Core 0** — runs a line-based command protocol over USB CDC
  (`src/commands.c`): select a shuttle design through the mux, set or
  single-step the project clock, peek/poke the ui/uo/uio pins, and run
  BF sessions on `tt_um_brainfck_asic` (`src/bf_run.c`: feeds the
  program one instruction at a time, mirrors the ASIC's PC, answers
  `interrupt_jump` with the precomputed bracket-match table, and
  bridges `,` / `.` to the serial port).
- **Core 1** — bit-banged 23LC1024-style SPI RAM slave: the 64 KiB tape
  the ASIC pages its 9-byte cache against. Parked while a non-BF design
  is selected.

## Command protocol

One command per line; each command gets exactly one reply line,
`ok [payload]` or `err <token>`. Informational lines start with `# `.
Type `help` on the port for the list. The `../explorer` TUI speaks
this protocol; a bare terminal (`tio`, `screen`) works too.

| Command | Effect |
|---|---|
| `hello` | `ok tt-explorer 1 bf=448` — protocol version, BF mux address |
| `status` | design, clock mode/freq, pin state |
| `freq <hz>` | free-running clock, 10 Hz – 2 MHz (PWM) |
| `stop` / `step [n]` / `resume` | park the clock low, pulse it n times, restart PWM |
| `design <n>` | safe pin profile, mux-select design n, reset pulse |
| `reset [1\|0]` | pulse (no arg), assert, or release the project reset |
| `ui <hh>` / `uo` / `uio` | write ui_in, read uo_out / uio pads (hex byte) |
| `uiod [hh]` / `uiow <hh>` | uio direction mask (1 = MCU drives) / output latch |
| `bf` | interactive BF session (BF design + running clock required) |

Measured on ttsky25b silicon (2026-08-01): BF runs are fully reliable
at `freq 200000` and below. At 500 kHz+ the ASIC→MCU serial link
returns bit-slipped data (e.g. 0x42 arrives as 0x21). Use 200 kHz for
BF work until the link is debugged.

A `bf` session works as before: paste BF source, end with `!`.
Anything that is not one of the eight BF ops is a comment; anything
after the `!` is consumed as `,` input by the running program:

```
bf
ok bf
# paste program, end with '!'
,[.,]!hello
```

## Build

Requires the pico-sdk (≥ 2.0) and the Arm toolchain from the official
installer (the Homebrew `arm-none-eabi-gcc` lacks newlib's `nosys.specs`).

```sh
cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" .   # once
cmake -S . -B build -G Ninja -DPICO_TOOLCHAIN_PATH=$HOME/.pico-sdk/toolchain/14_2_Rel1
cmake --build build
```

Flash `build/bf_host.uf2` over BOOTSEL (or `picotool load -f -x`).
The firmware waits for the USB serial port to open, then serves the
command protocol.

Configuration knobs:

| Define | Where | Default | Notes |
|---|---|---|---|
| `CLK_HZ_MAX` | `include/clock.h` | 2 MHz | Serial links and the `instr_valid` pulse are bit-banged against this clock. |
| `BF_DESIGN_ADDR` | `src/commands.c` | 448 | tt_um_brainfck_asic mux slot on ttsky25b; the FPGA sim ignores the mux. |
| `BF_MIN_HZ` | `src/commands.c` | 50 kHz | `bf` refuses to run slower (USB starvation, handshake timeouts). |
| `WAIT_FOR_USB` | `src/main.c` | 1 | Hold boot until a terminal attaches. |
| `MAX_OPS` | `src/bf_run.c` | 1024 | Program size cap — the ASIC PC is 10 bits. |

For the v3 *Alpha* prototype board add `-DTT_DBV3_ALPHA` (different GPIO
map, see `include/tt_pins.h`).

## Protocol notes (things the RTL made the firmware do)

1. **`instr_valid` width is critical.** The core executes on *every*
   rising clock edge where `instr_valid` is high, so a pulse wider than
   one clock period double-executes. The firmware owns the clock and
   raises/drops the pulse between two consecutive falling edges — exactly
   one rising edge samples it high.

2. **SPI cache refills are invisible.** `irq_cache_pulse` exists in
   `bf_asic.v` but is never set, so a `<`/`>` that crosses the cache
   window stalls the core with no external indication. The firmware
   parks `inspect_sel` on PC and treats "PC low byte == expected" as the
   instruction-retired handshake, which covers the stall transparently.

3. **`spi_master` SCK phase bug (workaround in `spi_ram.c`).** SCK
   free-runs even while CS is high and is not forced low when a transfer
   starts, so with 50% probability the master's bit counter eats the
   first command bit and only 7 of the 8 command bits ever reach the
   wire. The emulator detects SCK-high at the CS falling edge and
   implies the missing MSB (both commands have bit7 = 0). **A real
   23LC1024 cannot do this and will fail half of all transfers.**
   Suggested RTL fix — hold SCK low while idle so every transaction
   starts in the driven phase, e.g. in `spi_master.v`:

   ```verilog
   always @(posedge clk or negedge rst_n) begin
       if (!rst_n)          sck <= 1'b0;
       else if (!busy)      sck <= 1'b0;   // park low between transfers
       else if (sck_edge)   sck <= ~sck;
   end
   ```

4. **Bracket-stack depth is 8.** Deeper nesting silently drops pushes;
   the loader warns at boot if the program nests deeper.

5. **`[` skip semantics.** On a taken forward skip the ASIC pushes its PC
   *before* waiting, and expects to be told the address **of** the
   matching `]` (not one past it) — executing that `]` with data==0 pops
   the placeholder. The firmware's match table does exactly this.
