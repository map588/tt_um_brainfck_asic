# RP2350 host firmware

Runs on the Tiny Tapeout demo board v3 (RP2350B) and drives
`tt_um_brainfck_asic` — either real silicon or the ICE40UP5K ASIC-sim
carrier built by the `fpga` workflow.

## What it does

- **Core 0** — generates the ASIC clock (PWM on the demo board's project
  clock pin), feeds the program one instruction at a time, mirrors the
  ASIC's PC, answers `interrupt_jump` with the precomputed bracket-match
  table, and bridges `,` / `.` to USB CDC (open the board's serial port in
  any terminal).
- **Core 1** — bit-banged 23LC1024-style SPI RAM slave: the 64 KiB tape
  the ASIC pages its 9-byte cache against.

Programs are loaded at runtime over the same USB serial port: paste BF
source at the `bf>` prompt and end it with `!`. Anything that isn't one
of the eight BF ops is treated as a comment; anything typed after the
`!` is consumed as `,` input by the running program. When the program
halts the firmware resets the ASIC, zeroes the tape, and prompts again:

```
bf> paste program, end with '!'
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

Flash `build/bf_host.uf2` over BOOTSEL. The firmware waits for the USB
serial port to open, prints the program, runs it, and prints a halt
banner when the PC walks off the end.

Configuration knobs, all in `src/main.c`:

| Define | Default | Notes |
|---|---|---|
| `ASIC_CLK_HZ` | 1 MHz | Keep ≤ ~2 MHz — serial links and the `instr_valid` pulse are bit-banged against this clock. |
| `DESIGN_NUM` | 0 | Mux slot on real silicon (see the shuttle index); the FPGA sim ignores it. |
| `WAIT_FOR_USB` | 1 | Hold boot until a terminal attaches. |
| `MAX_OPS` | 1024 | Program size cap — the ASIC PC is 10 bits. |

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
