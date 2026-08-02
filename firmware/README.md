# RP2350 host firmware

Runs on the Tiny Tapeout demo board v3 (RP2350B) and drives
`tt_um_brainfck_asic` — either real silicon or the ICE40UP5K ASIC-sim
carrier built by the `fpga` workflow.

## Built on the terminal-explorer kit

This firmware is an extension of the
[terminal-explorer kit](https://github.com/map588/tt_terminal_explorer),
pulled in as the `kit/` submodule. The kit core provides the command
protocol, the PIO clock, the pin control, and the carrier probe. The
whole BF integration is one file, `src/bf_ext.c`, which overrides
the kit's extension hooks (`kit/firmware/include/ext.h`): the
`bf`/`bfdbg` commands, the core-1 SPI RAM launch, the `bf=448` hello
field, serial timings that follow the clock, and the pin parking
that keeps core 1 off foreign designs.

The build makes two UF2s. Flash the one you need:

- **`bf_host`** (`src/bf_main.c`): standalone BF host with its own
  main. It boots straight into the paste-a-program loop: open the
  serial port, paste BF source, end with `!`. It selects design`include/bf_pins.h` | 448
  and fixes the clock at 200 kHz. There are no commands to learn.
  This is the debugging workhorse.
- **`tt_host`**: the kit's command firmware plus the BF extension.
  Select any shuttle design, set or single-step the clock, peek and
  poke pins, and run BF as a command. The `../explorer` TUI drives
  it.

Both share the BF engine (`src/bf_run.c`: feeds the program one
instruction at a time, mirrors the ASIC's PC, answers `interrupt_jump`
with the precomputed bracket-match table, bridges `,`/`.` to serial)
and run the SPI RAM tape slave on core 1 (`src/spi_ram.c`).

## bf_host

```
== tt_um_brainfck_asic host ==
ASIC clock 200000 Hz, design #448
# paste program, end with '!'
,+.,+.!AC
```

Anything that is not one of the eight BF ops is a comment; anything
after the `!` is consumed as `,` input by the running program.

## Silicon bugs and their firmware workarounds

Four RTL bugs were found on ttsky25b silicon (2026-08-01) by RTL
analysis plus step-mode pin traces. All four are worked around in
`src/bf_run.c`, and `Hello World!` runs on the chip.

1. **SPI cache refill never completes.** `STATE_SPI_WRITE/FETCH` in
   `bf_asic.v` re-issues its transaction forever: `transfer_done` and
   `!spi_busy` land on the same cycle, and the issue branch (which has
   no `!transfer_done` guard) wins. Trace: endless `02 00 00 ...`
   write transactions, CS parked low. *Workaround:* the host never
   feeds a pointer move that leaves the cache window; it virtualizes
   far moves through `.`/`,` and keeps the full 1024-cell tape in RAM.
2. **Phantom TX frames.** `bf_asic` clears `tx_start` one cycle after
   `serial_tx` returns to IDLE, so every ASIC→MCU frame repeats once.
   A tight `]`/`.` sequence collides with the phantom (this killed
   `[-]`). *Workaround:* the host drains the duplicate after every
   received frame.
3. **`]` jump double-pops the bracket stack.** The `WAIT_JUMP` start
   branch fires on two consecutive cycles before `tx_busy` rises;
   with two or more stack entries the ASIC pops both and its PC lands
   on the second (this killed nested loops). *Workaround:* the host
   models the double pop and feeds a synthetic `[` to push the lost
   entry back.
4. **Physical cell 4 has no storage.** The cache mux returns
   `data_current` for offset 0 and the save path drops the value when
   the pointer leaves `tape_base`; the cell reads as a copy of
   whatever the pointer carried in (this corrupted one letter of
   hello-world). *Workaround:* logical cell L maps to physical L for
   L < 4 and L + 1 for L ≥ 4; cell 4 is transit-only.

Separate MCU-side limit: the host's bit-banged sampling of the
ASIC→MCU link bit-slips at ASIC clocks ≥ 500 kHz. 200 kHz and below
is fully reliable — hence bf_host's fixed clock.

## tt_host command protocol

One command per line; each command gets exactly one reply line,
`ok [payload]` or `err <token>`. Informational lines start with `# `.
Type `help` on the port for the list. The `../explorer` TUI speaks
this protocol; a bare terminal (`tio`, `screen`) works too.

| Command | Effect |
|---|---|
| `hello` | `ok tt-explorer 2 shuttle=ttsky25b bf=448`: the kit protocol plus the BF extension field |
| `status` | design, clock mode/freq, pin state (`uidrv=0` when ui is released) |
| `freq <hz>` | free-running clock, 1 Hz – clk_sys/2 (75 MHz), made by PIO with one-sys-cycle resolution. The true output frequency never exceeds the request; the reply reports it. |
| `stop` / `step [n]` / `resume` | park the clock low, pulse it n times, restart PWM |
| `design <n>` | safe pin profile, mux-select design n, reset pulse |
| `reset [1\|0]` | pulse (no arg), assert, or release the project reset |
| `ui <hh>` / `ui off` / `ui` | drive ui_in, release it for the DIP switches / PMOD, or read the pad levels |
| `uo` / `uio` | read uo_out / uio pad levels (hex byte) |
| `uiod [hh]` / `uiow <hh>` | uio direction mask (1 = MCU drives) / output latch |
| `bf` | interactive BF session (BF design + running clock required) |
| `bfdbg` | BF debugger: same program load, then `n` = one instruction, `c` = run to the next breakpoint or the end, `b<index>` = toggle a breakpoint on a program index, `q` = stop. A `# dbg` state line (pc, next op, pointer, cell, bracket stack) follows each step. |

A `bf` session needs `design`include/bf_pins.h` | 448` and a running clock first, then
works like bf_host: paste BF source, end with `!`.

## Build

You need the pico-sdk (2.0 or newer, `PICO_SDK_PATH`), the Arm GNU
toolchain, cmake, and ninja. The Homebrew `arm-none-eabi-gcc` does
not work: it lacks newlib and fails on `nosys.specs`.

```sh
git submodule update --init firmware/kit   # once
cmake -S . -B build -G Ninja -DPICO_TOOLCHAIN_PATH=$HOME/.pico-sdk/toolchain/14_2_Rel1
cmake --build build
```

Flash `build/bf_host.uf2` or `build/tt_host.uf2` over BOOTSEL (or
`picotool load -f -x`). Both wait for the USB serial port to open.

Configuration knobs:

| Define | Where | Default | Notes |
|---|---|---|---|
| `BF_CLK_HZ` | `src/bf_main.c` | 200 kHz | bf_host clock; the silicon serial-link ceiling. |
| `BF_DESIGN_ADDR` | `include/bf_pins.h` | 448| 448 | tt_um_brainfck_asic mux slot on ttsky25b; the FPGA sim ignores the mux. |
| `BF_MIN_HZ` / `BF_MAX_HZ` | `src/bf_ext.c` | 50 kHz| 50 kHz / 2 MHz | tt_host `bf` refuses to run outside this window (bit-banged handshake limits). |
| `MAX_OPS` | `src/bf_run.c` | 1024 | Program size cap — the ASIC PC is 10 bits. |

For the v3 *Alpha* prototype board add `-DTT_DBV3_ALPHA` (different GPIO
map, see the kit tt_pins.h and `include/bf_pins.h`).

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
