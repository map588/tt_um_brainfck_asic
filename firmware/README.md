# RP2350 host firmware

Runs on the Tiny Tapeout demo board v3 (RP2350B) and drives
`tt_um_brainfck_asic`: either real silicon or the ICE40UP5K
ASIC-sim carrier built by the `fpga` workflow.

## Built on the terminal-explorer kit

This firmware is an extension of the
[terminal-explorer kit](https://github.com/map588/tt_terminal_explorer),
pulled in as the `kit/` submodule. The kit core provides the command
protocol, the PIO clock, the pin control, and the carrier probe. The
BF integration is `src/bf_ext.c`, which defines the kit's extension
hooks (`kit/firmware/include/ext.h`): the `bf`/`bfdbg` commands, the
core-1 engine launch, the `bf=448` hello field, and serial timings
that follow the clock.

The build makes one UF2, `tt_host`: select any shuttle design, set
or single-step the clock, peek and poke pins, and run BF as a
command. The `../explorer` TUI drives it, and a bare terminal
(`tio`, `screen`) works too.

The two RP2350 cores split the work. Core 1 runs the execution
engine (`src/bf_run.c`) with interrupts off: it feeds the program
one instruction at a time, mirrors the ASIC's PC, answers
`interrupt_jump` with the precomputed bracket-match table, and
drives the `,`/`.` serial links with deterministic timing. Core 0
runs USB and the session pump (`src/bf_session.c`): it parses the
program, moves input bytes and the Ctrl-C stop flag to the engine,
and renders engine events as text. The mailbox between them is the
SIO FIFO plus a shared struct (`include/bf_link.h`). Session output
bytes go out raw: a 0x0A cell prints as one byte, not CRLF.

`src/spi_ram.c` (the SPI RAM tape slave core 1 ran before it became
the engine) stays in the tree as reference and is not built: the
on-chip SPI path never completes a transaction on this silicon (bug
1 below), so the tape lives in MCU RAM.

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
is fully reliable, which is why the TUI sets 200 kHz for BF runs.

## tt_host command protocol

One command per line. Each command gets exactly one reply line,
`ok [payload]` or `err <token>`. Informational lines start with `# `.
Type `help` on the port for the list. The `../explorer` TUI
communicates over this protocol, and a bare terminal (`tio`,
`screen`) works too.

| Command | Effect |
|---|---|
| `hello` | `ok tt-explorer 2 shuttle=ttsky25b bf=448`: the kit protocol plus the BF extension field |
| `status` | design, clock mode/freq, pin state (`uidrv=0` when ui is released) |
| `freq <hz>` | free-running clock, 1 Hz to clk_sys/2 (75 MHz), made by PIO with one-sys-cycle resolution. The true output frequency never exceeds the request, and the reply reports it. |
| `stop` / `step [n]` / `resume` | park the clock low, pulse it n times, restart the clock |
| `design <n>` | safe pin profile, mux-select design n, reset pulse |
| `reset [1\|0]` | pulse (no arg), assert, or release the project reset. A pulse in step mode makes 10 clock edges while reset is low. |
| `ui <hh>` / `ui off` / `ui` | drive ui_in, release it for the DIP switches / PMOD, or read the pad levels |
| `uo` / `uio` | read uo_out / uio pad levels (hex byte) |
| `uiod [hh]` / `uiow <hh>` | uio direction mask (1 = MCU drives) / output latch |
| `bf` | interactive BF session (BF design + running clock required) |
| `bfdbg` | BF debugger: same program load, then `n` = one instruction, `c` = run to the next breakpoint or the end, `b<index>` = toggle a breakpoint on a program index, `q` = stop. A `# dbg` state line (pc, next op, pointer, cell, bracket stack) follows each step. |

A `bf` session needs `design 448` and a running clock first. Paste
BF source and end with `!`. Anything that is not one of the eight
BF ops is a comment. Anything after the `!` is consumed as `,`
input by the running program. Two control bytes work during a
session: Ctrl-D is end of input, so the next `,` reads 0. Ctrl-C
stops the session with `err stopped`.

## Build

You need the pico-sdk (2.0 or newer, `PICO_SDK_PATH`), the Arm GNU
toolchain, cmake, and ninja. The Homebrew `arm-none-eabi-gcc` does
not work: it lacks newlib and fails on `nosys.specs`.

```sh
git submodule update --init firmware/kit   # once
cmake -S . -B build -G Ninja -DPICO_TOOLCHAIN_PATH=$HOME/.pico-sdk/toolchain/14_2_Rel1
cmake --build build
```

Flash `build/tt_host.uf2` over BOOTSEL (or `picotool load -f -x`).
The firmware waits for the USB serial port to open.

Configuration knobs:

| Define | Where | Default | Notes |
|---|---|---|---|
| `BF_DESIGN_ADDR` | `include/bf_pins.h` | 448 | tt_um_brainfck_asic mux slot on ttsky25b. The FPGA sim ignores the mux. |
| `BF_MIN_HZ` / `BF_MAX_HZ` | `src/bf_ext.c` | 50 kHz / 2 MHz | `bf` refuses to run outside this window (bit-banged handshake limits). |
| `MAX_OPS` | `src/bf_run.c` | 1024 | Program size cap. The ASIC PC is 10 bits. |

For the v3 *Alpha* prototype board add `-DTT_DBV3_ALPHA` (different GPIO
map, see the kit tt_pins.h and `include/bf_pins.h`).

## Protocol notes (things the RTL made the firmware do)

1. **`instr_valid` width is critical.** The core executes on *every*
   rising clock edge where `instr_valid` is high, so a pulse wider than
   one clock period double-executes. The firmware owns the clock and
   raises/drops the pulse between two consecutive falling edges, so
   exactly one rising edge samples it high.

2. **SPI cache refills are invisible.** `irq_cache_pulse` exists in
   `bf_asic.v` but is never set, so a `<`/`>` that crosses the cache
   window stalls the core with no external indication. The firmware
   parks `inspect_sel` on PC and treats "PC low byte == expected" as the
   instruction-retired handshake, which covers the stall transparently.

3. **`spi_master` SCK phase bug (workaround in `spi_ram.c`, kept as
   reference).** SCK
   free-runs even while CS is high and is not forced low when a transfer
   starts, so with 50% probability the master's bit counter eats the
   first command bit and only 7 of the 8 command bits ever reach the
   wire. The emulator detects SCK-high at the CS falling edge and
   implies the missing MSB (both commands have bit7 = 0). **A real
   23LC1024 cannot do this and will fail half of all transfers.**
   Suggested RTL fix: hold SCK low while idle so every transaction
   starts in the driven phase, e.g. in `spi_master.v`:

   ```verilog
   always @(posedge clk or negedge rst_n) begin
       if (!rst_n)          sck <= 1'b0;
       else if (!busy)      sck <= 1'b0;   // park low between transfers
       else if (sck_edge)   sck <= ~sck;
   end
   ```

4. **Bracket-stack depth is 7.** The stack has 8 slots, but the RTL
   push guard (`bstack_ptr < 7`) stops one early, so 7 entries are
   usable. Deeper nesting silently drops pushes; the loader warns
   when a program nests past 7.

5. **`[` skip semantics.** On a taken forward skip the ASIC pushes its PC
   *before* waiting, and expects to be told the address **of** the
   matching `]` (not one past it): executing that `]` with data==0
   pops the placeholder. The firmware's match table does exactly
   this.
