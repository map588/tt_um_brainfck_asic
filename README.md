![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Tiny Tapeout Brainf*ck ASIC

- [Project datasheet](docs/info.md)
- [RP2350 host firmware](firmware/README.md)
- [bf-explorer TUI](explorer/README.md)

## Run it

Flash `tt_host.uf2` from the Releases page onto the demo board.
Then either run the TUI (`uv sync && uv run bf-explorer` in
[explorer/](explorer/)), or use a bare serial terminal: `design 448`,
`freq 200000`, then `bf` and paste a program that ends with `!`.

## Silicon status (ttsky25b)

The chip runs Brainfuck programs on real silicon, at project clocks
from 50 kHz to 2 MHz. The compute core is correct. Four interface
bugs exist in the taped-out RTL (found 2026-08-01). The host
firmware compensates for all four, so no program change is
necessary. One silicon limit does reach the programs: the bracket
stack holds 7 usable entries, so loops must not nest deeper than 7.
The loader warns when a program does.

We found the bugs with the firmware's step mode. The host stops the
project clock, sends single clock pulses, and reads the chip pins
after each pulse. This gives a full trace of each transaction.

1. **The SPI cache refill does not complete.** The refill state
   machine in `bf_asic.v` starts its transaction again forever,
   because the exit branch can never win against the issue branch.
   The firmware does not use the SPI tape. It keeps the full
   1024-cell tape in MCU memory and moves cell values through the
   serial I/O paths.
2. **The serial transmitter sends each frame twice.** The core clears
   `tx_start` one cycle too late, so `serial_tx` starts again. The
   duplicate frame collides with the next handshake in tight loops.
   The firmware reads and discards the duplicate frame.
3. **A `]` jump removes two bracket-stack entries.** The jump branch
   fires on two consecutive cycles before `tx_busy` rises. Nested
   loops lose one stack entry, and the chip PC lands on the wrong
   target. The firmware models the double pop and sends a synthetic
   `[` to push the lost entry back.
4. **Physical tape cell 4 has no storage.** The cache multiplexer
   returns `data_current` for offset 0, and the save path drops the
   value. The firmware maps logical cells around cell 4 and uses the
   cell only for transit.

The firmware README lists the trace evidence and the workaround for
each bug.
