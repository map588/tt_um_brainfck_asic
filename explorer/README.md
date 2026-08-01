# tt-explorer

Textual TUI to explore the Tiny Tapeout demo board through the
`firmware/` command protocol: browse the ttsky25b shuttle index,
select a design, control or single-step the project clock, and
peek/poke the ui/uo/uio pins. BF sessions on `tt_um_brainfck_asic`
run in a raw passthrough screen.

## Run

```sh
uv sync
uv run tt-explorer            # autodetects /dev/tty.usbmodem*
uv run tt-explorer --port /dev/tty.usbmodemXXXX
```

## Layout

Two tabs. "Projects" is the shuttle browser: filter, read the
description, press enter to load a design. "Bench" is the instrument
panel for the loaded design: the clock (running or single-step), one
labeled row per pin on all three buses with the design's own pin
names, a mirror of the board's 7-segment display, and the serial
console.

## Keys

- `s` — stop / resume the project clock
- `space` — one clock pulse (when stopped)
- `b` — open a BF session (BF design must be selected, clock running)
- `i` — refresh the shuttle index (cached in `~/.cache/tt-explorer/`)
- `q` — quit

## Test

```sh
uv run pytest
```
