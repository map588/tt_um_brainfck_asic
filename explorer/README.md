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

Three tabs. "Projects" is the shuttle browser: filter, read the
description, press enter to load a design. "Bench" is the instrument
panel for the loaded design: the clock (running or single-step), one
labeled row per pin on all three buses with the design's own pin
names, a mirror of the board's 7-segment display, and the serial
console. "BF" is the program tab for the Brainf*ck ASIC: type or
paste a program, press Run, feed ',' input through the field beside
it, or press Debug to step the program one instruction at a time
with live pc / cell / bracket-stack state between steps. "Break @
cursor" toggles a breakpoint on the op under the editor cursor.
Continue then runs to the next breakpoint. While a program or
debug session runs, the firmware is in a raw byte stream and the
Bench freezes. Between runs the Bench is fully live, including the
inspect_sel pins (ui6/7), which stay unlocked for state
inspection. Pins the BF host owns (ui0-5 and the uio bus) are
locked whenever the BF design is loaded.

## Keys

- `s` : stop / resume the project clock
- `space` : one clock pulse (when stopped)
- `b` : go to the BF tab
- `i` : refresh the shuttle index (cached in `~/.cache/tt-explorer/`)
- `q` : quit

## Test

```sh
uv run pytest
```
