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

## Keys

- `r` — refresh the shuttle index (cached in `~/.cache/tt-explorer/`)
- `b` — open a BF session (BF design must be selected, clock running)
- `q` — quit

## Test

```sh
uv run pytest
```
