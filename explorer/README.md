# bf-explorer

The terminal-explorer kit TUI extended with a Brainf*ck tab. The kit
package (`tt-explorer`) comes from the `firmware/kit` submodule, so
one submodule pin versions the firmware and the UI together. This
package holds only the BF extension: `BfExplorerApp` subclasses the
kit app through its extension hooks, and `BfPanel` is the BF tab.

## Run

```sh
uv sync
uv run bf-explorer            # autodetects /dev/tty.usbmodem*
uv run bf-explorer --port /dev/tty.usbmodemXXXX
```

## Layout

The kit tabs come as-is: "Projects" (shuttle browser), "Bench"
(clock, pin rows, serial console), and "Signals" (per-clock-edge
waveform capture). The "BF" tab is the program tab for the
Brainf*ck ASIC: type or paste a program, press Run, feed ',' input
through the field beside it, or press Debug to step the program one
instruction at a time with live pc / cell / bracket-stack state
between steps. "End input" makes the next ',' read 0, so programs
that read until zero can finish. "Stop" ends a stuck or endless run
at any time. "Break @ cursor" toggles a breakpoint on the op under
the editor cursor.

While the BF design is loaded, the pins its firmware host owns are
locked on the Bench (ui0-5 and all uio pins); inspect_sel (ui6/7)
stays usable between runs. Sessions run at the clock you set, 50 kHz
to 2 MHz; out-of-range clocks fall back to 200 kHz.
