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
Brainf*ck ASIC. A banner at the top loads the BF design when a
different design is active. One status line tracks the session:
idle, running (with the clock and the elapsed time), waiting for
',' input (the input field gets focus), and then a result line
with the instruction count and the rate. Only the controls that
act in the current state are visible: Run and Debug while idle,
Stop, "End input", and the input field while a program runs.
"End input" makes the next ',' read 0, so programs that read until
zero can finish. "Break @ cursor" toggles a breakpoint on the op
under the editor cursor before a Debug session starts. During
Debug, a read-only view of the program replaces the editor: green
marks the next op, red marks a breakpoint, and a state line shows
pc, cell, and bracket-stack details after each step. While a
session runs, the Bench and Signals tabs freeze and their tab
labels say so.

While the BF design is loaded, the pins its firmware host owns are
locked on the Bench (ui0-5 and all uio pins); inspect_sel (ui6/7)
stays usable between runs. Sessions run at the clock you set, 50 kHz
to 2 MHz; out-of-range clocks fall back to 200 kHz.
