"""The BF tab: program entry, session status, and output.

The panel is a small state machine, shown on one status line:
idle -> running (elapsed time + clock) or debugging -> waiting for
input -> done / stopped / failed. Controls appear only in the states
where they act. The app (app.py) drives the transitions.
"""

from __future__ import annotations

import time

from rich.text import Text
from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Input, Label, RichLog, Static, TextArea

BF_OPS = "+-<>[],."


def op_index(text: str, row: int, col: int) -> int | None:
    """Program index of the op at a cursor position: the count of BF
    ops before it in the text (comments do not count). None when the
    cursor is past the last op."""
    lines = text.split("\n")
    offset = sum(len(line) + 1 for line in lines[:row]) + col
    count = sum(1 for c in text[:offset] if c in BF_OPS)
    total = sum(1 for c in text if c in BF_OPS)
    return count if count < total else None


def fmt_hz(hz: int) -> str:
    if hz >= 1_000_000:
        return f"{hz / 1_000_000:g} MHz"
    if hz >= 1_000:
        return f"{hz / 1_000:g} kHz"
    return f"{hz} Hz"


def render_program(text: str, pc: int | None, breaks: set[int]) -> Text:
    """The program text with the next op and the breakpoints marked.
    Comments are dimmed. Pure, so tests can check the mapping."""
    out = Text()
    i = 0
    for ch in text:
        if ch in BF_OPS:
            if i == pc and i in breaks:
                style = "black on red3"
            elif i == pc:
                style = "black on green"
            elif i in breaks:
                style = "bold red3 underline"
            else:
                style = ""
            out.append(ch, style)
            i += 1
        else:
            out.append(ch, "dim")
    return out


class BfPanel(Vertical):
    """BF program entry and session output, as a tab. While a program
    runs, the firmware is in a raw byte stream and the command
    protocol is unavailable, so the Bench freezes until the run ends;
    between runs the Bench works as usual (inspect_sel included)."""

    BORDER_TITLE = "brainf*ck"

    _state = "idle"  # idle | running | debug | ended
    _waiting = False
    _t0 = 0.0
    _freq_hz = 0
    _outbuf = ""  # session bytes arrive in small chunks; line-buffer them

    def compose(self) -> ComposeResult:
        yield Button("Load the Brainf*ck ASIC first", id="bf-load")
        yield Static("", id="bf-status")
        yield Label("program: everything except + - < > [ ] , . is a "
                    "comment. Run appends '!'. End input makes the "
                    "next ',' read 0. Stop ends the run.",
                    classes="hint")
        yield TextArea(id="bf-program")
        yield Static("", id="bf-view")
        with Horizontal(id="bf-controls"):
            yield Button("▶  Run on ASIC", id="bf-run")
            yield Button("⏯  Debug", id="bf-debug")
            yield Input(placeholder="input for ',', sent raw on enter",
                        id="bf-stdin")
            yield Button("⏹ End input", id="bf-eof")
            yield Button("✖ Stop", id="bf-abort")
        with Horizontal(id="bf-dbg-controls"):
            yield Button("⏵ Step", id="bf-step")
            yield Button("▶ Continue", id="bf-cont")
            yield Static("", id="bf-dbg-state")
        with Horizontal(id="bf-bp-row"):
            yield Button("⏺ Break @ cursor", id="bf-break")
            yield Button("✕ Clear breaks", id="bf-bp-clear")
            yield Static("breaks: none", id="bf-bp-list")
        yield RichLog(id="bf-output", markup=False, wrap=True)

    def on_mount(self) -> None:
        self.query_one("#bf-load").display = False
        self._disclose(False, False)
        self._render_status()
        # One timer for both: flush partial output lines (prompts,
        # output with no newline) and advance the elapsed time.
        self.set_interval(0.25, self._tick)

    def _tick(self) -> None:
        self.flush_output()
        if self._state == "running" and not self._waiting:
            self._render_status()

    # -- program access --

    def program(self) -> str:
        return self.query_one("#bf-program", TextArea).text

    def op_index_at_cursor(self) -> int | None:
        area = self.query_one("#bf-program", TextArea)
        row, col = area.cursor_location
        return op_index(area.text, row, col)

    # -- state machine --

    def session_started(self, debug: bool, freq_hz: int) -> None:
        self._state = "debug" if debug else "running"
        self._waiting = False
        self._t0 = time.monotonic()
        self._freq_hz = freq_hz
        self._disclose(True, debug)
        self._render_status()

    def session_ended(self) -> None:
        self._state = "ended"
        self._waiting = False
        self._disclose(False, False)

    def waiting_input(self) -> None:
        """The program blocks on ',' with nothing typed yet."""
        self._waiting = True
        self._render_status()
        self.query_one("#bf-stdin", Input).focus()

    def input_sent(self) -> None:
        if self._waiting:
            self._waiting = False
            self._render_status()

    def set_needs_design(self, needs: bool, label: str) -> None:
        """Show the entry banner while the BF design is not loaded."""
        btn = self.query_one("#bf-load", Button)
        btn.label = label
        btn.display = needs
        self.query_one("#bf-run", Button).disabled = needs
        self.query_one("#bf-debug", Button).disabled = needs

    def _disclose(self, running: bool, debug: bool) -> None:
        """Show only the controls that act in the current state."""
        shown = {
            "#bf-run": not running,
            "#bf-debug": not running,
            "#bf-stdin": running,
            "#bf-eof": running,
            "#bf-abort": running,
            "#bf-dbg-controls": running and debug,
            "#bf-bp-row": not running or debug,
            "#bf-view": running and debug,
            "#bf-program": not (running and debug),
        }
        for wid, show in shown.items():
            self.query_one(wid).display = show
        self.query_one("#bf-program", TextArea).read_only = running
        # The debug view has no cursor, so breakpoints are set in the
        # editor before the session starts.
        self.query_one("#bf-break", Button).disabled = running

    def _render_status(self) -> None:
        status = self.query_one("#bf-status", Static)
        for cls in ("bf-run", "bf-wait", "bf-ok", "bf-err"):
            status.set_class(False, cls)
        if self._waiting:
            status.set_class(True, "bf-wait")
            status.update("⌨  waiting for ',' input — type below, "
                          "Enter sends, End input sends 0")
        elif self._state == "running":
            status.set_class(True, "bf-run")
            status.update(f"● running at {fmt_hz(self._freq_hz)} · "
                          f"{time.monotonic() - self._t0:.1f} s")
        elif self._state == "debug":
            status.set_class(True, "bf-run")
            status.update(f"● debugging at {fmt_hz(self._freq_hz)} — "
                          "green marks the next op, red a breakpoint")
        elif self._state == "idle":
            status.update("idle — write a program, then Run or Debug")

    def show_result(self, ok: bool, detail: str) -> None:
        status = self.query_one("#bf-status", Static)
        for cls in ("bf-run", "bf-wait"):
            status.set_class(False, cls)
        status.set_class(ok, "bf-ok")
        status.set_class(not ok, "bf-err")
        status.update(detail)

    # -- debug view --

    def show_view(self, pc: int | None, breaks: set[int]) -> None:
        self.query_one("#bf-view", Static).update(
            render_program(self.program(), pc, breaks))

    def show_dbg(self, fields: dict[str, str]) -> None:
        line = (f"pc {fields.get('pc', '?')} · next '{fields.get('op', '?')}'"
                f" · cell[{fields.get('vptr', '?')}] ="
                f" 0x{fields.get('data', '??')}"
                f" · bstack 0x{fields.get('bstk', '??')}"
                f" · executed {fields.get('exec', '?')}")
        self.query_one("#bf-dbg-state", Static).update(line)

    def show_breaks(self, breaks: set[int]) -> None:
        self.query_one("#bf-bp-list", Static).update(
            "breaks: " + (", ".join(str(b) for b in sorted(breaks))
                          if breaks else "none"))

    # -- output --

    def write_output(self, text: str) -> None:
        self._outbuf += text.replace("\r", "")
        while "\n" in self._outbuf:
            line, self._outbuf = self._outbuf.split("\n", 1)
            if line.strip() == "# input?":
                continue  # shown on the status line instead
            self.query_one("#bf-output", RichLog).write(line)

    def flush_output(self) -> None:
        """Show a partial line (a prompt, or output with no newline)."""
        if self._outbuf:
            self.query_one("#bf-output", RichLog).write(self._outbuf)
            self._outbuf = ""

    def clear_output(self) -> None:
        self._outbuf = ""
        self.query_one("#bf-output", RichLog).clear()
