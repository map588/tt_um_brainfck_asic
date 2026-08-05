"""The BF tab: program entry, session controls, and output."""

from __future__ import annotations

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


class BfPanel(Vertical):
    """BF program entry and session output, as a tab. While a program
    runs, the firmware is in a raw byte stream and the command
    protocol is unavailable, so the Bench freezes until the run ends;
    between runs the Bench works as usual (inspect_sel included)."""

    BORDER_TITLE = "brainf*ck"

    def compose(self) -> ComposeResult:
        yield Label("program: everything except + - < > [ ] , . is a "
                    "comment. Run appends '!'. End input makes the "
                    "next ',' read 0. Stop ends the run.",
                    classes="hint")
        yield TextArea(id="bf-program")
        with Horizontal(id="bf-controls"):
            yield Button("▶  Run on ASIC", id="bf-run")
            yield Button("⏯  Debug", id="bf-debug")
            yield Input(placeholder="input for ',', sent raw on enter",
                        id="bf-stdin", disabled=True)
            yield Button("⏹ End input", id="bf-eof", disabled=True)
            yield Button("✖ Stop", id="bf-abort", disabled=True)
            yield Static("", id="bf-state")
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
        self.query_one("#bf-dbg-controls").display = False
        # Session bytes arrive in chunks; show partial lines (prompts,
        # output with no newline) at the poll cadence. Outside a
        # session the buffer is empty and this is a no-op.
        self.set_interval(0.25, self.flush_output)

    def program(self) -> str:
        return self.query_one("#bf-program", TextArea).text

    def op_index_at_cursor(self) -> int | None:
        area = self.query_one("#bf-program", TextArea)
        row, col = area.cursor_location
        return op_index(area.text, row, col)

    def show_breaks(self, breaks: set[int]) -> None:
        self.query_one("#bf-bp-list", Static).update(
            "breaks: " + (", ".join(str(b) for b in sorted(breaks))
                          if breaks else "none"))

    def set_running(self, running: bool, debug: bool = False) -> None:
        self.query_one("#bf-run", Button).disabled = running
        self.query_one("#bf-debug", Button).disabled = running
        self.query_one("#bf-stdin", Input).disabled = not running
        self.query_one("#bf-eof", Button).disabled = not running
        self.query_one("#bf-abort", Button).disabled = not running
        self.query_one("#bf-dbg-controls").display = running and debug
        state = self.query_one("#bf-state", Static)
        if running:
            state.update("● stepping, Bench frozen until the session ends"
                         if debug else
                         "● running, Bench frozen until the program ends")
            state.set_class(True, "bf-running")

    def show_dbg(self, fields: dict[str, str]) -> None:
        line = (f"pc {fields.get('pc', '?')} · next '{fields.get('op', '?')}'"
                f" · cell[{fields.get('vptr', '?')}] ="
                f" 0x{fields.get('data', '??')}"
                f" · bstack 0x{fields.get('bstk', '??')}"
                f" · executed {fields.get('exec', '?')}")
        self.query_one("#bf-dbg-state", Static).update(line)

    def show_result(self, ok: bool, detail: str) -> None:
        state = self.query_one("#bf-state", Static)
        state.set_class(False, "bf-running")
        state.set_class(not ok, "bf-error")
        state.update(detail)

    _outbuf = ""  # session bytes arrive in small chunks; line-buffer them

    def write_output(self, text: str) -> None:
        self._outbuf += text.replace("\r", "")
        while "\n" in self._outbuf:
            line, self._outbuf = self._outbuf.split("\n", 1)
            self.query_one("#bf-output", RichLog).write(line)

    def flush_output(self) -> None:
        """Show a partial line (a prompt, or output with no newline)."""
        if self._outbuf:
            self.query_one("#bf-output", RichLog).write(self._outbuf)
            self._outbuf = ""

    def clear_output(self) -> None:
        self._outbuf = ""
        self.query_one("#bf-output", RichLog).clear()
