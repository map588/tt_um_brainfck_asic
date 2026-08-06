"""bf-explorer: the kit TUI (tt_explorer) plus a Brainf*ck tab.

BfExplorerApp subclasses the kit app and fills its extension hooks;
everything else, including future kit features, is inherited. The
kit package comes from the firmware/kit submodule.
"""

from __future__ import annotations

import argparse
import os
import re
import time

from textual.widgets import Button, Input, TabbedContent, TabPane

from tt_explorer.app import TTExplorerApp
from tt_explorer.index import Project
from tt_explorer.widgets import (
    ClockPanel,
    ProjectList,
    TracePanel,
    UiPanel,
    UioPanel,
    UoPanel,
)

from .widgets import BfPanel, fmt_hz

BF_END = re.compile(r"(?:^|\n)(?:ok (done)|err ([a-z-]+))\s*$")
BF_HALTED = re.compile(r"# halted: (\d+) instructions")

# The firmware's session bounds (bf_ext.c). The whole range is
# hardware-verified; 200 kHz is the safe default when the clock is
# out of range.
BF_MIN_HZ = 50_000
BF_MAX_HZ = 2_000_000
BF_SAFE_HZ = 200_000

BF_CSS = """
    /* bf tab */
    BfPanel { padding: 0 1; }
    BfPanel Button { border: none; height: 1; margin-right: 2; }
    #bf-load { background: $warning-darken-2; margin-bottom: 1; }
    #bf-status { height: 1; margin-bottom: 1; color: $text-muted; }
    #bf-status.bf-run { color: $success; text-style: bold; }
    #bf-status.bf-wait { color: $warning; text-style: bold; }
    #bf-status.bf-ok { color: $success; text-style: bold; }
    #bf-status.bf-err { color: $warning; text-style: bold; }
    #bf-program { height: 10; }
    #bf-view { height: 10; border: round $primary; padding: 0 1; }
    #bf-controls { height: 1; margin: 1 0; }
    #bf-run { border: none; height: 1; background: $success-darken-2; }
    #bf-stdin {
        width: 40; height: 1; border: none;
        padding: 0 1; background: $boost;
    }
    #bf-stdin:focus { background: $primary-darken-2; }
    #bf-dbg-controls { height: 1; margin-bottom: 1; }
    #bf-bp-row { height: 1; margin-bottom: 1; }
    #bf-break { background: $error-darken-2; }
    #bf-bp-clear { background: $panel-lighten-1; }
    #bf-bp-list { margin-left: 2; color: $text-muted; }
    #bf-step { background: $primary-darken-1; }
    #bf-cont { background: $success-darken-2; }
    #bf-abort { background: $warning-darken-2; }
    #bf-eof { background: $panel-lighten-1; margin-left: 2; }
    #bf-dbg-state { margin-left: 2; color: $success; text-style: bold; }
    #bf-output { height: 1fr; }
"""


class BfExplorerApp(TTExplorerApp):
    TITLE = "bf-explorer"

    CSS = TTExplorerApp.CSS + BF_CSS

    BINDINGS = [("b", "bf", "BF session")]

    def __init__(self, shuttle: str = "ttsky25b",
                 port: str | None = None) -> None:
        super().__init__(shuttle=shuttle, port=port)
        self._bf_addr: int | None = None
        self._bf_active = False
        self._bf_debug = False
        self._bf_tail = ""
        self._bf_linebuf = ""
        self._bf_end: tuple[bool, str] | None = None
        self._bf_breaks: set[int] = set()
        self._bf_ready: bool | None = None  # None forces the first update
        self._bf_pc = 0
        self._bf_halted: int | None = None
        self._bf_t0 = 0.0
        self._bf_waiting = False
        self._bf_wait_t0 = 0.0
        self._bf_wait = 0.0  # total time spent waiting for ',' input
        self._bf_t_end = 0.0
        self._bench_label = None

    # -- kit extension hooks --

    def extension_tabs(self):
        yield TabPane("BF", BfPanel(), id="tab-bf")

    def on_hello(self, hello: dict) -> None:
        if "bf" in hello:
            self._bf_addr = int(hello["bf"])
        else:
            self._log("! this firmware has no BF extension "
                      "(no bf= in hello)")

    def on_design_loaded(self, p: Project) -> None:
        if p.address == self._bf_addr:
            # The BF host owns instr/valid/rx (ui0-5) and the SPI
            # pins; ui6/7 stay free: they are inspect_sel, useful
            # between runs.
            self.query_one(UiPanel).lock_bits(0x3F)
            self.query_one(UioPanel).lock_bits(0xFF)
        else:
            self.query_one(UiPanel).lock_bits(0x00)
            self.query_one(UioPanel).lock_bits(0x00)

    def design_clock_cap(self, p: Project) -> int | None:
        return BF_MAX_HZ if p.address == self._bf_addr else None

    # -- session end and readiness watcher --

    async def on_mount(self) -> None:
        await super().on_mount()
        self.set_interval(0.25, self._bf_tick)

    async def _bf_tick(self) -> None:
        ready = self._bf_addr is not None and self._design == self._bf_addr
        if ready != self._bf_ready:
            self._bf_ready = ready
            if self._bf_addr is not None:
                label = f"Load the Brainf*ck ASIC (design {self._bf_addr})"
            else:
                label = "this firmware has no BF extension"
            panel = self.query_one(BfPanel)
            panel.set_needs_design(not ready, label)
            panel.query_one("#bf-load", Button).disabled = \
                self._bf_addr is None
        if self._bf_end is not None:
            await self._bf_finish()

    # -- BF sessions --

    def action_bf(self) -> None:
        self.query_one(TabbedContent).active = "tab-bf"
        self.query_one("#bf-program").focus()

    def _set_bench_frozen(self, frozen: bool) -> None:
        for widget in (self.query_one(ClockPanel), self.query_one(UiPanel),
                       self.query_one(UoPanel), self.query_one(UioPanel),
                       self.query_one(TracePanel)):
            widget.disabled = frozen
        self.query_one("#console-input", Input).disabled = frozen
        # Say WHY the tabs are dead, on the tabs themselves.
        tabs = self.query_one(TabbedContent)
        bench = tabs.get_tab("tab-bench")
        signals = tabs.get_tab("tab-signals")
        if frozen:
            self._bench_label = bench.label
            bench.label = "Bench ⏸ BF session"
            signals.label = "Signals ⏸ BF session"
        else:
            if self._bench_label is not None:
                bench.label = self._bench_label
                self._bench_label = None
            signals.label = "Signals"

    async def _bf_load_design(self) -> None:
        """The entry banner: load the BF design, stay on the BF tab."""
        if self._bf_addr is None:
            return
        pl = self.query_one(ProjectList)
        p = pl._by_address.get(self._bf_addr) or Project(
            macro=f"design {self._bf_addr}", address=self._bf_addr,
            title=f"design {self._bf_addr}")
        await self._load_design(p)
        self.query_one(TabbedContent).active = "tab-bf"
        self.query_one("#bf-program").focus()

    async def _bf_start(self, debug: bool = False) -> None:
        if self.link is None or self._bf_active:
            return
        if self._design != self._bf_addr:
            return  # Run and Debug are disabled then; belt and braces
        panel = self.query_one(BfPanel)
        if not self._clk_running or not BF_MIN_HZ <= self._freq <= BF_MAX_HZ:
            self._log("# BF needs a running clock at 50 kHz - 2 MHz — "
                      "setting 200 kHz")
            reply = await self._clock_send(f"freq {BF_SAFE_HZ}")
            if not (reply and reply.ok):
                panel.show_result(False, "cannot set the clock — see console")
                return
            await self._refresh_status()
        program = panel.program()
        if "!" not in program:
            program += "!"
        panel.clear_output()
        self._set_bench_frozen(True)
        self._bf_tail = ""
        self._bf_linebuf = ""
        self._bf_end = None
        self._bf_pc = 0
        self._bf_halted = None
        self._bf_t0 = time.monotonic()
        self._bf_waiting = False
        self._bf_wait = 0.0
        self._bf_active = True
        self._bf_debug = debug
        panel.session_started(debug, self._freq)
        if debug:
            panel.show_view(0, self._bf_breaks)
        await self.link.begin_raw(self._on_bf_chunk)
        self.link.write_raw("bfdbg\n" if debug else "bf\n")
        self.link.write_raw(program)
        if debug:
            for n in sorted(self._bf_breaks):
                self.link.write_raw(f"b{n}\n")

    def _on_bf_chunk(self, text: str) -> None:
        panel = self.query_one(BfPanel)
        panel.write_output(text)
        self._bf_linebuf += text
        while "\n" in self._bf_linebuf:
            line, self._bf_linebuf = self._bf_linebuf.split("\n", 1)
            line = line.strip()
            if line == "# input?":
                if not self._bf_waiting:
                    self._bf_waiting = True
                    self._bf_wait_t0 = time.monotonic()
                panel.waiting_input()
            elif line.startswith("# dbg pc="):
                fields = dict(part.split("=", 1)
                              for part in line[6:].split()
                              if "=" in part)
                panel.show_dbg(fields)
                try:
                    self._bf_pc = int(fields["pc"])
                except (KeyError, ValueError):
                    pass
                panel.show_view(self._bf_pc, self._bf_breaks)
            else:
                m = BF_HALTED.search(line)
                if m:
                    self._bf_halted = int(m.group(1))
        self._bf_tail = (self._bf_tail + text)[-96:]
        m = BF_END.search(self._bf_tail.replace("\r", ""))
        if m:
            self._bf_end = (bool(m.group(1)), m.group(2) or "done")
            self._bf_t_end = time.monotonic()

    def _bf_input_sent(self) -> None:
        """Input reached the program: leave the waiting state and
        book the waited time, so the rate counts execution only."""
        if self._bf_waiting:
            self._bf_waiting = False
            self._bf_wait += time.monotonic() - self._bf_wait_t0
        self.query_one(BfPanel).input_sent()

    async def _bf_finish(self) -> None:
        ok, token = self._bf_end
        self._bf_end = None
        self._bf_active = False
        if self.link:
            self.link.set_raw_sink(None)
        self._set_bench_frozen(False)
        panel = self.query_one(BfPanel)
        panel.flush_output()
        panel.session_ended()
        self._bf_input_sent()
        busy = max(self._bf_t_end - self._bf_t0 - self._bf_wait, 1e-6)
        n = self._bf_halted
        if ok and n is not None and not self._bf_debug:
            detail = (f"✓ done — {n} instructions in {busy:.2f} s"
                      f" = {n / busy:,.0f}/s at {fmt_hz(self._freq)}")
            if self._bf_wait >= 0.1:
                detail += f" (+{self._bf_wait:.1f} s waiting for input)"
        elif ok and n is not None:
            detail = f"✓ done — {n} instructions"
        elif ok:
            detail = "✓ debug session ended"
        elif token == "stopped":
            detail = (f"■ stopped after "
                      f"{time.monotonic() - self._bf_t0:.1f} s")
        else:
            detail = f"✗ failed: {token} — see the output above"
        panel.show_result(ok, detail)
        await self._refresh_status()

    # -- UI events: bf ids here, everything else to the kit --

    async def on_button_pressed(self, event) -> None:
        bid = event.button.id or ""
        if bid == "bf-run":
            await self._bf_start()
        elif bid == "bf-debug":
            await self._bf_start(debug=True)
        elif bid == "bf-load":
            await self._bf_load_design()
        elif bid in ("bf-step", "bf-cont") and self._bf_active:
            if self.link:
                self.link.write_raw({"bf-step": "n", "bf-cont": "c"}[bid])
        elif bid == "bf-abort" and self._bf_active:
            if self.link:
                self.link.write_raw("\x03")  # Ctrl-C: stop the session
        elif bid == "bf-eof" and self._bf_active:
            if self.link:
                self.link.write_raw("\x04")  # Ctrl-D: ',' reads 0
                self._bf_input_sent()
        elif bid == "bf-break":
            panel = self.query_one(BfPanel)
            n = panel.op_index_at_cursor()
            if n is None:
                return
            if n in self._bf_breaks:
                self._bf_breaks.discard(n)
            else:
                self._bf_breaks.add(n)
            panel.show_breaks(self._bf_breaks)
        elif bid == "bf-bp-clear":
            if self._bf_active and self._bf_debug and self.link:
                for n in self._bf_breaks:
                    self.link.write_raw(f"b{n}\n")
            self._bf_breaks.clear()
            panel = self.query_one(BfPanel)
            panel.show_breaks(self._bf_breaks)
            if self._bf_active and self._bf_debug:
                panel.show_view(self._bf_pc, self._bf_breaks)
        else:
            await super().on_button_pressed(event)

    async def on_input_submitted(self, event) -> None:
        if event.input.id == "bf-stdin":
            if self._bf_active and event.value and self.link:
                self.link.write_raw(event.value)
                event.input.value = ""
                self._bf_input_sent()
            return
        await super().on_input_submitted(event)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Tiny Tapeout board explorer with a BF tab")
    parser.add_argument("--shuttle",
                        default=os.environ.get("TT_SHUTTLE", "ttsky25b"),
                        help="shuttle run the chip is from "
                             "(default: ttsky25b)")
    parser.add_argument("--port", help="serial device (default: autodetect)")
    args = parser.parse_args()
    BfExplorerApp(shuttle=args.shuttle, port=args.port).run()


if __name__ == "__main__":
    main()
