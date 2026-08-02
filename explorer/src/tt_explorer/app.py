"""tt-explorer: Textual TUI for the tt-explorer firmware."""

from __future__ import annotations

import argparse
import asyncio

from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import (
    Button,
    Footer,
    Header,
    Input,
    TabbedContent,
    TabPane,
)

import re

from . import index, protocol
from .index import Project
from .serial_link import SerialLink, find_ports
from .widgets import (
    BfPanel,
    ClockPanel,
    ConsolePane,
    CycleButton,
    DetailPane,
    ProjectList,
    UiPanel,
    UioPanel,
    UoPanel,
)

MAX_HZ = 75_000_000  # clk_sys / 2, the firmware PWM ceiling
BF_END = re.compile(r"(?:^|\n)(?:ok (done)|err ([a-z-]+))\s*$")
BF_MAX_HZ = 200_000  # ASIC->MCU serial link bit-slips above this


def parse_hz(text: str) -> int | None:
    """'440' -> 440, '32k' -> 32000, '1.5M' -> 1500000."""
    text = text.strip().replace(" ", "").removesuffix("Hz").removesuffix("hz")
    scale = 1
    if text and text[-1] in "kK":
        scale, text = 1_000, text[:-1]
    elif text and text[-1] in "mM":
        scale, text = 1_000_000, text[:-1]
    try:
        value = float(text)
    except ValueError:
        return None
    hz = int(round(value * scale))
    return hz if hz > 0 else None


class TTExplorerApp(App):
    TITLE = "tt-explorer"

    CSS = """
    /* projects tab */
    ProjectList { width: 45%; }
    #project-table { height: 1fr; }
    DetailPane { width: 55%; border: round $primary; padding: 0 1; }

    /* bench tab */
    ClockPanel { height: auto; border: round $secondary; padding: 0 1; }
    #clk-run, #clk-step { height: auto; }
    .clk-line { height: 1; margin-bottom: 1; }
    #clk-run-state { width: 12; color: $success; text-style: bold; }
    #clk-step-state { width: 32; color: $warning; text-style: bold; }
    #clk-run-freq { width: 16; text-style: bold; }
    #step-total { width: 20; color: $text-muted; }
    #clk-error { color: $error; height: 1; }
    ClockPanel Button { border: none; height: 1; margin-right: 2; }
    #clk-stop { background: $warning-darken-2; }
    #clk-resume { background: $success-darken-2; }
    .preset { background: $panel-lighten-1; }
    #freq-input {
        width: 16; height: 1; border: none;
        padding: 0 1; background: $boost;
    }
    #freq-input:focus { background: $primary-darken-2; }
    #freq-preview { width: 22; margin: 0 1; color: $success; }
    #freq-preview.preview-bad { color: $warning; }
    .hint { color: $text-muted; }

    #buses { height: auto; }
    UiPanel, UoPanel, UioPanel {
        width: 1fr; height: auto;
        border: round $primary; padding: 0 1; margin-right: 1;
    }
    .bus-head { height: 1; margin-bottom: 1; }
    .pin-row { height: 1; }
    .pin-bit { width: 2; color: $text-muted; }
    .pin-lvl { width: 2; }
    .pin-name { width: 1fr; color: $text; }
    .pin-btn { margin-right: 1; }
    CycleButton { border: none; height: 1; min-width: 5; }
    .cyc-low { background: $panel-lighten-2; }
    .cyc-high { background: $success-darken-1; }
    .cyc-listen { background: $panel; color: $text-muted; }
    .cyc-mcu { background: $primary-darken-1; }
    .cyc-ext { background: $warning-darken-2; }
    #sevenseg-row { height: 3; margin-top: 1; }
    #sevenseg { width: 6; text-style: bold; color: $error; }
    #uo-hex { color: $text-muted; }

    ConsolePane { height: 1fr; border: round $accent; }
    #console-log { height: 1fr; }

    /* bf tab */
    BfPanel { padding: 0 1; }
    BfPanel Button { border: none; height: 1; margin-right: 2; }
    #bf-program { height: 10; }
    #bf-controls { height: 1; margin: 1 0; }
    #bf-run { border: none; height: 1; background: $success-darken-2; }
    #bf-stdin {
        width: 40; height: 1; border: none;
        padding: 0 1; margin-left: 2; background: $boost;
    }
    #bf-stdin:focus { background: $primary-darken-2; }
    #bf-state { margin-left: 2; color: $text-muted; }
    #bf-dbg-controls { height: 1; margin-bottom: 1; }
    #bf-bp-row { height: 1; margin-bottom: 1; }
    #bf-break { background: $error-darken-2; }
    #bf-bp-clear { background: $panel-lighten-1; }
    #bf-bp-list { margin-left: 2; color: $text-muted; }
    #bf-step { background: $primary-darken-1; }
    #bf-cont { background: $success-darken-2; }
    #bf-abort { background: $warning-darken-2; }
    #bf-dbg-state { margin-left: 2; color: $success; text-style: bold; }
    #bf-state.bf-running { color: $success; }
    #bf-state.bf-error { color: $warning; text-style: bold; }
    #bf-output { height: 1fr; }
    """

    BINDINGS = [
        ("q", "quit", "quit"),
        ("i", "refresh_index", "refresh index"),
        ("b", "bf", "BF session"),
        ("s", "toggle_clock", "stop/resume"),
        ("space", "step_one", "step ×1"),
    ]

    def __init__(self, port: str | None = None) -> None:
        super().__init__()
        self._port_arg = port
        self.link: SerialLink | None = None
        self._bf_addr: int | None = None
        self._design: int | None = None
        self._freq = 0
        self._ui_driving = True
        self._clk_running = True
        self._steps = 0
        self._bf_active = False
        self._bf_debug = False
        self._bf_tail = ""
        self._bf_linebuf = ""
        self._bf_end: tuple[bool, str] | None = None
        self._bf_breaks: set[int] = set()

    def compose(self) -> ComposeResult:
        yield Header()
        with TabbedContent(initial="tab-projects"):
            with TabPane("Projects", id="tab-projects"):
                with Horizontal():
                    yield ProjectList()
                    yield DetailPane()
            with TabPane("Bench", id="tab-bench"):
                with Vertical():
                    yield ClockPanel()
                    with Horizontal(id="buses"):
                        yield UiPanel()
                        yield UoPanel()
                        yield UioPanel()
                    yield ConsolePane()
            with TabPane("BF", id="tab-bf"):
                yield BfPanel()
        yield Footer()

    # -- startup --

    async def on_mount(self) -> None:
        self.load_projects(refresh=False)
        await self._connect()
        self.set_interval(0.25, self._poll)

    @work(thread=True)
    def load_projects(self, refresh: bool) -> None:
        try:
            projects = index.load_index(refresh=refresh)
        except OSError as exc:
            self.call_from_thread(self._log, f"! index fetch failed: {exc}")
            return
        self.call_from_thread(
            self.query_one(ProjectList).set_projects, projects)
        self.call_from_thread(
            self._log, f"# index: {len(projects)} projects")

    async def _connect(self) -> None:
        ports = [self._port_arg] if self._port_arg else find_ports()
        if not ports:
            self._log("! no serial port found — is the board plugged in?")
            return
        try:
            self.link = SerialLink(ports[0], on_line=self._log)
        except OSError as exc:
            self._log(f"! cannot open {ports[0]}: {exc}")
            return
        self._log(f"# connected to {ports[0]}")
        reply = await self.send("hello")
        if reply and reply.ok:
            hello = protocol.parse_hello(reply.payload)
            self._bf_addr = hello["bf"]
            self.sub_title = f"{self.link.port} · fw v{hello['version']}"
        await self._refresh_status()

    # -- command plumbing --

    async def send(self, cmd: str) -> protocol.Reply | None:
        if self.link is None:
            self._log("! not connected")
            return None
        self._log(f"> {cmd}")
        try:
            reply = await self.link.request(cmd)
        except asyncio.TimeoutError:
            self._log(f"! timeout waiting for reply to {cmd!r}")
            return None
        except OSError as exc:
            self._log(f"! serial error: {exc} — is the board still plugged in?")
            return None
        prefix = "ok" if reply.ok else "err"
        self._log(f"{prefix} {reply.payload}".rstrip())
        return reply

    async def _clock_send(self, cmd: str) -> protocol.Reply | None:
        """A clock command with its error shown inline on the panel."""
        reply = await self.send(cmd)
        clock = self.query_one(ClockPanel)
        if reply is None:
            clock.set_error(f"no reply to '{cmd}'")
        elif not reply.ok:
            clock.set_error(f"'{cmd}' failed: {reply.payload}")
        else:
            clock.set_error("")
        return reply

    def _log(self, line: str) -> None:
        self.query_one(ConsolePane).log_line(line)

    async def _refresh_status(self) -> None:
        reply = await self.send("status")
        if not (reply and reply.ok):
            return
        st = protocol.parse_status(reply.payload)
        self._clk_running = st["mode"] == "run"
        self._freq = st["freq"]
        if st["design"] >= 0:
            self._design = st["design"]
        self.query_one(ClockPanel).show_mode(st["mode"], st["freq"])
        if "uidrv" in st:
            self._ui_driving = bool(st["uidrv"])
            self.query_one(UiPanel).set_bus(self._ui_driving)

    async def _poll(self) -> None:
        if self._bf_end is not None:
            await self._bf_finish()
        link = self.link
        if link is None or link.busy or link.raw:
            return
        try:
            reply = await link.request("uo", timeout=1.0)
            if reply.ok:
                self.query_one(UoPanel).show(
                    protocol.parse_hex_byte(reply.payload))
            reply = await link.request("uio", timeout=1.0)
            if reply.ok:
                self.query_one(UioPanel).show(
                    protocol.parse_hex_byte(reply.payload))
            if not self._ui_driving:
                reply = await link.request("ui", timeout=1.0)
                if reply.ok:
                    self.query_one(UiPanel).show_levels(
                        protocol.parse_hex_byte(reply.payload))
        except (asyncio.TimeoutError, ValueError, OSError):
            pass

    # -- actions --

    def action_refresh_index(self) -> None:
        self.load_projects(refresh=True)

    def action_bf(self) -> None:
        self.query_one(TabbedContent).active = "tab-bf"
        self.query_one("#bf-program").focus()

    # -- BF sessions --

    def _set_bench_frozen(self, frozen: bool) -> None:
        for widget in (self.query_one(ClockPanel), self.query_one(UiPanel),
                       self.query_one(UoPanel), self.query_one(UioPanel)):
            widget.disabled = frozen
        self.query_one("#console-input", Input).disabled = frozen

    async def _bf_start(self, debug: bool = False) -> None:
        if self.link is None or self._bf_active:
            return
        panel = self.query_one(BfPanel)
        if self._design != self._bf_addr:
            panel.show_result(False, "load the Brainf*ck ASIC on the "
                                     "Projects tab first")
            return
        if not self._clk_running or not 50_000 <= self._freq <= BF_MAX_HZ:
            self._log("# BF needs a running clock at 50-200 kHz — "
                      "setting 200 kHz")
            reply = await self._clock_send("freq 200000")
            if not (reply and reply.ok):
                panel.show_result(False, "cannot set the clock — see console")
                return
            await self._refresh_status()
        program = panel.program()
        if "!" not in program:
            program += "!"
        panel.clear_output()
        panel.set_running(True, debug)
        self._set_bench_frozen(True)
        self._bf_tail = ""
        self._bf_linebuf = ""
        self._bf_end = None
        self._bf_active = True
        self._bf_debug = debug
        self.link.set_raw_sink(self._on_bf_chunk)
        self.link.write_raw("bfdbg\n" if debug else "bf\n")
        self.link.write_raw(program)
        if debug:
            for n in sorted(self._bf_breaks):
                self.link.write_raw(f"b{n}\n")

    def _on_bf_chunk(self, text: str) -> None:
        panel = self.query_one(BfPanel)
        panel.write_output(text)
        if self._bf_debug:
            self._bf_linebuf += text
            while "\n" in self._bf_linebuf:
                line, self._bf_linebuf = self._bf_linebuf.split("\n", 1)
                line = line.strip()
                if line.startswith("# dbg pc="):
                    fields = dict(part.split("=", 1)
                                  for part in line[6:].split()
                                  if "=" in part)
                    panel.show_dbg(fields)
        self._bf_tail = (self._bf_tail + text)[-96:]
        m = BF_END.search(self._bf_tail.replace("\r", ""))
        if m:
            self._bf_end = (bool(m.group(1)), m.group(2) or "done")

    async def _bf_finish(self) -> None:
        ok, token = self._bf_end
        self._bf_end = None
        self._bf_active = False
        if self.link:
            self.link.set_raw_sink(None)
        self._set_bench_frozen(False)
        panel = self.query_one(BfPanel)
        panel.set_running(False)
        panel.show_result(ok, "✓ done — Bench live again" if ok
                          else f"failed: {token} — see output above")
        await self._refresh_status()

    async def action_toggle_clock(self) -> None:
        if self._clk_running:
            await self._stop_clock()
        else:
            await self._clock_send("resume")
            await self._refresh_status()

    async def action_step_one(self) -> None:
        if not self._clk_running:
            await self._do_step(1)

    async def _stop_clock(self) -> None:
        reply = await self._clock_send("stop")
        if reply and reply.ok:
            self._steps = 0
            self.query_one(ClockPanel).set_steps(0)
        await self._refresh_status()

    async def _set_freq_from_input(self) -> None:
        text = self.query_one("#freq-input", Input).value
        hz = parse_hz(text)
        if hz is None:
            self.query_one(ClockPanel).set_error(
                f"cannot read {text!r} — try 440, 32k, or 1.5M")
            return
        await self._clock_send(f"freq {hz}")
        await self._refresh_status()

    async def _do_step(self, n: int) -> None:
        reply = await self._clock_send(f"step {n}")
        if reply and reply.ok:
            self._steps += n
            self.query_one(ClockPanel).set_steps(self._steps)

    # -- UI events --

    async def on_project_list_selected(self, event: ProjectList.Selected) -> None:
        p: Project = event.project
        self.query_one(DetailPane).show(p)
        reply = await self.send(f"design {p.address}")
        if not (reply and reply.ok):
            return
        self.query_one(UiPanel).set_names(p.pinout)
        self.query_one(UoPanel).set_names(p.pinout)
        self.query_one(UioPanel).set_names(p.pinout)
        self.query_one(UiPanel).reset()
        self.query_one(UioPanel).reset()
        if p.address == self._bf_addr:
            # the BF host owns instr/valid/rx (ui0-5) and the SPI pins;
            # ui6/7 stay free: they are inspect_sel, useful between runs.
            self.query_one(UiPanel).lock_bits(0x3F)
            self.query_one(UioPanel).lock_bits(0xFF)
        else:
            self.query_one(UiPanel).lock_bits(0x00)
            self.query_one(UioPanel).lock_bits(0x00)
        self._ui_driving = True
        if p.clock_hz:
            cap = BF_MAX_HZ if p.address == self._bf_addr else MAX_HZ
            await self._clock_send(f"freq {min(p.clock_hz, cap)}")
        await self._refresh_status()
        tabs = self.query_one(TabbedContent)
        tabs.get_tab("tab-bench").label = f"Bench · {p.title or p.macro}"
        tabs.active = "tab-bench"
        self.set_focus(None)  # so keys like 'b' and space work at once

    async def on_cycle_button_cycled(self, event: CycleButton.Cycled) -> None:
        bid = event.button.id or ""
        if bid == "ui-bus":
            ui = self.query_one(UiPanel)
            if event.state == "ext":
                reply = await self.send("ui off")
                if reply and reply.ok:
                    self._ui_driving = False
                    ui.set_bus(False)
                else:
                    ui.set_bus(True)
            else:
                reply = await self.send(f"ui {protocol.hex_byte(ui.byte())}")
                if reply and reply.ok:
                    self._ui_driving = True
                    ui.set_bus(True)
                else:
                    ui.set_bus(False)
        elif bid.startswith("uio"):
            panel = self.query_one(UioPanel)
            # value latch first, so a newly-driven pin never glitches
            await self.send(f"uiow {protocol.hex_byte(panel.value())}")
            await self.send(f"uiod {protocol.hex_byte(panel.mask())}")
        elif bid.startswith("ui") and self._ui_driving:
            byte = self.query_one(UiPanel).byte()
            await self.send(f"ui {protocol.hex_byte(byte)}")

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id or ""
        if bid == "clk-stop":
            await self._stop_clock()
        elif bid == "clk-resume":
            await self._clock_send("resume")
            await self._refresh_status()
        elif bid == "freq-set":
            await self._set_freq_from_input()
        elif bid.startswith("preset-"):
            await self._clock_send(f"freq {bid.removeprefix('preset-')}")
            await self._refresh_status()
        elif bid.startswith("step-"):
            await self._do_step(int(bid.removeprefix("step-")))
        elif bid == "bf-run":
            await self._bf_start()
        elif bid == "bf-debug":
            await self._bf_start(debug=True)
        elif bid in ("bf-step", "bf-cont", "bf-abort") and self._bf_active:
            if self.link:
                self.link.write_raw(
                    {"bf-step": "n", "bf-cont": "c", "bf-abort": "q"}[bid])
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
            if self._bf_active and self._bf_debug and self.link:
                self.link.write_raw(f"b{n}\n")
        elif bid == "bf-bp-clear":
            if self._bf_active and self._bf_debug and self.link:
                for n in self._bf_breaks:
                    self.link.write_raw(f"b{n}\n")
            self._bf_breaks.clear()
            self.query_one(BfPanel).show_breaks(self._bf_breaks)

    def on_input_changed(self, event: Input.Changed) -> None:
        if event.input.id == "freq-input":
            text = event.value
            self.query_one(ClockPanel).show_freq_preview(
                parse_hz(text), empty=not text.strip())

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        value = event.value.strip()
        if event.input.id == "bf-stdin":
            if self._bf_active and event.value and self.link:
                self.link.write_raw(event.value)
                event.input.value = ""
            return
        if event.input.id == "console-input":
            if value:
                await self.send(value)
                event.input.value = ""
        elif event.input.id == "freq-input":
            await self._set_freq_from_input()


def main() -> None:
    parser = argparse.ArgumentParser(description="Tiny Tapeout board explorer")
    parser.add_argument("--port", help="serial device (default: autodetect)")
    args = parser.parse_args()
    TTExplorerApp(port=args.port).run()


if __name__ == "__main__":
    main()
