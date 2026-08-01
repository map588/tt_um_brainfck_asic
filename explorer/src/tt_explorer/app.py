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

from . import index, protocol
from .bf_screen import BfScreen
from .index import Project
from .serial_link import SerialLink, find_ports
from .widgets import (
    ClockPanel,
    ConsolePane,
    CycleButton,
    DetailPane,
    ProjectList,
    UiPanel,
    UioPanel,
    UoPanel,
)

MAX_HZ = 2_000_000
BF_MAX_HZ = 200_000  # ASIC->MCU serial link bit-slips above this


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
    #freq-input { width: 12; }
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
        self._ui_driving = True
        self._clk_running = True
        self._steps = 0

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
        self.query_one(ClockPanel).show_mode(st["mode"], st["freq"])
        if "uidrv" in st:
            self._ui_driving = bool(st["uidrv"])
            self.query_one(UiPanel).set_bus(self._ui_driving)

    async def _poll(self) -> None:
        link = self.link
        if link is None or link.busy or link.raw:
            return
        if isinstance(self.screen, BfScreen):
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
        except (asyncio.TimeoutError, ValueError):
            pass

    # -- actions --

    def action_refresh_index(self) -> None:
        self.load_projects(refresh=True)

    def action_bf(self) -> None:
        if self.link is None:
            self._log("! not connected")
            return
        self.push_screen(BfScreen(self.link))

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
        self._ui_driving = True
        if p.clock_hz:
            cap = BF_MAX_HZ if p.address == self._bf_addr else MAX_HZ
            await self._clock_send(f"freq {min(p.clock_hz, cap)}")
        await self._refresh_status()
        tabs = self.query_one(TabbedContent)
        tabs.get_tab("tab-bench").label = f"Bench · {p.title or p.macro}"
        tabs.active = "tab-bench"

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
            value = self.query_one("#freq-input", Input).value.strip()
            if value.isdigit():
                await self._clock_send(f"freq {value}")
                await self._refresh_status()
        elif bid.startswith("preset-"):
            await self._clock_send(f"freq {bid.removeprefix('preset-')}")
            await self._refresh_status()
        elif bid.startswith("step-"):
            await self._do_step(int(bid.removeprefix("step-")))

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        value = event.value.strip()
        if event.input.id == "console-input":
            if value:
                await self.send(value)
                event.input.value = ""
        elif event.input.id == "freq-input" and value.isdigit():
            await self._clock_send(f"freq {value}")
            await self._refresh_status()


def main() -> None:
    parser = argparse.ArgumentParser(description="Tiny Tapeout board explorer")
    parser.add_argument("--port", help="serial device (default: autodetect)")
    args = parser.parse_args()
    TTExplorerApp(port=args.port).run()


if __name__ == "__main__":
    main()
