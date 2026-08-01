"""tt-explorer: Textual TUI for the tt-explorer firmware."""

from __future__ import annotations

import argparse
import asyncio

from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Footer, Header, Input, Switch

from . import index, protocol
from .bf_screen import BfScreen
from .index import Project
from .serial_link import SerialLink, find_ports
from .widgets import (
    ClockPanel,
    ConsolePane,
    DetailPane,
    PinPanel,
    ProjectList,
)

MAX_HZ = 2_000_000
BF_MAX_HZ = 200_000  # ASIC->MCU serial link bit-slips above this


class TTExplorerApp(App):
    TITLE = "tt-explorer"

    CSS = """
    #main { height: 3fr; }
    ProjectList { width: 45%; }
    #project-table { height: 1fr; }
    #right { width: 55%; }
    DetailPane { height: 1fr; border: solid $primary; padding: 0 1; }
    #panels { height: auto; }
    ClockPanel, PinPanel { width: 1fr; border: solid $secondary; padding: 0 1; }
    .panel-title { text-style: bold; }
    ConsolePane { height: 1fr; border: solid $accent; }
    #console-log { height: 1fr; }
    #freq-input, #step-count { width: 12; }
    Switch { width: 5; }
    Button { min-width: 8; }
    PinPanel Switch { border: none; height: 1; width: 4; padding: 0 1; }
    PinPanel { height: auto; }
    #ui-head { height: 1; }
    #ui-mode { border: none; height: 1; min-width: 9; margin-left: 2; }
    #ui-switches { height: 1; }
    .uio-row { height: 1; }
    .uio-bit { width: 3; color: $text-muted; }
    .uio-name { width: 19; color: $text-muted; }
    .uio-lvl { width: 3; }
    #uio-caption { color: $text-muted; }
    """

    BINDINGS = [
        ("q", "quit", "quit"),
        ("r", "refresh_index", "refresh index"),
        ("b", "bf", "bf session"),
    ]

    def __init__(self, port: str | None = None) -> None:
        super().__init__()
        self._port_arg = port
        self.link: SerialLink | None = None
        self._bf_addr: int | None = None
        self._ui_driving = True

    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal(id="main"):
            yield ProjectList()
            with Vertical(id="right"):
                yield DetailPane()
                with Horizontal(id="panels"):
                    yield ClockPanel()
                    yield PinPanel()
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

    def _log(self, line: str) -> None:
        self.query_one(ConsolePane).log_line(line)

    async def _refresh_status(self) -> None:
        reply = await self.send("status")
        if reply and reply.ok:
            st = protocol.parse_status(reply.payload)
            self.query_one(ClockPanel).show_mode(st["mode"], st["freq"])

    async def _poll(self) -> None:
        link = self.link
        if link is None or link.busy or link.raw:
            return
        if isinstance(self.screen, BfScreen):
            return
        panel = self.query_one(PinPanel)
        try:
            reply = await link.request("uo", timeout=1.0)
            if reply.ok:
                panel.show_uo(protocol.parse_hex_byte(reply.payload))
            reply = await link.request("uio", timeout=1.0)
            if reply.ok:
                panel.show_uio(protocol.parse_hex_byte(reply.payload))
            if not self._ui_driving:
                reply = await link.request("ui", timeout=1.0)
                if reply.ok:
                    panel.show_ui_levels(protocol.parse_hex_byte(reply.payload))
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

    # -- UI events --

    async def on_project_list_selected(self, event: ProjectList.Selected) -> None:
        p: Project = event.project
        self.query_one(DetailPane).show(p)
        reply = await self.send(f"design {p.address}")
        if reply and reply.ok:
            panel = self.query_one(PinPanel)
            panel.set_pinout(p.pinout)
            panel.reset_for_design()
            self._ui_driving = True
            if p.clock_hz:
                cap = BF_MAX_HZ if p.address == self._bf_addr else MAX_HZ
                await self.send(f"freq {min(p.clock_hz, cap)}")
        await self._refresh_status()

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "ui-mode":
            panel = self.query_one(PinPanel)
            if self._ui_driving:
                reply = await self.send("ui off")
                if reply and reply.ok:
                    self._ui_driving = False
                    panel.set_ui_mode(False)
            else:
                reply = await self.send(
                    f"ui {protocol.hex_byte(panel.ui_byte())}")
                if reply and reply.ok:
                    self._ui_driving = True
                    panel.set_ui_mode(True)
            return
        if event.button.id == "freq-set":
            value = self.query_one("#freq-input", Input).value.strip()
            if value.isdigit():
                await self.send(f"freq {value}")
        elif event.button.id == "clk-stop":
            await self.send("stop")
        elif event.button.id == "clk-step":
            count = self.query_one("#step-count", Input).value.strip() or "1"
            if count.isdigit():
                await self.send(f"step {count}")
        elif event.button.id == "clk-resume":
            await self.send("resume")
        else:
            return
        await self._refresh_status()

    async def on_switch_changed(self, event: Switch.Changed) -> None:
        sid = event.switch.id or ""
        panel = self.query_one(PinPanel)
        if sid.startswith("uiod"):
            panel.sync_uiow_enable()
            await self.send(f"uiod {protocol.hex_byte(panel.uiod_mask())}")
        elif sid.startswith("uiow"):
            await self.send(f"uiow {protocol.hex_byte(panel.uiow_byte())}")
        elif sid.startswith("ui") and self._ui_driving:
            await self.send(f"ui {protocol.hex_byte(panel.ui_byte())}")

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        value = event.value.strip()
        if event.input.id == "console-input":
            if value:
                await self.send(value)
                event.input.value = ""
        elif event.input.id == "freq-input" and value.isdigit():
            await self.send(f"freq {value}")
            await self._refresh_status()


def main() -> None:
    parser = argparse.ArgumentParser(description="Tiny Tapeout board explorer")
    parser.add_argument("--port", help="serial device (default: autodetect)")
    args = parser.parse_args()
    TTExplorerApp(port=args.port).run()


if __name__ == "__main__":
    main()
