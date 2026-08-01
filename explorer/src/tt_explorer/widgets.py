"""Widgets for the explorer UI."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.message import Message
from textual.widgets import Button, DataTable, Input, Label, RichLog, Static, Switch

from .index import Project


class ProjectList(Vertical):
    """Filterable table of shuttle projects."""

    class Selected(Message):
        def __init__(self, project: Project) -> None:
            self.project = project
            super().__init__()

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._projects: list[Project] = []
        self._by_address: dict[int, Project] = {}

    def compose(self) -> ComposeResult:
        yield Input(placeholder="filter projects…", id="project-filter")
        yield DataTable(id="project-table", cursor_type="row")

    def on_mount(self) -> None:
        table = self.query_one(DataTable)
        table.add_columns("addr", "title", "author", "clock")

    def set_projects(self, projects: list[Project]) -> None:
        self._projects = projects
        self._by_address = {p.address: p for p in projects}
        self._fill(projects)

    def _fill(self, projects: list[Project]) -> None:
        table = self.query_one(DataTable)
        table.clear()
        for p in projects:
            clock = f"{p.clock_hz:,}" if p.clock_hz else ""
            table.add_row(str(p.address), p.title or p.macro, p.author, clock,
                          key=str(p.address))

    def on_input_changed(self, event: Input.Changed) -> None:
        needle = event.value.lower()
        if not needle:
            self._fill(self._projects)
            return
        self._fill([
            p for p in self._projects
            if needle in p.title.lower() or needle in p.author.lower()
            or needle in p.macro.lower() or needle in str(p.address)
        ])

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        project = self._by_address.get(int(event.row_key.value))
        if project:
            self.post_message(self.Selected(project))


class DetailPane(VerticalScroll):
    """Description and pinout of the highlighted project."""

    def compose(self) -> ComposeResult:
        yield Static("select a project…", id="detail-text")

    def show(self, p: Project) -> None:
        lines = [
            f"[b]{p.title or p.macro}[/b]  (addr {p.address})",
            f"[dim]{p.author}[/dim]",
            "",
            p.description or "(no description)",
        ]
        if p.clock_hz:
            lines += ["", f"clock: {p.clock_hz:,} Hz"]
        if p.pinout:
            lines.append("")
            for pin, name in p.pinout.items():
                if name:
                    lines.append(f"  {pin:8s} {name}")
        self.query_one("#detail-text", Static).update("\n".join(lines))


class ClockPanel(Vertical):
    """ASIC clock: set frequency, stop into step mode, pulse, resume."""

    def compose(self) -> ComposeResult:
        yield Label("clock", classes="panel-title")
        with Horizontal():
            yield Input(placeholder="Hz", id="freq-input")
            yield Button("set", id="freq-set")
        with Horizontal():
            yield Button("stop", id="clk-stop")
            yield Input(value="1", id="step-count")
            yield Button("step", id="clk-step")
            yield Button("resume", id="clk-resume")
        yield Static("mode: ?", id="clk-mode")

    def show_mode(self, mode: str, freq: int) -> None:
        self.query_one("#clk-mode", Static).update(
            f"mode: {mode}  freq: {freq:,} Hz")


class PinPanel(Vertical):
    """Pin control. Every board connector (DIP switches, 7-segment,
    PMODs, headers, MCU) shares the same nets, so this panel manages
    who drives:

    - ui_in: the MCU drives the switches' value, or `release` frees
      the pins so the DIP switches / PMOD can drive them.
    - uo_out: always driven by the design; the 7-segment shows the
      same byte.
    - uio: the DESIGN controls its own side per pin (uio_oe). The
      `out` toggle here sets only the MCU side — enable it only for
      pins the selected design's pinout declares as its inputs.
    """

    def compose(self) -> ComposeResult:
        yield Label("pins", classes="panel-title")
        with Horizontal(id="ui-head"):
            yield Label("ui_in  bit 7 → 0")
            yield Button("release", id="ui-mode")
        with Horizontal(id="ui-switches"):
            for i in range(7, -1, -1):
                yield Switch(id=f"ui{i}")
        yield Static("", id="ui-display")
        yield Static("uo_out: --", id="uo-display")
        yield Label("uio   pin (design)      out val lvl", id="uio-caption")
        for i in range(8):
            with Horizontal(classes="uio-row"):
                yield Label(str(i), classes="uio-bit")
                yield Label("", id=f"uio-name{i}", classes="uio-name")
                yield Switch(id=f"uiod{i}")
                yield Switch(id=f"uiow{i}", disabled=True)
                yield Static("○", id=f"uio-lvl{i}", classes="uio-lvl")

    # -- reads --

    def ui_byte(self) -> int:
        v = 0
        for i in range(8):
            if self.query_one(f"#ui{i}", Switch).value:
                v |= 1 << i
        return v

    def uiod_mask(self) -> int:
        v = 0
        for i in range(8):
            if self.query_one(f"#uiod{i}", Switch).value:
                v |= 1 << i
        return v

    def uiow_byte(self) -> int:
        v = 0
        for i in range(8):
            if self.query_one(f"#uiow{i}", Switch).value:
                v |= 1 << i
        return v

    # -- updates --

    @staticmethod
    def _bits(value: int) -> str:
        return " ".join("●" if (value >> i) & 1 else "○"
                        for i in range(7, -1, -1))

    def set_pinout(self, pinout: dict[str, str]) -> None:
        """Show the selected design's own uio pin names."""
        for i in range(8):
            name = pinout.get(f"uio[{i}]", "") or ""
            self.query_one(f"#uio-name{i}", Label).update(name[:18])

    def set_ui_mode(self, driving: bool) -> None:
        self.query_one("#ui-mode", Button).label = (
            "release" if driving else "drive")
        for i in range(8):
            self.query_one(f"#ui{i}", Switch).disabled = not driving
        self.query_one("#ui-display", Static).update(
            "" if driving else "released — DIP/PMOD drive the pins")

    def show_ui_levels(self, value: int) -> None:
        self.query_one("#ui-display", Static).update(
            f"pads:   {self._bits(value)}  0x{value:02x}")

    def show_uo(self, value: int) -> None:
        self.query_one("#uo-display", Static).update(
            f"uo_out: {self._bits(value)}  0x{value:02x}")

    def show_uio(self, value: int) -> None:
        for i in range(8):
            self.query_one(f"#uio-lvl{i}", Static).update(
                "●" if (value >> i) & 1 else "○")

    def sync_uiow_enable(self) -> None:
        for i in range(8):
            drives = self.query_one(f"#uiod{i}", Switch).value
            self.query_one(f"#uiow{i}", Switch).disabled = not drives

    def reset_for_design(self) -> None:
        """Match the firmware's safe profile after a design switch."""
        with self.prevent(Switch.Changed):
            for i in range(8):
                self.query_one(f"#ui{i}", Switch).value = False
                self.query_one(f"#uiod{i}", Switch).value = False
                self.query_one(f"#uiow{i}", Switch).value = False
        self.sync_uiow_enable()
        self.set_ui_mode(True)


class ConsolePane(Vertical):
    """Raw protocol traffic plus a free-form command line."""

    def compose(self) -> ComposeResult:
        yield RichLog(id="console-log", markup=False, wrap=True)
        yield Input(placeholder="raw command…", id="console-input")

    def log_line(self, line: str) -> None:
        self.query_one("#console-log", RichLog).write(line)
