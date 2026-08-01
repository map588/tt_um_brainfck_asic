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
    """ui_in toggles, uo_out readout, uio direction/value."""

    def compose(self) -> ComposeResult:
        yield Label("pins", classes="panel-title")
        yield Label("ui_in (drive)")
        with Horizontal(id="ui-switches"):
            for i in range(7, -1, -1):
                yield Switch(id=f"ui{i}")
        yield Static("uo_out: --", id="uo-display")
        yield Static("uio:    --", id="uio-display")
        with Horizontal():
            yield Input(placeholder="uio dir hh", id="uiod-input")
            yield Input(placeholder="uio val hh", id="uiow-input")

    def ui_byte(self) -> int:
        v = 0
        for i in range(8):
            if self.query_one(f"#ui{i}", Switch).value:
                v |= 1 << i
        return v

    @staticmethod
    def _bits(value: int) -> str:
        return " ".join("●" if (value >> i) & 1 else "○"
                        for i in range(7, -1, -1))

    def show_uo(self, value: int) -> None:
        self.query_one("#uo-display", Static).update(
            f"uo_out: {self._bits(value)}  0x{value:02x}")

    def show_uio(self, value: int) -> None:
        self.query_one("#uio-display", Static).update(
            f"uio:    {self._bits(value)}  0x{value:02x}")


class ConsolePane(Vertical):
    """Raw protocol traffic plus a free-form command line."""

    def compose(self) -> ComposeResult:
        yield RichLog(id="console-log", markup=False, wrap=True)
        yield Input(placeholder="raw command…", id="console-input")

    def log_line(self, line: str) -> None:
        self.query_one("#console-log", RichLog).write(line)
