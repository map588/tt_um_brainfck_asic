"""Widgets for the explorer UI."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.message import Message
from textual.widgets import (
    Button,
    DataTable,
    Input,
    Label,
    RichLog,
    Static,
    TextArea,
)

from .index import Project


class CycleButton(Button):
    """A button whose label always shows the CURRENT state; a click
    advances to the next state and posts Cycled. Programmatic
    set_state() is silent, so syncing the UI never echoes commands."""

    class Cycled(Message):
        def __init__(self, button: "CycleButton", state: str) -> None:
            self.button = button
            self.state = state
            super().__init__()

        @property
        def control(self) -> "CycleButton":
            return self.button

    def __init__(self, states: list[tuple[str, str, str]], **kwargs) -> None:
        """states: (key, label, css_class) per state."""
        self._states = states
        self._index = 0
        super().__init__(states[0][1], **kwargs)
        self.add_class(states[0][2])

    @property
    def state(self) -> str:
        return self._states[self._index][0]

    def set_state(self, key: str) -> None:
        for i, (k, label, css) in enumerate(self._states):
            if k == key:
                self.remove_class(self._states[self._index][2])
                self._index = i
                self.label = label
                self.add_class(css)
                return
        raise ValueError(f"unknown state {key!r}")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        event.stop()
        nxt = self._states[(self._index + 1) % len(self._states)][0]
        self.set_state(nxt)
        self.post_message(self.Cycled(self, nxt))


def seven_seg(value: int) -> str:
    """ASCII art of the board's 7-segment display for a uo byte.
    Segments a..g are uo[0..6], the decimal point is uo[7]."""
    a, b, c, d, e, f, g, dp = ((value >> i) & 1 for i in range(8))
    return "\n".join([
        " _ " if a else "   ",
        ("|" if f else " ") + ("_" if g else " ") + ("|" if b else " "),
        ("|" if e else " ") + ("_" if d else " ") + ("|" if c else " ")
        + ("." if dp else " "),
    ])


def _dot(level: int) -> str:
    return "●" if level else "○"


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
    """Description of the highlighted project. Pin names appear on the
    Bench rows, so no pinout listing here."""

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
            lines += ["", f"intended clock: {p.clock_hz:,} Hz"]
        lines += ["", "[dim]select (enter) to load it on the Bench[/dim]"]
        self.query_one("#detail-text", Static).update("\n".join(lines))


class UiPanel(Vertical):
    """The 8 chip inputs. Either the MCU drives them (value buttons)
    or the bus is released so the DIP switches / PMOD drive."""

    BORDER_TITLE = "ui_in — chip inputs"

    _locked = 0  # bits the BF host owns while the BF design is loaded

    def compose(self) -> ComposeResult:
        with Horizontal(classes="bus-head"):
            yield Label("driven by ")
            yield CycleButton(
                [("mcu", " MCU ", "cyc-mcu"),
                 ("ext", " DIP·PMOD ", "cyc-ext")],
                id="ui-bus")
        for i in range(8):
            with Horizontal(classes="pin-row"):
                yield Label(str(i), classes="pin-bit")
                yield CycleButton(
                    [("0", " 0 ", "cyc-low"), ("1", " 1 ", "cyc-high")],
                    id=f"ui{i}", classes="pin-btn")
                yield Static("·", id=f"ui-lvl{i}", classes="pin-lvl")
                yield Label("", id=f"ui-name{i}", classes="pin-name")

    def byte(self) -> int:
        v = 0
        for i in range(8):
            if self.query_one(f"#ui{i}", CycleButton).state == "1":
                v |= 1 << i
        return v

    def set_names(self, pinout: dict[str, str]) -> None:
        for i in range(8):
            self.query_one(f"#ui-name{i}", Label).update(
                pinout.get(f"ui[{i}]", "") or "")

    def set_bus(self, mcu_drives: bool) -> None:
        self.query_one("#ui-bus", CycleButton).set_state(
            "mcu" if mcu_drives else "ext")
        for i in range(8):
            locked = bool((self._locked >> i) & 1)
            self.query_one(f"#ui{i}", CycleButton).disabled = (
                not mcu_drives or locked)
            if mcu_drives:
                self.query_one(f"#ui-lvl{i}", Static).update("·")

    def show_levels(self, value: int) -> None:
        """Pad levels while released (what the DIP switches set)."""
        for i in range(8):
            self.query_one(f"#ui-lvl{i}", Static).update(
                _dot((value >> i) & 1))

    def reset(self) -> None:
        for i in range(8):
            self.query_one(f"#ui{i}", CycleButton).set_state("0")
        self.set_bus(True)

    def lock_bits(self, mask: int) -> None:
        """Pins the BF host owns: their value buttons stay disabled.
        Pass 0 to unlock (non-BF designs)."""
        self._locked = mask
        mcu = self.query_one("#ui-bus", CycleButton).state == "mcu"
        for i in range(8):
            self.query_one(f"#ui{i}", CycleButton).disabled = (
                not mcu or bool((mask >> i) & 1))


class UoPanel(Vertical):
    """The 8 chip outputs, plus a mirror of the board's 7-segment
    display (which is permanently wired to this bus)."""

    BORDER_TITLE = "uo_out — chip outputs (live)"

    def compose(self) -> ComposeResult:
        for i in range(8):
            with Horizontal(classes="pin-row"):
                yield Label(str(i), classes="pin-bit")
                yield Static("○", id=f"uo-lvl{i}", classes="pin-lvl")
                yield Label("", id=f"uo-name{i}", classes="pin-name")
        with Horizontal(id="sevenseg-row"):
            yield Static(seven_seg(0), id="sevenseg")
            yield Static("uo = 0x00\non the board's\n7-segment", id="uo-hex")

    def set_names(self, pinout: dict[str, str]) -> None:
        for i in range(8):
            self.query_one(f"#uo-name{i}", Label).update(
                pinout.get(f"uo[{i}]", "") or "")

    def show(self, value: int) -> None:
        for i in range(8):
            self.query_one(f"#uo-lvl{i}", Static).update(
                _dot((value >> i) & 1))
        self.query_one("#sevenseg", Static).update(seven_seg(value))
        self.query_one("#uo-hex", Static).update(
            f"uo = 0x{value:02x}\non the board's\n7-segment")


class UioPanel(Vertical):
    """The 8 bidirectional pins. The DESIGN controls its own side per
    pin (uio_oe); these buttons set only the MCU side. Check the pin
    name before driving one."""

    BORDER_TITLE = "uio — bidirectional (MCU side)"

    def compose(self) -> ComposeResult:
        with Horizontal(classes="bus-head"):
            yield Label("design owns its side — drive inputs only")
        for i in range(8):
            with Horizontal(classes="pin-row"):
                yield Label(str(i), classes="pin-bit")
                yield CycleButton(
                    [("listen", " listen ", "cyc-listen"),
                     ("d0", " drive 0 ", "cyc-low"),
                     ("d1", " drive 1 ", "cyc-high")],
                    id=f"uio{i}", classes="pin-btn")
                yield Static("○", id=f"uio-lvl{i}", classes="pin-lvl")
                yield Label("", id=f"uio-name{i}", classes="pin-name")

    def mask(self) -> int:
        v = 0
        for i in range(8):
            if self.query_one(f"#uio{i}", CycleButton).state != "listen":
                v |= 1 << i
        return v

    def value(self) -> int:
        v = 0
        for i in range(8):
            if self.query_one(f"#uio{i}", CycleButton).state == "d1":
                v |= 1 << i
        return v

    def set_names(self, pinout: dict[str, str]) -> None:
        for i in range(8):
            self.query_one(f"#uio-name{i}", Label).update(
                pinout.get(f"uio[{i}]", "") or "")

    def show(self, value: int) -> None:
        for i in range(8):
            self.query_one(f"#uio-lvl{i}", Static).update(
                _dot((value >> i) & 1))

    def reset(self) -> None:
        for i in range(8):
            self.query_one(f"#uio{i}", CycleButton).set_state("listen")

    def lock_bits(self, mask: int) -> None:
        """Pins the BF host or the design owns: buttons disabled,
        level dots stay live. Pass 0 to unlock."""
        for i in range(8):
            self.query_one(f"#uio{i}", CycleButton).disabled = bool(
                (mask >> i) & 1)


class ClockPanel(Vertical):
    """The project clock. Exactly one of the two mode containers is
    visible: RUNNING (free-running PWM) or STOPPED (single-step)."""

    BORDER_TITLE = "clock"

    PRESETS = [(10, "10 Hz"), (1_000, "1 kHz"), (50_000, "50 kHz"),
               (200_000, "200 kHz"), (1_000_000, "1 MHz"),
               (10_000_000, "10 MHz"), (50_000_000, "50 MHz")]

    def compose(self) -> ComposeResult:
        with Vertical(id="clk-run"):
            with Horizontal(classes="clk-line"):
                yield Static("▶ RUNNING", id="clk-run-state")
                yield Static("", id="clk-run-freq")
                yield Button("■  Stop clock", id="clk-stop")
            with Horizontal(classes="clk-line"):
                yield Label("set frequency:")
                yield Input(placeholder="e.g. 440, 32k, 1.5M", id="freq-input")
                yield Static("", id="freq-preview")
                yield Button("Set", id="freq-set")
            with Horizontal(classes="clk-line"):
                yield Label("presets: ")
                for hz, label in self.PRESETS:
                    yield Button(label, id=f"preset-{hz}", classes="preset")
            yield Label("1 Hz – 75 MHz · BF programs need ≤ 200 kHz "
                        "(serial link limit)", classes="hint")
        with Vertical(id="clk-step"):
            with Horizontal(classes="clk-line"):
                yield Static("⏸ STOPPED — single-step mode",
                             id="clk-step-state")
                yield Static("stepped 0", id="step-total")
            with Horizontal(classes="clk-line"):
                yield Button("Step 1", id="step-1")
                yield Button("Step 10", id="step-10")
                yield Button("Step 100", id="step-100")
                yield Button("▶  Resume clock", id="clk-resume")
            yield Label("space = step 1 · s = resume", classes="hint")
        yield Static("", id="clk-error")

    def on_mount(self) -> None:
        """One mode container at a time, before the first status too."""
        self.show_mode("run", 0)

    def show_freq_preview(self, hz: int | None, empty: bool) -> None:
        """Live feedback while typing: what the input will set."""
        preview = self.query_one("#freq-preview", Static)
        if empty:
            preview.update("")
        elif hz is None:
            preview.update("= ? (try 440, 32k, 1.5M)")
            preview.set_class(True, "preview-bad")
        else:
            preview.update(f"= {hz:,} Hz")
            preview.set_class(False, "preview-bad")

    def show_mode(self, mode: str, freq: int) -> None:
        running = mode == "run"
        self.query_one("#clk-run").display = running
        self.query_one("#clk-step").display = not running
        self.query_one("#clk-run-freq", Static).update(f"{freq:,} Hz")

    def set_steps(self, n: int) -> None:
        self.query_one("#step-total", Static).update(f"stepped {n:,}")

    def set_error(self, text: str) -> None:
        self.query_one("#clk-error", Static).update(text)


class BfPanel(Vertical):
    """BF program entry and session output, as a tab. While a program
    runs, the firmware is in a raw byte stream and the command
    protocol is unavailable, so the Bench freezes until the run ends;
    between runs the Bench works as usual (inspect_sel included)."""

    BORDER_TITLE = "brainf*ck"

    def compose(self) -> ComposeResult:
        yield Label("program — everything except + - < > [ ] , . is a "
                    "comment; Run appends the '!' terminator",
                    classes="hint")
        yield TextArea(id="bf-program")
        with Horizontal(id="bf-controls"):
            yield Button("▶  Run on ASIC", id="bf-run")
            yield Input(placeholder="input for ',' — sent raw on enter",
                        id="bf-stdin", disabled=True)
            yield Static("", id="bf-state")
        yield RichLog(id="bf-output", markup=False, wrap=True)

    def program(self) -> str:
        return self.query_one("#bf-program", TextArea).text

    def set_running(self, running: bool) -> None:
        self.query_one("#bf-run", Button).disabled = running
        self.query_one("#bf-stdin", Input).disabled = not running
        state = self.query_one("#bf-state", Static)
        if running:
            state.update("● running — Bench frozen until the program ends")
            state.set_class(True, "bf-running")

    def show_result(self, ok: bool, detail: str) -> None:
        state = self.query_one("#bf-state", Static)
        state.set_class(False, "bf-running")
        state.set_class(not ok, "bf-error")
        state.update(detail)

    def write_output(self, text: str) -> None:
        self.query_one("#bf-output", RichLog).write(text)

    def clear_output(self) -> None:
        self.query_one("#bf-output", RichLog).clear()


class ConsolePane(Vertical):
    """Raw protocol traffic plus a free-form command line."""

    BORDER_TITLE = "serial console"

    def compose(self) -> ComposeResult:
        yield RichLog(id="console-log", markup=False, wrap=True)
        yield Input(placeholder="raw command…", id="console-input")

    def log_line(self, line: str) -> None:
        self.query_one("#console-log", RichLog).write(line)
