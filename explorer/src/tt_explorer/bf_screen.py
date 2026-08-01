"""Modal raw-passthrough screen for BF sessions.

The firmware's `bf` command switches the session to a raw byte
stream: program echo, `,` input, and `.` output are not line-based.
Every keystroke goes straight to the port; every incoming chunk is
shown. The screen closes itself when the final "ok done"/"err …"
reply appears, or on Ctrl+Q.
"""

from __future__ import annotations

import re

from textual import events
from textual.app import ComposeResult
from textual.screen import ModalScreen
from textual.widgets import Footer, RichLog, Static

from .serial_link import SerialLink

_END = re.compile(r"(?:^|\n)(ok done|err [a-z-]+)\s*$")


class BfScreen(ModalScreen):
    BINDINGS = [("ctrl+q", "dismiss", "leave bf session")]

    def __init__(self, link: SerialLink) -> None:
        super().__init__()
        self._link = link
        self._tail = ""

    def compose(self) -> ComposeResult:
        yield Static("BF session — keys go to the board, Ctrl+Q leaves",
                     id="bf-banner")
        yield RichLog(id="bf-log", markup=False, wrap=True)
        yield Footer()

    def on_mount(self) -> None:
        self._link.set_raw_sink(self._on_chunk)
        self._link.write_raw("bf\n")

    def _on_chunk(self, text: str) -> None:
        self.query_one("#bf-log", RichLog).write(text)
        self._tail = (self._tail + text)[-96:]
        if _END.search(self._tail.replace("\r", "")):
            self.set_timer(0.5, self.dismiss)

    def on_key(self, event: events.Key) -> None:
        if event.key == "enter":
            self._link.write_raw("\n")
            event.stop()
        elif event.character and event.character.isprintable():
            self._link.write_raw(event.character)
            event.stop()

    def on_unmount(self) -> None:
        self._link.set_raw_sink(None)
