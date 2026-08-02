"""Modal raw-passthrough screen for BF sessions.

The firmware's `bf` command switches the session to a raw byte
stream: program echo, `,` input, and `.` output are not line-based.
Every keystroke goes straight to the port; every incoming chunk is
shown. The screen closes itself when the final "ok done"/"err …"
reply appears, or on Esc.
"""

from __future__ import annotations

import re

from textual import events
from textual.app import ComposeResult
from textual.screen import ModalScreen
from textual.widgets import Footer, RichLog, Static

from .serial_link import SerialLink

_END = re.compile(r"(?:^|\n)(?:ok (done)|err ([a-z-]+))\s*$")


class BfScreen(ModalScreen):
    BINDINGS = [("escape", "dismiss", "leave bf session")]

    def __init__(self, link: SerialLink) -> None:
        super().__init__()
        self._link = link
        self._tail = ""
        self._done = False
        self._failed: str | None = None
        self._fail_shown = False

    def compose(self) -> ComposeResult:
        yield Static("BF session — keys go to the board, Esc leaves",
                     id="bf-banner")
        yield RichLog(id="bf-log", markup=False, wrap=True)
        yield Footer()

    def on_mount(self) -> None:
        self._link.set_raw_sink(self._on_chunk)
        self._link.write_raw("bf\n")
        # _on_chunk runs as a plain asyncio callback, outside the
        # Textual app context, so it cannot start timers itself.
        self.set_interval(0.25, self._check_done)

    def _check_done(self) -> None:
        if self._done:
            self._done = False
            self.dismiss()
        elif self._failed and not self._fail_shown:
            self._fail_shown = True
            banner = self.query_one("#bf-banner", Static)
            banner.update(
                f"session failed: {self._failed} — Esc to close")
            banner.add_class("bf-error")

    def _on_chunk(self, text: str) -> None:
        self.query_one("#bf-log", RichLog).write(text)
        self._tail = (self._tail + text)[-96:]
        m = _END.search(self._tail.replace("\r", ""))
        if m:
            if m.group(1):  # ok done: close quietly
                self._done = True
            else:  # err <token>: stay open, show why
                self._failed = m.group(2)

    def on_key(self, event: events.Key) -> None:
        if event.key == "enter":
            self._link.write_raw("\n")
            event.stop()
        elif event.character and event.character.isprintable():
            self._link.write_raw(event.character)
            event.stop()

    def on_paste(self, event: events.Paste) -> None:
        """Terminal pastes arrive as one event, not as keys."""
        text = event.text.replace("\r\n", "\n").replace("\r", "\n")
        if text:
            self._link.write_raw(text)
        event.stop()

    def on_unmount(self) -> None:
        self._link.set_raw_sink(None)
