"""Serial connection to the tt-explorer firmware.

One background thread reads bytes; lines are routed on the asyncio
event loop. request() sends one command and awaits its ok/err reply.
While raw mode is active (BF sessions), incoming chunks bypass the
line router and go to the raw sink.
"""

from __future__ import annotations

import asyncio
import glob
import threading
from dataclasses import dataclass, field
from typing import Callable

import serial
from serial.tools import list_ports

from .protocol import Reply, is_info_line, is_reply_line, parse_reply

RPI_VID = 0x2E8A
BAUD = 115200  # ignored by USB CDC, required by pyserial


def find_ports() -> list[str]:
    ports = [p.device for p in list_ports.comports() if p.vid == RPI_VID]
    if not ports:
        ports = sorted(glob.glob("/dev/tty.usbmodem*"))
    return ports


@dataclass
class _Pending:
    sent: str
    future: asyncio.Future
    info: list[str] = field(default_factory=list)
    echo_dropped: bool = False


class SerialLink:
    def __init__(self, port: str, on_line: Callable[[str], None]):
        """on_line gets every unsolicited line (called on the loop)."""
        self._ser = serial.Serial(port, BAUD, timeout=0.2)
        self._on_line = on_line
        self._loop = asyncio.get_running_loop()
        self._lock = asyncio.Lock()
        self._pending: _Pending | None = None
        self._raw_sink: Callable[[str], None] | None = None
        self._closed = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    @property
    def port(self) -> str:
        return self._ser.port

    @property
    def busy(self) -> bool:
        return self._lock.locked()

    @property
    def raw(self) -> bool:
        return self._raw_sink is not None

    def close(self) -> None:
        self._closed = True
        try:
            self._ser.cancel_read()  # wake the reader thread
        except (OSError, AttributeError):
            pass
        try:
            self._ser.close()
        except OSError:
            pass
        self._thread.join(timeout=1.0)

    # -- raw mode (BF passthrough) --

    def set_raw_sink(self, sink: Callable[[str], None] | None) -> None:
        self._raw_sink = sink

    def write_raw(self, text: str) -> None:
        self._ser.write(text.encode())

    # -- command/reply --

    async def request(self, line: str, timeout: float = 3.0) -> Reply:
        async with self._lock:
            fut: asyncio.Future = self._loop.create_future()
            self._pending = _Pending(line, fut)
            try:
                self._ser.write((line + "\n").encode())
                return await asyncio.wait_for(fut, timeout)
            finally:
                self._pending = None

    # -- reader thread --

    def _reader(self) -> None:
        buf = b""
        while not self._closed:
            try:
                data = self._ser.read(256)
            except (OSError, serial.SerialException):
                break
            if not data:
                continue
            if self._raw_sink is not None:
                text = data.decode(errors="replace")
                self._loop.call_soon_threadsafe(self._raw_sink, text)
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").rstrip("\r")
                self._loop.call_soon_threadsafe(self._route_line, line)

    def _route_line(self, line: str) -> None:
        p = self._pending
        if p is not None and not p.future.done():
            # The firmware echoes what we sent, and it processes
            # commands in order. So the echo marks where our reply
            # begins: any reply line before it belongs to an earlier
            # command that timed out, and accepting it would shift
            # every later reply to the wrong request.
            if not p.echo_dropped:
                if line == p.sent:
                    p.echo_dropped = True
                elif line:
                    self._on_line(line)
                return
            if is_info_line(line):
                p.info.append(line)
                self._on_line(line)
                return
            if is_reply_line(line):
                p.future.set_result(parse_reply(line, p.info))
                return
        if line:
            self._on_line(line)
