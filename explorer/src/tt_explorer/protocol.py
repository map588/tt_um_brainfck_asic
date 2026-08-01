"""Pure helpers for the tt-explorer serial protocol.

The firmware speaks one command per line and answers with exactly one
"ok [payload]" or "err <token>" line. Informational lines start with
"# ". No I/O happens here, so everything is unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field

PROTO_VERSION = 1


@dataclass
class Reply:
    ok: bool
    payload: str
    info: list[str] = field(default_factory=list)


def is_reply_line(line: str) -> bool:
    """True when the line ends a command (ok/err)."""
    return line.startswith("ok") or line.startswith("err")


def is_info_line(line: str) -> bool:
    return line.startswith("# ") or line == "#"


def parse_reply(line: str, info: list[str] | None = None) -> Reply:
    if line.startswith("ok"):
        return Reply(True, line[2:].strip(), info or [])
    if line.startswith("err"):
        return Reply(False, line[3:].strip(), info or [])
    raise ValueError(f"not a reply line: {line!r}")


def parse_hello(payload: str) -> dict:
    """'tt-explorer 1 bf=448' -> {'version': 1, 'bf': 448}"""
    parts = payload.split()
    if len(parts) != 3 or parts[0] != "tt-explorer":
        raise ValueError(f"bad hello: {payload!r}")
    return {"version": int(parts[1]), "bf": int(parts[2].split("=")[1])}


def parse_status(payload: str) -> dict:
    """'design=448 mode=run freq=1000000 ui=00 uiod=00 bf=1' -> dict."""
    out: dict = {}
    for part in payload.split():
        key, _, value = part.partition("=")
        if key in ("design", "freq", "bf", "uidrv"):
            out[key] = int(value)
        elif key in ("ui", "uiod"):
            out[key] = int(value, 16)
        else:
            out[key] = value
    return out


def hex_byte(value: int) -> str:
    if not 0 <= value <= 255:
        raise ValueError(f"byte out of range: {value}")
    return f"{value:02x}"


def parse_hex_byte(payload: str) -> int:
    value = int(payload, 16)
    if not 0 <= value <= 255:
        raise ValueError(f"byte out of range: {payload!r}")
    return value
