"""Shuttle index: fetch, cache, and parse the ttsky25b project list."""

from __future__ import annotations

import json
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

SHUTTLE = "ttsky25b"
URL = (
    f"https://index.tinytapeout.com/{SHUTTLE}.json"
    "?fields=address,title,author,description,clock_hz,pinout"
)
CACHE = Path(f"~/.cache/tt-explorer/{SHUTTLE}.json").expanduser()
CACHE_MAX_AGE_S = 7 * 24 * 3600


@dataclass
class Project:
    macro: str
    address: int
    title: str = ""
    author: str = ""
    description: str = ""
    clock_hz: int | None = None
    pinout: dict[str, str] = field(default_factory=dict)


def _parse(raw: bytes) -> list[Project]:
    data = json.loads(raw)
    projects = []
    for p in data["projects"]:
        projects.append(
            Project(
                macro=p.get("macro", ""),
                address=p["address"],
                title=p.get("title", ""),
                author=p.get("author", ""),
                description=p.get("description", ""),
                clock_hz=p.get("clock_hz"),
                pinout=p.get("pinout", {}) or {},
            )
        )
    projects.sort(key=lambda p: p.address)
    return projects


def load_index(refresh: bool = False) -> list[Project]:
    """Return the project list. Network fetch when the cache is stale
    or refresh is requested; otherwise (and on fetch failure) use the
    cache."""
    cache_ok = CACHE.exists()
    cache_fresh = cache_ok and (time.time() - CACHE.stat().st_mtime) < CACHE_MAX_AGE_S

    if cache_fresh and not refresh:
        return _parse(CACHE.read_bytes())

    try:
        # The index server rejects the default Python-urllib user agent.
        req = urllib.request.Request(URL, headers={"User-Agent": "tt-explorer"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            raw = resp.read()
        CACHE.parent.mkdir(parents=True, exist_ok=True)
        CACHE.write_bytes(raw)
        return _parse(raw)
    except OSError:
        if cache_ok:
            return _parse(CACHE.read_bytes())
        raise
