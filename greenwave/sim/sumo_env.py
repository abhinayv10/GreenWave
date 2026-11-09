"""Locate SUMO binaries from the pip-installed `eclipse-sumo` package (no system SUMO_HOME needed)."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import sumo

SUMO_HOME = Path(sumo.SUMO_HOME)
os.environ.setdefault("SUMO_HOME", str(SUMO_HOME))


def binary(name: str) -> str:
    exe = SUMO_HOME / "bin" / (name + (".exe" if sys.platform == "win32" else ""))
    if not exe.exists():
        raise FileNotFoundError(exe)
    return str(exe)
