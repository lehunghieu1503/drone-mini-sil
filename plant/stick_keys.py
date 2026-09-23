"""Keyboard adapter for `--fly`. Terminals only report key-down and auto-repeat.

A key counts as held until repeats stop. There is no key-up event, so the
hold window has to cover the keyboard repeat delay or a press would blip.
"""

from __future__ import annotations

import select
import sys
import termios
import tty

from .pilot import Manual

HOLD_S = 0.45
TILT = 0.28
YAW = 0.35
ALT_RATE = 0.45

FLY_HELP = """
fly: the pilot takes off and holds. Keys (this terminal):
  W/S forward/back   A/D left/right   Q/E yaw
  R/F altitude up/down    L land    Space arm/disarm    X quit
Release the key and it recaptures a hover where it is.
"""


def manual_from_active(held: set[str], edges: set[str]) -> Manual:
    """Map keys to sticks. On this plant, positive pitch flies toward world +x."""
    pitch = (TILT if "w" in held else 0.0) - (TILT if "s" in held else 0.0)
    roll = (TILT if "d" in held else 0.0) - (TILT if "a" in held else 0.0)
    yaw = (YAW if "e" in held else 0.0) - (YAW if "q" in held else 0.0)
    alt_rate = (ALT_RATE if "r" in held else 0.0) - (ALT_RATE if "f" in held else 0.0)
    return Manual(
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        alt_rate=alt_rate,
        toggle_arm=(" " in edges),
        land=("l" in edges),
        quit=("x" in edges),
    )


class StickReader:
    """Non-blocking cbreak reader. `open`/`close` must pair, including on failure."""

    def __init__(self, stdin=None, hold_s: float = HOLD_S):
        self._stdin = sys.stdin if stdin is None else stdin
        self._hold_s = hold_s
        self._fd = None
        self._saved = None
        self._until: dict[str, float] = {}

    def open(self) -> None:
        self._fd = self._stdin.fileno()
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def close(self) -> None:
        if self._saved is not None and self._fd is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        self._saved = None

    def poll(self, now: float) -> Manual:
        edges: set[str] = set()
        for key in self._read_ready():
            if key in ("x", "l", " "):
                edges.add(key)
            else:
                self._until[key] = now + self._hold_s
        held = {key for key, expiry in self._until.items() if expiry >= now}
        return manual_from_active(held, edges)

    def _read_ready(self) -> list[str]:
        keys: list[str] = []
        while True:
            ready, _, _ = select.select([self._stdin], [], [], 0.0)
            if not ready:
                return keys
            chunk = self._stdin.read(1)
            if not chunk:
                return keys
            if chunk == "\x1b":
                # Swallow a pending arrow sequence so it cannot stick in the buffer.
                while select.select([self._stdin], [], [], 0.0)[0]:
                    nxt = self._stdin.read(1)
                    if not nxt or nxt.isalpha():
                        break
                continue
            keys.append(chunk.lower())
