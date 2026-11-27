"""A whole session with nothing real in it: simulated headset, wearer and
mouse on a simulated clock. Used by `python -m questpath demo` and the tests.
"""

from __future__ import annotations

from .clock import SimClock
from .display import SimDisplay
from .eeg import EEGBuffer
from .mouse import Mouse, MouseConfig, SimMouse
from .session import Session, SessionConfig
from .wearer import SimWearer

LETTERS = {"L": 0, "M": 1, "R": 2, "-": None}


def parse_path(text: str):
    """"M,L,L,R" -> {1: 1, 2: 0, 3: 0, 4: 2}: the square looked at in each
    row after the start ("-" = looking away)."""
    out = {}
    for row, word in enumerate((w.strip().upper() for w in text.split(",") if w.strip()), start=1):
        if word in LETTERS:
            out[row] = LETTERS[word]
        elif word.isdigit():
            out[row] = int(word)
        else:
            raise ValueError("path entries are L, M, R, - or a column number, not %r" % word)
    return out


class Rehearsal:
    def __init__(self, gaze, config: SessionConfig | None = None, seed: int = 0, refresh: float = 72.0,
                 wearer: dict | None = None, mouse: dict | None = None, mouse_config: MouseConfig | None = None,
                 say=None, eeg_rate: float = 256.0):
        self.clock = SimClock()
        self.config = config or SessionConfig(wait_start=False)
        self.config.wait_start = False
        if self.config.seed is None:
            self.config.seed = seed
        self.display = SimDisplay(self.clock, refresh=refresh)
        self.eeg = EEGBuffer()
        self.wearer = SimWearer(self.display, self.eeg, gaze, seed=seed, rate=eeg_rate, **(wearer or {}))
        self.board_model = SimMouse(self.clock, **(mouse or {}))
        self.mouse = Mouse(self.board_model, self.clock, mouse_config)
        self.clock.listeners.append(self.wearer.advance)
        self.lines = []
        self.session = Session(self.clock, self.display, self.eeg, self.mouse, self.config,
                               say=say or self.lines.append)

    def run(self) -> dict:
        started = self.clock.now()
        summary = self.session.run()
        summary["seconds"] = round(self.clock.now() - started, 1)
        lane = self.config.lane
        summary["mouse_mm"] = [round(self.board_model.x, 1), round(self.board_model.y, 1)]
        summary["mouse_heading"] = round(self.board_model.heading, 1)
        summary["target_mm"] = [round(v, 1) for v in lane.centre_mm(tuple(summary["mouse"]))]
        return summary
