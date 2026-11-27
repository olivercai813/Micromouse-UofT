"""Time, real or simulated.

Everything in the package asks a clock for the time and for sleeps, so a whole
session (headset, wearer, mouse) can be run in simulated time in a test: the
simulated clock steps its listeners forward whenever someone sleeps.
"""

from __future__ import annotations

import time


class RealClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


class SimClock:
    """Simulated time. `listeners` are called with the new time on every step,
    so simulated devices advance while the session waits."""

    def __init__(self, start: float = 100.0, step: float = 0.004):
        self.t = float(start)
        self.step = float(step)
        self.listeners = []

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        end = self.t + max(0.0, seconds)
        while self.t < end:
            # always reaches `end` exactly, however small the remainder
            self.t = end if end - self.t <= self.step else self.t + self.step
            for listener in self.listeners:
                listener(self.t)
