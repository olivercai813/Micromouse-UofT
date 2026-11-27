"""The headset's side of the session: showing the lane and flashing squares.

Two displays share one small interface:

  WebDisplay   the page in the Quest browser (questpath/web/index.html), told
               what to do through PageServer; it reports when each flash
               really started and ended on its own frame clock.
  SimDisplay   no page at all: flashes happen exactly on a simulated clock.

A flash is one square (or, kind "calib", the whole row; kind "blank", nothing)
flickering for a set time. When it is over the display hands back an Epoch
with its start and end on the laptop's clock and the true flicker rate.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Epoch:
    id: int
    row: int
    col: int
    t0: float
    t1: float
    hz: float
    kind: str = "square"
    dropped: int = 0


def pick_rate(refresh: float, target: float):
    """Flicker rate a display can show exactly: on for k frames, off for k.
    Returns (hz, k): the rate nearest `target` that stays out of the 9-11.5 Hz
    alpha band (the brain's own idle rhythm) if there is one."""
    options = [(refresh / (2.0 * k), k) for k in range(1, 13)]
    clear = [o for o in options if 11.5 <= o[0] <= 20.0]
    return min(clear or options, key=lambda o: abs(o[0] - target))


class SimDisplay:
    def __init__(self, clock, refresh: float = 72.0, target_hz: float = 13.0, delay: float = 0.03):
        self.clock = clock
        self.refresh = refresh
        self.hz, self.half_frames = pick_rate(refresh, target_hz)
        self.delay = delay
        self.epochs = []                 # every flash, done or still running
        self.states = []                 # every board shown, for tests
        self._reported = 0
        self._next_id = 1
        self.buttons = []                # tests push names here

    def ready(self) -> bool:
        return True

    def show(self, state: dict) -> None:
        self.states.append(dict(state))

    def flash(self, row: int, col: int, seconds: float, kind: str = "square") -> int:
        t0 = self.clock.now() + self.delay
        cycles = max(1, round(seconds * self.hz))
        epoch = Epoch(self._next_id, row, col, t0, t0 + cycles / self.hz, self.hz, kind)
        self._next_id += 1
        self.epochs.append(epoch)
        return epoch.id

    def poll(self):
        out = [{"type": "button", "name": name} for name in self.buttons]
        self.buttons = []
        now = self.clock.now()
        while self._reported < len(self.epochs) and self.epochs[self._reported].t1 <= now:
            out.append({"type": "flash_done", "epoch": self.epochs[self._reported]})
            self._reported += 1
        return out

    def known_until(self) -> float:
        return float("inf")              # the whole schedule is known in advance

    def close(self) -> None:
        pass


class WebDisplay:
    def __init__(self, server, clock, target_hz: float = 13.0):
        self.server = server
        self.clock = clock
        self.target_hz = target_hz
        self.epochs = []
        self._pending = {}               # flash id -> (when asked, row, col, kind)
        self._held = []                  # events read from the page, not yet handed on
        self._next_id = 1
        self.refresh = 0.0
        self.hz = 0.0

    def ready(self) -> bool:
        self._absorb()
        return self.server.clients() > 0 and self.refresh > 0

    def show(self, state: dict) -> None:
        self.server.push(dict(state, cmd="state"))

    def flash(self, row: int, col: int, seconds: float, kind: str = "square") -> int:
        flash_id = self._next_id
        self._next_id += 1
        self._pending[flash_id] = (self.clock.now(), row, col, kind)
        self.server.push({"cmd": "flash", "id": flash_id, "row": row, "col": col, "seconds": seconds,
                          "kind": kind, "target_hz": self.target_hz})
        return flash_id

    def _absorb(self):
        out = []
        for event in self.server.events():
            kind = event.get("type")
            if kind == "hello":
                self.refresh = float(event.get("refresh") or 0.0)
                self.hz = float(event.get("hz") or 0.0)
                out.append({"type": "hello", "refresh": self.refresh, "hz": self.hz,
                            "agent": event.get("agent", "")})
            elif kind == "flash_done" and event.get("id") in self._pending:
                _, row, col, flash_kind = self._pending.pop(event["id"])
                epoch = Epoch(event["id"], row, col, self.server.to_laptop(event["t0"]),
                              self.server.to_laptop(event["t1"]), float(event["hz"]), flash_kind,
                              int(event.get("dropped", 0)))
                self.epochs.append(epoch)
                out.append({"type": "flash_done", "epoch": epoch})
            elif kind == "button":
                out.append({"type": "button", "name": event.get("name", "")})
        self._held.extend(out)

    def poll(self):
        self._absorb()
        out, self._held = self._held, []
        return out

    def known_until(self) -> float:
        """Up to when it is known what was on the screen."""
        if self._pending:
            return min(asked for asked, _, _, _ in self._pending.values())
        return self.clock.now()

    def close(self) -> None:
        self.server.push({"cmd": "state", "phase": "done", "message": "Session closed."})
