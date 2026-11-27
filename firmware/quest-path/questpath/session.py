"""One tracing session, from "press Start" to the mouse's last square.

For each row ahead of the mouse:
  1. the row's squares are flashed one at a time, in random order, with a
     short pause between flashes; the program remembers which square each
     flash was;
  2. every flash is scored for the flicker rate in the EEG (detect.py);
  3. after each round the row is decided: one square stands out -> the path
     goes there and the mouse drives to it; nothing stands out after the last
     round -> the mouse stays where it is and the row is tried again.

The session only talks to a clock, a display, an EEG buffer and a mouse, so
the same code runs the real thing and a simulated rehearsal.
"""

from __future__ import annotations

import json
import random
from dataclasses import asdict, dataclass, field

import numpy as np

from .detect import DetectConfig, FlashScore, RowEvidence, Scorer
from .grid import Lane, board, plan_step
from .mouse import MouseError


class Stopped(Exception):
    """The wearer or the operator ended the session."""


class SessionError(RuntimeError):
    pass


@dataclass
class SessionConfig:
    lane: Lane = field(default_factory=Lane)
    detect: DetectConfig = field(default_factory=DetectConfig)
    flash_s: float = 3.0          # how long each square flickers
    gap_s: float = 0.5            # steady pause between flashes
    style: str = "square"         # "square": straight runs and quarter turns; "diagonal"
    drive: str = "each"           # "each": drive after every row; "end": trace first, then drive
    calib_blocks: int = 4         # flicker / no-flicker pairs before tracing (0 = skip)
    calib_s: float = 3.0
    max_idle: int = 5             # rows in a row with nothing picked before giving up
    settle_s: float = 0.6         # after the mouse stops, before the next row
    wait_start: bool = True
    ready_timeout_s: float = 120.0
    seed: int | None = None


class Session:
    def __init__(self, clock, display, eeg, mouse, config: SessionConfig | None = None,
                 pump=None, say=print, log_path: str | None = None):
        self.clock = clock
        self.display = display
        self.eeg = eeg
        self.mouse = mouse
        self.config = config or SessionConfig()
        self.pump = pump or (lambda now: None)
        self.say = say
        self.log_path = log_path
        self.scorer = Scorer(self.config.detect)
        self.rng = random.Random(self.config.seed)
        self.records = []
        self._inbox = []
        self.mouse_cell = self.config.lane.start
        self.path = [self.config.lane.start]
        self.decisions = []
        self.calibration = None
        self.outcome = "not run"
        self._last_window = None

    # ------------------------------------------------------------- plumbing
    def _record(self, kind: str, **fields) -> None:
        entry = dict(kind=kind, t=round(self.clock.now(), 3), **fields)
        self.records.append(entry)
        if self.log_path:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry) + "\n")

    def _tick(self) -> None:
        """One turn of every waiting loop: let simulated things run, read the
        page, obey a Stop."""
        now = self.clock.now()
        self.pump(now)
        for event in self.display.poll():
            if event["type"] == "button" and event.get("name") == "stop":
                raise Stopped("stopped from the headset")
            self._inbox.append(event)

    def _wait(self, seconds: float) -> None:
        end = self.clock.now() + seconds
        while self.clock.now() < end:
            self._tick()
            self.clock.sleep(min(0.01, max(0.0, end - self.clock.now())))

    def _take(self, wanted):
        for i, event in enumerate(self._inbox):
            if wanted(event):
                return self._inbox.pop(i)
        return None

    def _await(self, wanted, seconds: float):
        end = self.clock.now() + seconds
        while True:
            self._tick()
            event = self._take(wanted)
            if event is not None:
                return event
            if self.clock.now() >= end:
                return None
            self.clock.sleep(0.01)

    def _show(self, message: str, phase: str, active_row=None) -> None:
        lane = self.config.lane
        self.display.show({"rows": lane.rows, "cols": lane.cols, "cell_mm": lane.cell_mm,
                           "start": list(lane.start), "mouse": list(self.mouse_cell),
                           "path": [list(c) for c in self.path], "active_row": active_row,
                           "message": message, "phase": phase})

    # -------------------------------------------------------------- flashes
    def _flash(self, row: int, col: int, seconds: float, kind: str = "square"):
        """Flash and score once. Returns (epoch or None, FlashScore or None)."""
        c = self.config.detect
        flash_id = self.display.flash(row, col, seconds, kind)
        done = self._await(lambda e: e["type"] == "flash_done" and e["epoch"].id == flash_id, seconds + 6.0)
        if done is None:
            return None, None
        epoch = done["epoch"]
        # the EEG of the end of the flash is still on its way
        end = self.clock.now() + 3.0
        while self.eeg.latest() < epoch.t1 + c.tail_s and self.clock.now() < end:
            self._tick()
            self.clock.sleep(0.01)
        x, rate, cover = self.eeg.window(epoch.t0 + c.skip_s, epoch.t1 + c.tail_s)
        self._last_window = (x, rate, epoch.hz)
        if cover < c.min_cover:
            score = FlashScore(False, why="EEG missing (%.0f%% of the flash arrived)" % (100 * cover))
        else:
            score = self.scorer.score(x, rate, epoch.hz)
        self._record("flash", flash=kind, row=row, col=col, t0=round(epoch.t0, 4), t1=round(epoch.t1, 4),
                     hz=round(epoch.hz, 4), dropped=epoch.dropped, ok=score.ok, z=round(score.z, 3),
                     rho=round(score.rho, 4), cover=round(cover, 3), channels=score.channels, why=score.why)
        return epoch, score

    def _flash_ok(self, row: int, col: int, seconds: float, kind: str = "square"):
        """A scored flash, repeated if the page or the EEG let it down."""
        why = "the headset page did not answer"
        for _ in range(3):
            epoch, score = self._flash(row, col, seconds, kind)
            if score is not None and score.ok:
                return score
            if score is not None:
                why = score.why
            self._wait(self.config.gap_s)
        raise SessionError("cannot score flashes: " + why)

    # ---------------------------------------------------------- calibration
    def calibrate(self) -> dict:
        """Flicker against no flicker, the wearer watching the row: says
        whether this headband on this head shows the flicker at all."""
        cfg = self.config
        self._show("Signal check: just watch the flickering squares.", "calib", active_row=1)
        self._wait(1.0)
        flicker, steady = [], []
        for _ in range(cfg.calib_blocks):
            self._flash_ok(1, -1, cfg.calib_s, "calib")
            flicker.append(self._last_window)
            self._wait(cfg.gap_s)
            self._flash_ok(1, -1, cfg.calib_s, "blank")
            steady.append(self._last_window)
            self._wait(cfg.gap_s)
        # which electrodes carry the flicker on this head?
        names = self.eeg.names()
        tried = self.scorer.choose_channels(flicker, steady, len(names))
        used = names if self.scorer.channels is None else [names[k] for k in self.scorer.channels]
        on = [self.scorer.score(*w).z for w in flicker]
        off = [self.scorer.score(*w).z for w in steady]
        result = {"on": [round(v, 2) for v in on], "off": [round(v, 2) for v in off],
                  "median_on": float(np.median(on)), "median_off": float(np.median(off)),
                  "clear": int(sum(v > max(off) for v in on)), "blocks": cfg.calib_blocks,
                  "electrodes": used,
                  "tried": {"+".join(names[k] for k in key): round(v, 2) for key, v in tried.items()}}
        result["usable"] = bool(result["median_on"] - result["median_off"] >= 2.0
                                and result["median_on"] >= cfg.detect.threshold * 0.8)
        self._record("calibration", **result)
        self.say("signal check: flicker scores %s, no-flicker scores %s, using %s"
                 % (result["on"], result["off"], "+".join(used)))
        if result["usable"]:
            self.say("  the flicker shows in the EEG (median %.1f against %.1f)."
                     % (result["median_on"], result["median_off"]))
        else:
            self.say("  WARNING: the flicker barely shows in the EEG (median %.1f against %.1f). Expect "
                     "rows with nothing picked. Check electrode contact; an electrode over the back of "
                     "the head (visual cortex) helps most." % (result["median_on"], result["median_off"]))
        return result

    # ----------------------------------------------------------------- rows
    def choose(self, row: int):
        """Flash the row until one square stands out. Returns (col or None,
        the evidence)."""
        cfg = self.config
        evidence = RowEvidence(cfg.lane.cols, cfg.detect)
        while True:
            order = list(range(cfg.lane.cols))
            self.rng.shuffle(order)
            for col in order:
                evidence.add(col, self._flash_ok(row, col, cfg.flash_s).z)
                self._wait(cfg.gap_s)
            verdict, col = evidence.decide()
            if verdict != "more":
                combined = {k: round(v, 2) for k, v in sorted(evidence.combined().items())}
                self._record("decision", row=row, pick=col, rounds=evidence.rounds(), z=combined)
                self.decisions.append({"row": row, "pick": col, "rounds": evidence.rounds(), "z": combined})
                return col, evidence

    def _drive(self, moves, target) -> None:
        if self.mouse is None:
            self.mouse_cell = target
            return
        self.mouse.execute(moves)
        self.mouse_cell = target
        self._record("moved", to=list(target))

    def trace(self) -> None:
        cfg = self.config
        lane = cfg.lane
        here = lane.start
        pending = []
        idle = 0
        row = 1
        while row < lane.rows:
            self._show("Look at the square the mouse should go to.", "trace", active_row=row)
            self._wait(cfg.settle_s)
            col, evidence = self.choose(row)
            scores = "  ".join("%s %+.1f" % ("LMR"[k] if lane.cols == 3 else k, v)
                               for k, v in sorted(evidence.combined().items()))
            if col is None:
                idle += 1
                self.say("row %d: nothing picked (%s) - the mouse stays" % (row, scores))
                self._show("No square picked: the mouse stays. Trying this row again.", "trace", active_row=row)
                if idle >= cfg.max_idle:
                    self.outcome = "gave up: nothing picked %d times in a row" % idle
                    return
                self._wait(1.5)
                continue
            idle = 0
            target = (row, col)
            self.path.append(target)
            name = ("left", "middle", "right")[col] if lane.cols == 3 else "square %d" % col
            self.say("row %d: %s (%s, %d round%s)" % (row, name, scores, evidence.rounds(),
                                                      "" if evidence.rounds() == 1 else "s"))
            moves = plan_step(lane, here, target, cfg.style)
            if cfg.drive == "each":
                self._show("Going to the %s square." % name, "moving", active_row=None)
                self._drive(moves, target)
                self._wait(cfg.settle_s)
            else:
                pending.append((moves, target))
            here = target
            row += 1
        if pending:
            self._show("Path traced. The mouse is driving it.", "moving")
            for moves, target in pending:
                self._drive(moves, target)
                self._show("Path traced. The mouse is driving it.", "moving")
        self.outcome = "finished"

    # ------------------------------------------------------------------ run
    def run(self) -> dict:
        cfg = self.config
        try:
            end = self.clock.now() + cfg.ready_timeout_s
            while not self.display.ready():
                if self.clock.now() > end:
                    raise SessionError("the headset page never connected")
                self._tick()
                self.clock.sleep(0.05)
            self._show("Put the mouse on the green square, facing up the lane. Then press Start.", "wait")
            if cfg.wait_start:
                self.say("waiting for Start (the button on the headset page)...")
                while self._await(lambda e: e["type"] == "button" and e.get("name") == "start", 1.0) is None:
                    pass
            if self.mouse is not None:
                self.mouse.connect()
                self.mouse.zero()
                self.mouse.watch = self._tick        # Stop on the headset also interrupts a move
            if cfg.calib_blocks:
                self.calibration = self.calibrate()
            self.trace()
        except Stopped as exc:
            self.outcome = str(exc)
        except (MouseError, SessionError) as exc:
            self.outcome = "error: %s" % exc
        finally:
            if self.mouse is not None:
                self.mouse.stop()
            try:
                self._show("Done: %s." % self.outcome, "done")
            except Exception:
                pass
        summary = self.summary()
        self._record("summary", **summary)
        return summary

    def summary(self) -> dict:
        cfg = self.config
        return {"outcome": self.outcome, "path": [list(c) for c in self.path],
                "mouse": list(self.mouse_cell), "decisions": self.decisions,
                "calibration": self.calibration, "lane": asdict(cfg.lane),
                "flash_s": cfg.flash_s, "style": cfg.style, "drive": cfg.drive}

    def board(self) -> str:
        return board(self.config.lane, self.mouse_cell, self.path)
