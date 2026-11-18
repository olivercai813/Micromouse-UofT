"""One run, from finding the headband to driving the car.

A Session is advanced by calling tick(now, key) about ten times a second.
It never reads a clock, a keyboard or a port itself: the time and the key
are passed in, and the headband, the car and the operator's console are
objects handed to it. So the same code drives the real car in real time and
is rehearsed in tests at thousands of ticks a second.

Stages, in order:

  search    wait for a signal with at least one electrode in contact
  ready     wait for SPACE
  relax     record the relaxed state           (calibration; relax and focus
  focus     record the focused state            alternate for Config.rounds)
  upright, left, right                         (steering calibration)
  paused    decode and show, wheels held still; SPACE lets the car move
  driving   the car follows the wearer; SPACE pauses, R recalibrates
  done

With a cued test (Config.trials) the wearer is asked, while driving, to
focus and to relax in turn, each for a time drawn at random, and every tick
records the cue beside what the car did. See score.py.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import random
from typing import NamedTuple

from .board import STOP
from .decode import CalibrationError, Latch, fit
from .dsp import ELECTRODES, OK
from .tilt import TiltError, fit_tilt, mix, shape


class Stage(Enum):
    SEARCH = "search"
    READY = "ready"
    RELAX = "relax"
    FOCUS = "focus"
    UPRIGHT = "upright"
    LEFT = "left"
    RIGHT = "right"
    PAUSED = "paused"
    DRIVING = "driving"
    DONE = "done"


@dataclass
class Config:
    rounds: int = 2               # calibration rounds: relax then focus, each time
    calibrate_s: float = 15.0     # length of each calibration recording
    settle_s: float = 3.0         # start of each recording that is not used
    go: float = 0.65              # level at which the car starts
    stop: float = 0.35            # level at which it stops
    smooth: float = 1.0           # seconds of smoothing on the level
    rest: float = 0.5             # shortest stop, seconds
    shrink: float = 0.25          # see decode.fit
    speed: int = 75               # wheel duty when going straight, percent
    steer: bool = True
    armed: bool = False           # no waiting for SPACE anywhere
    trials: int = 0               # cued test: this many focus and relax cues
    steer_trials: int = 0         # and this many left, right and centre cues
    hold: tuple = (9.0, 17.0)     # a focus or relax cue lasts this long, seconds
    steer_hold: tuple = (6.0, 9.0)


class Cue(NamedTuple):
    kind: str           # focus, relax, left, right or centre
    seconds: float


PROMPTS = {"focus": "Focus. Eyes open, count backwards in sevens.",
           "relax": "Relax. Eyes closed, breathe slowly.",
           "left": "Tilt your head left.",
           "right": "Tilt your head right.",
           "centre": "Head straight."}


def make_plan(rng, trials, steer_trials=0, hold=(9.0, 17.0), steer_hold=(6.0, 9.0)):
    """The cues of a test. Focus and relax alternate, since a cue has to
    ask for a change; which comes first is a coin flip and how long each
    lasts is drawn at random. The spread of the lengths is what lets the
    score tell a car that follows the cues from one that merely alternates
    at about the right pace, so it should not be made small. The steering
    cues follow, in random order with no cue twice in a row."""
    first = rng.random() < 0.5
    cues = [Cue("focus" if (i % 2 == 0) == first else "relax", rng.uniform(*hold))
            for i in range(2 * trials)]
    while steer_trials:
        left = {"left": steer_trials, "right": steer_trials, "centre": steer_trials}
        order = []
        while any(left.values()):
            choice = [kind for kind, count in left.items()
                      if count and kind != (order[-1] if order else None)]
            if not choice:
                break                   # only the last cue's kind remains: start over
            kind = rng.choices(choice, [left[k] for k in choice])[0]
            left[kind] -= 1
            order.append(kind)
        else:
            cues += [Cue(kind, rng.uniform(*steer_hold)) for kind in order]
            break
    return cues


class Session:
    POSE_S = 3.5          # each steering hold
    POSE_SKIP_S = 1.2     # its start, while the head is still moving

    def __init__(self, senses, car, ui, log, config=None, rng=None):
        self.senses, self.car, self.ui, self.log = senses, car, ui, log
        self.config = config or Config()
        self.rng = rng or random.Random()
        self.stage = Stage.SEARCH
        self.ended = ""               # why the run ended: quit, test, ...
        self.calibration = None
        self.tilt = None
        self.latch = Latch(self.config.go, self.config.stop,
                           self.config.smooth, self.config.rest)
        self.plan = None              # cues still to come
        self.cue = None
        self.trial = 0
        self.now = 0.0
        self._zero = None
        self._since = 0.0             # when the stage began
        self._cue_end = 0.0
        self._recent = deque(maxlen=20)
        self._round = 0
        self._kept = {}
        self._poses = {}
        self._failures = 0
        self._hinted = self._reminded = 0.0
        self._steps = {Stage.SEARCH: self._search, Stage.READY: self._ready,
                       Stage.RELAX: self._record, Stage.FOCUS: self._record,
                       Stage.UPRIGHT: self._pose, Stage.LEFT: self._pose,
                       Stage.RIGHT: self._pose, Stage.PAUSED: self._pilot,
                       Stage.DRIVING: self._pilot}

    # ------------------------------------------------------------ the tick

    def tick(self, now, key=""):
        self.now = now
        if self._zero is None:
            self._zero = self._since = self._hinted = self._reminded = now
        if self.stage is Stage.DONE:
            return
        if key in ("q", "\x1b"):
            self._end("quit")
            return
        look = self.senses.look(now)
        fields = {}
        wheels = self._steps[self.stage](look, key, fields) or STOP
        if self.stage is Stage.DONE:
            return
        if self.stage is not Stage.DRIVING:
            wheels = STOP
        reached = self.car.send(wheels, now)
        if not reached and self.stage is Stage.DRIVING and not self.config.armed:
            # The car must not set off by itself when the cable comes back.
            self._pause("The car is unplugged. Paused: press space when it is back.")
        self._write(look, fields)

    def _write(self, look, fields):
        driving = self.stage is Stage.DRIVING
        row = dict(t=f"{self.now - self._zero:.2f}", stage=self.stage.value,
                   armed=int(driving), left=self.car.sent.left,
                   right=self.car.sent.right, board=self.car.describe(self.now))
        if driving and self.cue is not None:
            row.update(cue=self.cue.kind, trial=self.trial)
        heard = self.car.ack(self.now)
        if heard is not None:
            row.update(ack_left=heard.wheels.left, ack_right=heard.wheels.right)
        if look is not None:
            row["contact"] = "".join(str(int(code)) for code in look.state)
            row.update({f"spread_{name}": round(float(value), 1)
                        for name, value in zip(ELECTRODES, look.spread)})
        row.update(fields)
        self.log.tick(**row)

    def _end(self, why):
        self.stage, self.ended = Stage.DONE, why
        self.car.send(STOP, self.now)
        self.log.tick(t=f"{self.now - self._zero:.2f}", stage="done", armed=0,
                      left=0, right=0, board=self.car.describe(self.now))

    def _enter(self, stage):
        self.stage, self._since = stage, self.now
        say, cue = self.ui.say, self.ui.cue
        if stage is Stage.READY:
            say("Press space to begin the calibration.")
            self._reminded = self.now
        elif stage is Stage.RELAX:
            self._kept[stage].append([])
            say(f"Round {self._round} of {self.config.rounds}. Close your eyes and relax, "
                "until the beep.")
            cue("relax", self.now, True)
        elif stage is Stage.FOCUS:
            self._kept[stage].append([])
            say("Open your eyes and count backwards from 300 in sevens, until the beep.")
            cue("focus", self.now, True)
        elif stage is Stage.UPRIGHT:
            self._poses = {Stage.UPRIGHT: [], Stage.LEFT: [], Stage.RIGHT: []}
            say("Now the steering. Keep your head straight.")
            cue("upright", self.now, True)
        elif stage in (Stage.LEFT, Stage.RIGHT):
            say(f"Tilt your head to the {stage.value} and hold it.")
            cue(stage.value, self.now, True)

    # ------------------------------------------------- before the calibration

    def _calibrate(self):
        """Begin the calibration from its first round."""
        cfg = self.config
        self._round = 1
        self._kept = {Stage.RELAX: [], Stage.FOCUS: []}
        self.plan = self.cue = None
        self.ui.note(f"Calibration: {cfg.rounds} rounds of relaxing and focusing, "
                     f"{cfg.calibrate_s:.0f} s each. Sit still, jaw and forehead loose.")
        self._enter(Stage.RELAX)

    def _search(self, look, key, fields):
        self._recent.append(look is not None and bool(look.usable.any()))
        if len(self._recent) == self._recent.maxlen and sum(self._recent) >= 15:
            self.ui.note("Headband signal found.")
            if self.config.armed:
                self._calibrate()
            else:
                self._enter(Stage.READY)
        elif look is not None:
            self.ui.status("Signal arriving; waiting for electrode contact.  "
                           + look.contact())
        else:
            self.ui.status("Waiting for the headband.  " + self.senses.trouble)
            if self.now - self._hinted > 10.0:
                self._hinted = self.now + 1e9       # said once
                self.ui.note("Nothing yet. Bluetooth: headband on and not connected "
                             "to a phone. Phone app: same Wi-Fi, streaming to this "
                             "computer's address, raw EEG at full rate.")

    def _ready(self, look, key, fields):
        if key == " ":
            self._calibrate()
            return
        self.ui.status((look.contact() if look else "no signal") + "   "
                       + self.car.describe(self.now))
        if look is not None and self.now - self._reminded > 15.0:
            self._reminded = self.now
            off = [name for name, code in zip(ELECTRODES, look.state) if code != OK]
            if off:
                self.ui.say(f"Check the contact at {', '.join(off)} before you start.")

    # ---------------------------------------------------------- calibration

    def _record(self, look, key, fields):
        elapsed = self.now - self._since
        if look is not None and elapsed >= self.config.settle_s:
            self._kept[self.stage][-1].append((look.log_power.copy(), look.usable.copy()))
        self.ui.status(f"{self.stage.value} {self._round}/{self.config.rounds}: "
                       f"{self.config.calibrate_s - elapsed:3.0f} s left   "
                       + (look.contact() if look else "NO SIGNAL") + "   "
                       + self.car.describe(self.now))
        if elapsed < self.config.calibrate_s:
            return
        self.ui.beep()
        if self.stage is Stage.RELAX:
            self._enter(Stage.FOCUS)
            return
        if self._round < self.config.rounds:
            self._round += 1
            self._enter(Stage.RELAX)
            return
        try:
            found = fit(self._kept[Stage.RELAX], self._kept[Stage.FOCUS],
                        shrink=self.config.shrink)
        except CalibrationError as problem:
            self._failures += 1
            self.ui.say(str(problem))
            if not self.config.armed:
                self._enter(Stage.READY)
            elif self._failures < 3:
                self._calibrate()
            else:
                self._end("calibration failed")
            return
        self.calibration = found
        self.log.meta(calibration=found.to_dict())
        self.ui.note(f"The two states are {found.separation:.1f} scatter widths apart. Fitted "
                     f"without them, {found.validation} were told apart "
                     f"{100 * found.held_out_accuracy:.0f}% of the time "
                     f"(AUC {found.held_out_auc:.2f})."
                     + (" With one round that number is too kind if the signal drifted."
                        if self.config.rounds < 2 else ""))
        if not found.held_out_accuracy >= 0.7:
            self.ui.say("The two states overlap a lot. R records them again.")
        if self.config.steer and self.senses.gravity.value(self.now) is not None:
            self._enter(Stage.UPRIGHT)
        else:
            if self.config.steer:
                self.ui.note("No motion data from the headband: no steering.")
            self._begin_driving()

    def _pose(self, look, key, fields):
        gravity = self.senses.gravity.value(self.now)
        elapsed = self.now - self._since
        problem = None
        if gravity is None:
            problem = "The motion data stopped"
        else:
            if elapsed >= self.POSE_SKIP_S:
                self._poses[self.stage].append(gravity)
            self.ui.status(f"tilt {self.stage.value}: {self.POSE_S - elapsed:3.0f} s left")
            if elapsed < self.POSE_S:
                return
            self.ui.beep()
            following = {Stage.UPRIGHT: Stage.LEFT, Stage.LEFT: Stage.RIGHT}.get(self.stage)
            if following:
                self._enter(following)
                return
            try:
                self.tilt = fit_tilt(self._poses[Stage.UPRIGHT], self._poses[Stage.LEFT],
                                     self._poses[Stage.RIGHT])
                self.log.meta(tilt=self.tilt.to_dict())
                self.ui.say("Steering calibrated. Head straight.")
            except TiltError as error:
                problem = str(error)
        if problem:
            self.tilt = None
            self.ui.say(problem + ": no steering.")
        self.ui.cue("upright", self.now, True)
        self._begin_driving()

    def _begin_driving(self):
        self.latch.reset()
        self.cue = None
        cfg = self.config
        steering = cfg.steer_trials if self.tilt is not None else 0
        if cfg.trials or steering:
            self.plan = deque(make_plan(self.rng, cfg.trials, steering,
                                        cfg.hold, cfg.steer_hold))
            self.ui.note(f"Cued test: {len(self.plan)} cues, starting when the car is live.")
        self.ui.note("SPACE = start / pause the car    R = recalibrate    Q = quit")
        if cfg.armed:
            self.stage = Stage.DRIVING
            self.ui.say("The car is live.")
        else:
            self.stage = Stage.PAUSED
            self.ui.say("Press space to let the car move.")

    # -------------------------------------------------------------- driving

    def _pause(self, text):
        if self.cue is not None:        # the interrupted cue is given again
            self.plan.appendleft(self.cue)
            self.cue = None
        self.stage = Stage.PAUSED
        self.ui.say(text)

    def _next_cue(self):
        """Move on when the current cue has run its time. False once the
        test is over."""
        if self.cue is not None and self.now < self._cue_end:
            return True
        if not self.plan:
            self.ui.say("Test finished.")
            self._end("test")
            return False
        self.cue = self.plan.popleft()
        self.trial += 1
        self._cue_end = self.now + self.cue.seconds
        self.ui.say(PROMPTS[self.cue.kind])
        self.ui.cue(self.cue.kind, self.now, False)
        return True

    def _pilot(self, look, key, fields):
        if key == " ":
            if self.stage is Stage.DRIVING:
                self._pause("Paused.")
            else:
                self.stage = Stage.DRIVING
                self.ui.say("The car is live. Focus to drive, relax to stop.")
        elif key == "r":
            self._calibrate()
            return STOP
        driving = self.stage is Stage.DRIVING
        if driving and self.plan is not None and not self._next_cue():
            return STOP
        raw = self.calibration.level(look.log_power, look.usable) if look else None
        go = self.latch.update(self.now, raw)
        lean = turn = None
        gravity = self.senses.gravity.value(self.now)
        if self.tilt is not None and gravity is not None:
            lean = self.tilt.lean(gravity)
            turn = shape(lean)
        fields.update(go=int(go))
        if raw is not None:
            fields.update(raw_level=round(raw, 3))
        if self.latch.level is not None:
            fields.update(level=round(self.latch.level, 3))
        if lean is not None:
            fields.update(lean=round(lean, 2), turn=round(turn, 2))

        level = self.latch.level
        if level is None:
            meter = "no usable signal from the headband  "
        else:
            filled = round(20 * min(max(level, 0.0), 1.0))
            meter = (f"level {level:5.2f} |{'#' * filled}{'.' * (20 - filled)}| "
                     f"{'GO  ' if go else 'stop'}")
        heading = ("" if turn is None else "<< left " if turn > 0
                   else " right >>" if turn < 0 else " straight")
        self.ui.status(f"[{'LIVE  ' if driving else 'PAUSED'}] {meter} {heading}  "
                       + (look.contact() if look else "NO SIGNAL") + "  "
                       + self.car.describe(self.now))
        return mix(go and driving, turn or 0.0, self.config.speed)
