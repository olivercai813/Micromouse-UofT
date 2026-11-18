"""Steering from the tilt of the head.

The headband's accelerometer gives the direction of gravity in the
headband's own frame. Three short holds (head upright, tilted left, tilted
right) tell which direction in that frame is "sideways", so nothing has to
be assumed about how the headband sits. The lean is then a signed number,
+1 at the calibrated left tilt and -1 at the right one, and the wheel on the
inside of the turn slows down in proportion to it.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .board import STOP, Wheels


class TiltError(Exception):
    """The three holds cannot be turned into a steering axis."""


class Gravity:
    """The newest gravity vector, smoothed over about `window` seconds."""

    def __init__(self, window=0.3, stale=1.0):
        self.window, self.stale = window, stale
        self._vector = None
        self._at = None

    def update(self, now, xyz):
        xyz = np.asarray(xyz, float)[:3]
        if self._at is None or now - self._at > self.stale:
            self._vector = xyz.copy()
        else:
            self._vector += (1.0 - math.exp(-(now - self._at) / self.window)) * (xyz - self._vector)
        self._at = now

    def value(self, now):
        """The vector in g, or None when the accelerometer has gone quiet."""
        if self._at is None or now - self._at > self.stale:
            return None
        return self._vector.copy()


@dataclass
class TiltCalibration:
    centre: np.ndarray     # gravity with the head upright
    axis: np.ndarray       # unit vector from the right tilt toward the left
    reach: float           # how far along the axis a calibrated tilt goes

    def lean(self, gravity):
        """+1 at the calibrated left tilt, -1 at the right, 0 upright."""
        return float((gravity - self.centre) @ self.axis / self.reach)

    def to_dict(self):
        return {"centre": self.centre.tolist(), "axis": self.axis.tolist(),
                "reach": self.reach}

    @classmethod
    def from_dict(cls, saved):
        return cls(np.array(saved["centre"]), np.array(saved["axis"]), saved["reach"])


def fit_tilt(upright, left, right, least=5):
    """A TiltCalibration from the gravity vectors of the three holds."""
    if min(len(upright), len(left), len(right)) < least:
        raise TiltError("The motion data stopped")
    centre = np.median(np.asarray(upright, float), axis=0)
    at_left = np.median(np.asarray(left, float), axis=0)
    at_right = np.median(np.asarray(right, float), axis=0)
    axis = at_left - at_right
    span = float(np.linalg.norm(axis))
    if span < 0.2:      # in g; about 6 degrees each way
        raise TiltError("The left and right tilts looked the same")
    axis = axis / span
    reach = (abs((at_left - centre) @ axis) + abs((at_right - centre) @ axis)) / 2.0
    return TiltCalibration(centre, axis, float(max(reach, span / 4.0)))


def shape(lean, dead=0.2):
    """Lean to turn: nothing inside the dead zone, then linear up to a
    full turn (+1 left, -1 right) at the calibrated tilt."""
    size = min(max((abs(lean) - dead) / (1.0 - dead), 0.0), 1.0)
    return math.copysign(size, lean) if size else 0.0


def mix(go, turn, speed, slowest=1.0 / 3.0):
    """Wheel duties for a car that should move (`go`) while turning by
    `turn`: the outer wheel keeps `speed`, the inner one falls to
    `slowest` of it at a full turn."""
    if not go:
        return STOP
    inner = round(speed * (1.0 - (1.0 - slowest) * min(abs(turn), 1.0)))
    if turn > 0:
        return Wheels(inner, speed)
    if turn < 0:
        return Wheels(speed, inner)
    return Wheels(speed, speed)
