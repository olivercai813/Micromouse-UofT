"""From band powers to "go" or "stop".

Calibration records the band powers of every electrode while the wearer
relaxes and while the wearer concentrates, the two in turn over two or more
rounds. From those it learns:

  a baseline per electrode, so each electrode is read as a change from its
      own usual level and losing one electrode does not shift the reading;
  one weight per band (a linear discriminant with a shrunk covariance), the
      contrast between bands that best tells the two recordings apart;
  where the relaxed and the focused recordings fall along that contrast.

Afterwards every tick gives a level: 0 at the relaxed recording, 1 at the
focused one. A latch with two thresholds turns the level into go / stop.

The calibration also reports how well the contrast, fitted without one
round, reads that round. That is the number to look at before letting the
car move.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


class CalibrationError(Exception):
    """The recordings cannot be turned into a decoder; the text says why."""


def auc(high, low):
    """Chance that a value drawn from `high` exceeds one drawn from `low`
    (ties count half): 0.5 for no difference, 1.0 for no overlap."""
    high, low = np.asarray(high, float), np.asarray(low, float)
    if not len(high) or not len(low):
        return float("nan")
    _, where, counts = np.unique(np.concatenate([high, low]),
                                 return_inverse=True, return_counts=True)
    last = np.cumsum(counts)
    rank = (last - (counts - 1) / 2.0)[where]      # ties share their mean rank
    total = rank[:len(high)].sum() - len(high) * (len(high) + 1) / 2.0
    return float(total / (len(high) * len(low)))


@dataclass
class Calibration:
    baseline: np.ndarray        # log10 power [band, electrode]; nan = left out
    weights: np.ndarray         # one per band, unit length
    relaxed: float              # the contrast at the relaxed recording
    focused: float              # and at the focused one
    spread: float               # robust scatter of the contrast within a state
    held_out_auc: float
    held_out_accuracy: float
    ticks: tuple                # usable ticks (relaxed, focused)
    validation: str = ""        # what was left out for the held-out numbers

    @property
    def separation(self):
        """Distance between the two states in units of their scatter."""
        return (self.focused - self.relaxed) / max(self.spread, 1e-12)

    def features(self, log_power, usable):
        """One number per band: the median over usable electrodes of each
        electrode's change from its baseline, after removing that
        electrode's change common to all bands (a gain change, such as a
        shifting contact, moves every band alike). None without a usable
        electrode."""
        keep = np.asarray(usable, bool) & ~np.isnan(self.baseline[0])
        if not keep.any():
            return None
        change = (log_power - self.baseline)[:, keep]
        change = change - change.mean(axis=0)
        return np.median(change, axis=1)

    def level(self, log_power, usable):
        """0 at the relaxed recording, 1 at the focused one; None without
        a usable electrode."""
        found = self.features(log_power, usable)
        if found is None:
            return None
        return float((found @ self.weights - self.relaxed) /
                     (self.focused - self.relaxed))

    def to_dict(self):
        return {"baseline": [[None if math.isnan(v) else round(float(v), 5) for v in row]
                             for row in self.baseline],
                "weights": [round(float(v), 5) for v in self.weights],
                "relaxed": self.relaxed, "focused": self.focused,
                "spread": self.spread, "separation": round(self.separation, 2),
                "held_out_auc": self.held_out_auc,
                "held_out_accuracy": self.held_out_accuracy,
                "validation": self.validation, "ticks": list(self.ticks)}

    @classmethod
    def from_dict(cls, saved):
        baseline = np.array([[np.nan if v is None else v for v in row]
                             for row in saved["baseline"]], float)
        return cls(baseline, np.array(saved["weights"], float), saved["relaxed"],
                   saved["focused"], saved["spread"], saved["held_out_auc"],
                   saved["held_out_accuracy"], tuple(saved["ticks"]),
                   saved.get("validation", ""))


def _baseline(relaxed, focused):
    """Usual log power of each electrode: the mean of its level in the two
    states, so a state recorded for longer does not count for more. An
    electrode usable for less than half of either state is left out (nan)."""
    means = []
    for ticks in (relaxed, focused):
        power = np.stack([log_power for log_power, _ in ticks])
        usable = np.stack([mask for _, mask in ticks]).astype(bool)
        share = usable.mean(axis=0)
        weight = usable[:, None, :]
        mean = (power * weight).sum(axis=0) / np.maximum(weight.sum(axis=0), 1)
        mean[:, share < 0.5] = np.nan
        means.append(mean)
    return (means[0] + means[1]) / 2.0


def _discriminant(relaxed, focused, shrink):
    """Unit weights w, the two medians along w, and the scatter along w."""
    mean_r, mean_f = relaxed.mean(axis=0), focused.mean(axis=0)
    dev_r, dev_f = relaxed - mean_r, focused - mean_f
    cov = (dev_r.T @ dev_r + dev_f.T @ dev_f) / max(len(relaxed) + len(focused) - 2, 1)
    size = len(cov)
    # Pull the covariance toward a sphere: the samples are few and follow
    # one another closely, and the features nearly sum to zero.
    cov = (1.0 - shrink) * cov + shrink * np.trace(cov) / size * np.eye(size)
    try:
        weights = np.linalg.solve(cov, mean_f - mean_r)
    except np.linalg.LinAlgError:
        weights = np.zeros(size)
    length = float(np.linalg.norm(weights))
    if not math.isfinite(length) or length < 1e-12:
        raise CalibrationError("Relaxed and focused looked the same.")
    weights = weights / length
    along_r, along_f = relaxed @ weights, focused @ weights
    centre_r, centre_f = float(np.median(along_r)), float(np.median(along_f))
    if centre_f - centre_r < 1e-9:
        raise CalibrationError("Relaxed and focused looked the same.")
    scatter = 1.4826 * float(np.median(np.abs(np.concatenate(
        [along_r - centre_r, along_f - centre_f]))))
    return weights, centre_r, centre_f, scatter


def _learn(relaxed, focused, shrink, least):
    """A Calibration from the ticks of each state, without the held-out
    numbers."""
    if len(relaxed) < least or len(focused) < least:
        raise CalibrationError("Too little signal during the calibration.")
    baseline = _baseline(relaxed, focused)
    if np.isnan(baseline[0]).all():
        raise CalibrationError("No electrode kept contact through the calibration.")
    blank = Calibration(baseline, np.zeros(len(baseline)), 0.0, 1.0, 1.0,
                        float("nan"), float("nan"), (0, 0))

    def table(ticks):
        rows = [blank.features(log_power, usable) for log_power, usable in ticks]
        return np.array([row for row in rows if row is not None])

    table_r, table_f = table(relaxed), table(focused)
    if len(table_r) < least or len(table_f) < least:
        raise CalibrationError("Too little usable signal during the calibration: "
                               "check the electrode contact.")
    weights, centre_r, centre_f, scatter = _discriminant(table_r, table_f, shrink)
    return Calibration(baseline, weights, centre_r, centre_f, scatter,
                       float("nan"), float("nan"), (len(table_r), len(table_f)))


def _pieces(recording, count, purge):
    """One recording cut into stretches, with `purge` ticks dropped between
    neighbours, since neighbouring ticks share signal."""
    edges = np.linspace(0, len(recording), count + 1).astype(int)
    return [recording[start + purge // 2:stop - purge // 2]
            for start, stop in zip(edges, edges[1:])]


def fit(relaxed, focused, shrink=0.25, least=30, pieces=4, purge=10):
    """A Calibration from the recordings of each state: `relaxed` and
    `focused` are lists with one recording per calibration round, each
    recording a list of (log power [band, electrode], usable [electrode])
    per tick, in time order.

    The held-out numbers come from fitting without one round and reading
    that round, each round in turn. A slow drift of the signal then cannot
    pass for a difference between the states, because every state was
    recorded both early and late. With a single round only stretches of
    the one recording can be left out, and a drift between the relaxed and
    the focused recording does inflate the numbers; `validation` says
    which of the two it was.
    """
    found = _learn([tick for recording in relaxed for tick in recording],
                   [tick for recording in focused for tick in recording], shrink, least)
    if len(relaxed) >= 2 and len(focused) >= 2:
        parts_r, parts_f, found.validation = relaxed, focused, "whole rounds"
    else:
        parts_r, parts_f = _pieces(relaxed[0], pieces, purge), _pieces(focused[0], pieces, purge)
        found.validation = "stretches of one recording"
    levels_r, levels_f = [], []
    for out in range(min(len(parts_r), len(parts_f))):
        try:
            rest = _learn([tick for part in parts_r[:out] + parts_r[out + 1:] for tick in part],
                          [tick for part in parts_f[:out] + parts_f[out + 1:] for tick in part],
                          shrink, least=10)
        except CalibrationError:
            continue
        levels_r += [v for v in (rest.level(*tick) for tick in parts_r[out]) if v is not None]
        levels_f += [v for v in (rest.level(*tick) for tick in parts_f[out]) if v is not None]
    if levels_r and levels_f:
        found.held_out_auc = auc(levels_f, levels_r)
        found.held_out_accuracy = float(((np.array(levels_r) < 0.5).mean()
                                         + (np.array(levels_f) >= 0.5).mean()) / 2.0)
    return found


class Latch:
    """Go above `go`, stop below `stop`, keep the last answer in between.

    The level is smoothed over about `smooth` seconds first. After a stop
    the car stays stopped for at least `rest` seconds; a stop itself is
    never delayed. Without a level for longer than `lost` seconds the
    answer is stop.
    """

    def __init__(self, go=0.65, stop=0.35, smooth=1.0, rest=0.5, lost=0.6):
        if not stop < go:
            raise ValueError("the stop threshold must be below the go threshold")
        self.go, self.stop, self.smooth, self.rest, self.lost = go, stop, smooth, rest, lost
        self.reset()

    def reset(self):
        self.on = False
        self.level = None          # the smoothed level, None when unknown
        self._seen = None          # time of the last level
        self._stopped = None       # time of the last stop

    def update(self, now, level):
        if level is None:
            if self._seen is None or now - self._seen > self.lost:
                self._set(False, now)
            if self._seen is None or now - self._seen > 2.0:
                self.level = None
            return self.on
        if self.level is None:
            self.level = level
        else:
            step = now - self._seen
            keep = math.exp(-step / self.smooth) if self.smooth > 0 else 0.0
            self.level = level + keep * (self.level - level)
        self._seen = now
        if self.level <= self.stop:
            self._set(False, now)
        elif self.level >= self.go and (self._stopped is None or
                                        now - self._stopped >= self.rest):
            self._set(True, now)
        return self.on

    def _set(self, on, now):
        if self.on and not on:
            self._stopped = now
        self.on = on
