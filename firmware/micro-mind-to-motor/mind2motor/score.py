"""How well did the car do what the wearer was asked to do?

Reads the ticks.csv of a cued test and reports, for the focus / relax cues:

  steady accuracy   over the second half of every cue (the first half is
                    left for the wearer to get there): the share of ticks
                    on which the car was moving under "focus" and still
                    under "relax", the two averaged so that neither cue
                    counts for more. With a 95% interval from leaving out
                    one cue at a time.
  chance            the same number after sliding what the car did along
                    the cues by every possible delay. A car that pays no
                    attention to the wearer scores the same at any delay,
                    so the share of delays that do as well as the real
                    alignment is the p-value. The slid series keeps the
                    car's own rhythm of starts and stops, which a shuffle
                    of single ticks would destroy. It is a cautious test:
                    it needs cues of clearly different lengths, and gives
                    no p-value for fewer than six cues.
  reached           cues after which the car got to the asked state within
                    6 s, held it for 1 s, and was mostly there in the
                    second half; with the time that took
  bits              information per cue and per minute (Wolpaw), from the
                    share of cues whose second half was mostly right
  false starts      starts per minute during the second half of "relax"
  dropouts          stops per minute during the second half of "focus"
  level AUC         how separable the decoder's level was between the two
                    cues: tells a poor signal from poor thresholds
  link              how often the board's own status was on record

The car's state is taken from the board's status line where there is one,
else from the frame that was sent. Cues interrupted by a pause are left out.
Steering cues (left, right, centre) are scored the same way on the sign of
the decoded turn.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

from .decode import auc

REACH_S = 6.0       # a cue counts as reached if the car gets there this fast
HOLD_S = 1.0        # and stays there this long
MOST_SHIFTS = 2000
FEWEST_FOR_P = 6    # focus and relax cues needed before a p-value is given


def _number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _moving(row):
    """Was the car moving on this tick; (answer, from the board?)."""
    left, right = _number(row.get("ack_left")), _number(row.get("ack_right"))
    if left is not None and right is not None:
        return left + right > 0, True
    return (_number(row.get("left")) or 0) + (_number(row.get("right")) or 0) > 0, False


def _steering(row):
    turn = _number(row.get("turn"))
    return None if turn is None else (turn > 0) - (turn < 0)


WANTED_TURN = {"left": 1, "right": -1, "centre": 0}


class Trial:
    def __init__(self, kind, rows, interrupted):
        self.kind, self.rows, self.interrupted = kind, rows, interrupted
        self.t = np.array([float(r["t"]) for r in rows]) - float(rows[0]["t"])
        if kind in WANTED_TURN:
            self.right = np.array([_steering(r) == WANTED_TURN[kind] for r in rows])
            self.state = np.array([_steering(r) or 0 for r in rows])
        else:
            self.state = np.array([_moving(r)[0] for r in rows])
            self.right = self.state == (kind == "focus")
        self.steady = self.t >= self.t[-1] / 2.0

    def reached_after(self):
        """Seconds until the asked state was first held for HOLD_S; None if
        never, or if the second half of the cue was mostly wrong (a car
        that was still in the asked state from the cue before, and then
        left it, has not reached anything)."""
        if self.right[self.steady].mean() <= 0.5:
            return None
        for start in np.flatnonzero(self.right):
            if self.t[start] + HOLD_S > self.t[-1]:
                break
            if self.right[(self.t >= self.t[start]) & (self.t < self.t[start] + HOLD_S)].all():
                return float(self.t[start])
        return None


def trials_of(rows):
    """The cues of a log, in order. A cue is interrupted when the row after
    its last one is neither driving nor the end of the run."""
    found, current = [], None
    for index, row in enumerate(rows):
        label = row.get("trial") if row.get("stage") == "driving" else ""
        if label and (current is None or label != current[0]):
            current = (label, row["cue"], [])
            found.append(current)
        if label:
            current[2].append((index, row))
        else:
            current = None
    trials = []
    for _, kind, members in found:
        after = members[-1][0] + 1
        interrupted = after < len(rows) and rows[after].get("stage") not in ("driving", "done")
        trials.append(Trial(kind, [row for _, row in members], interrupted))
    return trials


def _balanced(trials, kinds):
    """Mean over the cue kinds of the share of steady ticks that were right."""
    shares = []
    for kind in kinds:
        right = sum(int(t.right[t.steady].sum()) for t in trials if t.kind == kind)
        total = sum(int(t.steady.sum()) for t in trials if t.kind == kind)
        if total:
            shares.append(right / total)
    return float(np.mean(shares)) if shares else float("nan")


def _leave_one_out(trials, kinds):
    """95% interval of the balanced accuracy by the jackknife over cues."""
    if len(trials) < 3:
        return None
    parts = np.array([_balanced(trials[:i] + trials[i + 1:], kinds)
                      for i in range(len(trials))])
    parts = parts[np.isfinite(parts)]
    count = len(parts)
    error = math.sqrt((count - 1) / count * float(((parts - parts.mean()) ** 2).sum()))
    middle = _balanced(trials, kinds)
    return [round(max(middle - 1.96 * error, 0.0), 3), round(min(middle + 1.96 * error, 1.0), 3)]


def _slid(trials):
    """Balanced accuracy of the focus / relax cues with the car's record
    slid along them by each delay; returns (delays tried, share at least as
    good as the real alignment)."""
    if len(trials) < FEWEST_FOR_P:
        return 0, None      # a handful of cues fits too many delays by luck
    wanted = np.concatenate([np.full(len(t.t), t.kind == "focus") for t in trials])
    steady = np.concatenate([t.steady for t in trials])
    moving = np.concatenate([t.state for t in trials]).astype(bool)
    focus, relax = steady & wanted, steady & ~wanted
    if not focus.any() or not relax.any():
        return 0, None

    def score(record):
        return (record[focus].mean() + (~record[relax]).mean()) / 2.0

    real = score(moving)
    margin = max(1, min(50, len(moving) // 10))      # delays near zero are not a different alignment
    delays = np.arange(margin, len(moving) - margin)
    if len(delays) > MOST_SHIFTS:
        delays = delays[np.linspace(0, len(delays) - 1, MOST_SHIFTS).astype(int)]
    if not len(delays):
        return 0, None
    as_good = sum(score(np.roll(moving, int(delay))) >= real - 1e-12 for delay in delays)
    return len(delays), (1 + as_good) / (1 + len(delays))


def _wilson(hits, total):
    if not total:
        return None
    z, share = 1.96, hits / total
    centre = (share + z * z / (2 * total)) / (1 + z * z / total)
    half = z * math.sqrt(share * (1 - share) / total + z * z / (4 * total * total)) / (1 + z * z / total)
    return [round(max(centre - half, 0.0), 3), round(min(centre + half, 1.0), 3)]


def _bits(share):
    """Information per two-way choice made with this accuracy (Wolpaw)."""
    if share <= 0.5:
        return 0.0
    if share >= 1.0:
        return 1.0
    return 1.0 + share * math.log2(share) + (1.0 - share) * math.log2(1.0 - share)


def _block(trials, kinds):
    """The numbers for one family of cues."""
    trials = [t for t in trials if t.kind in kinds]
    if not trials:
        return None
    reach = [t.reached_after() for t in trials]
    hits = sum(1 for r in reach if r is not None and r <= REACH_S)
    mostly_right = float(np.mean([t.right[t.steady].mean() > 0.5 for t in trials]))
    minutes = sum(float(t.t[-1]) for t in trials) / 60.0
    by_cue = {}
    for kind in kinds:
        own = [t for t in trials if t.kind == kind]
        if not own:
            continue
        times = [r for r in (t.reached_after() for t in own) if r is not None]
        by_cue[kind] = dict(
            cues=len(own),
            steady_accuracy=round(_balanced(own, [kind]), 3),
            reached=sum(1 for r in (t.reached_after() for t in own)
                        if r is not None and r <= REACH_S),
            seconds_to_reach_median=round(float(np.median(times)), 2) if times else None)
    return dict(
        cues=len(trials),
        steady_accuracy=round(_balanced(trials, kinds), 3),
        steady_accuracy_ci95=_leave_one_out(trials, kinds),
        reached=hits, reached_ci95=_wilson(hits, len(trials)),
        cues_mostly_right=round(mostly_right, 3),
        bits_per_cue=round(_bits(mostly_right), 3),
        bits_per_minute=round(_bits(mostly_right) * len(trials) / minutes, 2) if minutes else None,
        by_cue=by_cue)


def _changes_per_minute(trials, kind, onset):
    """Starts (onset True) or stops per minute in the steady half of the
    cues of one kind."""
    count, seconds = 0, 0.0
    for trial in (t for t in trials if t.kind == kind):
        state = trial.state[trial.steady].astype(bool)
        times = trial.t[trial.steady]
        if len(state) < 2:
            continue
        count += int((state[1:] & ~state[:-1]).sum() if onset else (~state[1:] & state[:-1]).sum())
        seconds += float(times[-1] - times[0])
    return round(60.0 * count / seconds, 2) if seconds else None


def analyse(rows):
    """The report, as a dictionary, for the rows of a ticks.csv."""
    every = trials_of(rows)
    trials = [t for t in every if not t.interrupted and len(t.rows) >= 10]
    drive = [t for t in trials if t.kind in ("focus", "relax")]
    result = dict(cues_scored=len(trials), cues_interrupted=len(every) - len(trials))
    if drive:
        result["drive"] = _block(drive, ("focus", "relax"))
        tried, p_value = _slid(drive)
        result["drive"].update(
            chance=dict(delays_tried=tried,
                        p_value=None if p_value is None else round(p_value, 4)),
            false_starts_per_minute=_changes_per_minute(drive, "relax", True),
            dropouts_per_minute=_changes_per_minute(drive, "focus", False))
        levels = {kind: [v for t in drive if t.kind == kind
                         for v in (_number(r.get("level")) for r, s in zip(t.rows, t.steady) if s)
                         if v is not None] for kind in ("focus", "relax")}
        separability = auc(levels["focus"], levels["relax"])
        result["drive"]["level_auc"] = None if math.isnan(separability) else round(separability, 3)
        ticks = [r for t in drive for r in t.rows]
        result["link"] = dict(
            ticks=len(ticks),
            with_board_status=round(float(np.mean([_moving(r)[1] for r in ticks])), 3))
    steer = _block(trials, ("left", "right", "centre"))
    if steer:
        result["steer"] = steer
    return result


def describe(result):
    """The report as a few lines of text."""
    lines = [f"{result['cues_scored']} cues scored"
             + (f", {result['cues_interrupted']} interrupted and left out"
                if result["cues_interrupted"] else "") + "."]
    drive = result.get("drive")
    if drive:
        interval = drive["steady_accuracy_ci95"]
        lines.append(
            f"Car matched the cue on {100 * drive['steady_accuracy']:.0f}% of steady ticks"
            + (f" (95% interval {100 * interval[0]:.0f}-{100 * interval[1]:.0f}%)" if interval else "")
            + "; 50% is what a car ignoring the wearer gets.")
        chance = drive["chance"]
        if chance["p_value"] is not None:
            lines.append(f"Slid along the cues by {chance['delays_tried']} delays, the record "
                         f"did as well as that in a share {chance['p_value']:.4f} of them "
                         "(the p-value).")
        else:
            lines.append(f"No p-value: it takes at least {FEWEST_FOR_P} focus and relax cues.")
        lines.append(f"Reached the asked state within {REACH_S:.0f} s after "
                     f"{drive['reached']} of {drive['cues']} cues; "
                     f"{drive['bits_per_cue']:.2f} bits per cue, "
                     f"{drive['bits_per_minute']} bits per minute.")
        for kind, numbers in drive["by_cue"].items():
            lines.append(f"  {kind:6s} {numbers['cues']} cues: steady accuracy "
                         f"{100 * numbers['steady_accuracy']:.0f}%, reached {numbers['reached']}, "
                         f"median {numbers['seconds_to_reach_median']} s to get there")
        lines.append(f"False starts while relaxing: {drive['false_starts_per_minute']} per minute; "
                     f"dropouts while focusing: {drive['dropouts_per_minute']} per minute.")
        if drive["level_auc"] is not None:
            lines.append(f"Decoder level, focus against relax: AUC {drive['level_auc']:.2f} "
                         "(0.5 none, 1.0 perfect).")
        lines.append(f"Board status on record for {100 * result['link']['with_board_status']:.0f}% "
                     "of those ticks; the rest were scored on the frame sent.")
    steer = result.get("steer")
    if steer:
        lines.append(f"Steering: decoded turn matched the cue on "
                     f"{100 * steer['steady_accuracy']:.0f}% of steady ticks; reached "
                     f"{steer['reached']} of {steer['cues']} cues.")
        for kind, numbers in steer["by_cue"].items():
            lines.append(f"  {kind:6s} {numbers['cues']} cues: steady accuracy "
                         f"{100 * numbers['steady_accuracy']:.0f}%, "
                         f"median {numbers['seconds_to_reach_median']} s to get there")
    return "\n".join(lines)


def report(path, write=True):
    """Score a run folder or a ticks.csv; returns the result and writes
    report.json beside the log."""
    path = Path(path)
    if path.is_dir():
        path = path / "ticks.csv"
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    result = analyse(rows)
    if write and result["cues_scored"]:
        (path.parent / "report.json").write_text(json.dumps(result, indent=2))
    return result
