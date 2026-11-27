"""How often does the row decision come out right? (not a test: a measurement)

    python tests/bench_detector.py [rows per condition]

Runs simulated rows for wearers of different strength and prints, for each,
the share of rows decided right, wrong and not at all, and the rounds used.
"""

import sys

import numpy as np

from questpath.clock import SimClock
from questpath.detect import DetectConfig
from questpath.display import SimDisplay
from questpath.eeg import EEGBuffer
from questpath.session import Session, SessionConfig
from questpath.wearer import SimWearer


def rows(n, gaze, seed, refresh=72.0, rate=256.0, detect=None, **wearer):
    clock = SimClock()
    display = SimDisplay(clock, refresh=refresh)
    eeg = EEGBuffer()
    looking = {"col": gaze}
    w = SimWearer(display, eeg, lambda row: looking["col"], seed=seed, rate=rate, **wearer)
    clock.listeners.append(w.advance)
    config = SessionConfig(wait_start=False, seed=seed, detect=detect or DetectConfig())
    session = Session(clock, display, eeg, None, config, say=lambda s: None)
    clock.sleep(3.0)                       # let the stream settle
    looking["col"] = 1
    session.calibrate()                    # learns which electrodes to trust
    looking["col"] = gaze
    out = []
    rng = np.random.default_rng(seed)
    for _ in range(n):
        if gaze == "any":
            looking["col"] = int(rng.integers(0, 3))
        col, evidence = session.choose(1)
        out.append((looking["col"], col, evidence.rounds()))
    return out


def report(name, results):
    n = len(results)
    right = sum(1 for want, got, _ in results if got is not None and got == want)
    none = sum(1 for _, got, _ in results if got is None)
    wrong = n - right - none
    rounds = np.mean([r for _, _, r in results])
    print("%-34s right %5.1f%%  wrong %5.1f%%  nothing %5.1f%%  rounds %.2f  (%d rows)"
          % (name, 100 * right / n, 100 * wrong / n, 100 * none / n, rounds, n))
    return right / n, wrong / n, none / n


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    report("looking, 3 uV response", rows(n, "any", 1))
    report("looking, 1.5 uV response", rows(n, "any", 2, look_uv=1.5))
    report("looking, 1 uV response", rows(n, "any", 3, look_uv=1.0))
    report("looking, 0.6 uV response", rows(n, "any", 4, look_uv=0.6))
    report("looking away (no response)", rows(n, None, 5))
    report("no response at all (noise)", rows(n, None, 6, away=0.0))
    report("no AUX electrode, 3 uV", rows(n, "any", 7, channels=4))
    report("64 Hz stream (app decimating)", rows(n, "any", 8, rate=64.0))
    report("90 Hz headset (15 Hz flicker)", rows(n, "any", 9, refresh=90.0))
    report("bursty delivery (0.5 s)", rows(n, "any", 10, burst=0.5))
