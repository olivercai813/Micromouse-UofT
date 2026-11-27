"""A made-up wearer, for rehearsals and tests: EEG that responds to the flicker.

The simulated person looks at one square of each row (or nowhere). While a
square flickers their visual cortex follows it: strongly if it is the square
they are looking at, weakly if it is a neighbour, hardly at all otherwise.
On top of that comes what a consumer headband really delivers: broadband
noise, a 10 Hz alpha rhythm, slow drift and mains hum, and a transport delay.

The numbers are deliberately modest (a few microvolts of response in ten of
noise); they are stand-ins, not measurements of anybody.
"""

from __future__ import annotations

import math

import numpy as np

# how much of the response each electrode sees: TP9, AF7, AF8, TP10, AUX (a
# cup electrode over the visual cortex, as on the lab's eye-as-a-camera rig)
WEIGHTS = (0.35, 0.08, 0.08, 0.35, 1.0)


class SimWearer:
    def __init__(self, display, eeg, gaze, rate: float = 256.0, channels: int = 5, seed: int = 0,
                 look_uv: float = 3.0, side: float = 0.25, away: float = 0.03, noise_uv: float = 8.0,
                 alpha_uv: float = 4.0, delay: float = 0.09, neural: float = 0.1, burst: float = 0.0):
        self.display = display
        self.eeg = eeg
        self.gaze = gaze if callable(gaze) else (lambda row, table=dict(gaze): table.get(row))
        self.rate = rate
        self.channels = channels
        self.rng = np.random.default_rng(seed)
        self.look_uv, self.side, self.away = look_uv, side, away
        self.noise_uv, self.alpha_uv = noise_uv, alpha_uv
        self.delay, self.neural, self.burst = delay, neural, burst
        self.n = None                    # number of the next sample
        self.t0 = None                   # time of sample 0
        self.alpha_phase = self.rng.uniform(0, 2 * math.pi)
        self.drift = np.zeros(channels)
        self.weights = np.array((WEIGHTS + (0.5,) * 8)[:channels])

    def _gain(self, epoch) -> float:
        if epoch.kind == "blank":
            return 0.0
        if epoch.kind == "calib":
            return self.look_uv
        looking = self.gaze(epoch.row)
        if looking is None:
            return self.look_uv * self.away
        apart = abs(looking - epoch.col)
        return self.look_uv * (1.0 if apart == 0 else self.side * 0.5 ** (apart - 1))

    def _response(self, t: float) -> float:
        """Cortical response at time t, microvolts at the best electrode."""
        total = 0.0
        for epoch in reversed(self.display.epochs):
            if epoch.t1 + self.neural + 0.3 < t:
                break                                   # earlier flashes are over
            start, end = epoch.t0 + self.neural, epoch.t1 + self.neural
            if start <= t < end:
                gain = self._gain(epoch)
                if gain:
                    rise = min(1.0, (t - start) / 0.2)
                    phase = 2 * math.pi * epoch.hz * (t - start)
                    total += gain * rise * (math.sin(phase) + 0.5 * math.sin(2 * phase + 0.7))
        return total

    def advance(self, now: float) -> None:
        """Produce every sample that would have reached the laptop by `now`."""
        if self.t0 is None:
            self.t0, self.n = now, 0
        limit = min(now - self.delay, self.display.known_until())
        dt = 1.0 / self.rate
        while self.t0 + self.n * dt <= limit:
            t = self.t0 + self.n * dt
            self.n += 1
            self.alpha_phase += 2 * math.pi * (10.0 + self.rng.normal(0, 0.6)) * dt
            self.drift += self.rng.normal(0, 0.15, self.channels)
            self.drift *= 0.999
            common = self.alpha_uv * math.sin(self.alpha_phase) + 5.0 * math.sin(2 * math.pi * 60.0 * t)
            sample = (self.rng.normal(0, self.noise_uv, self.channels) + self.drift + common
                      + self._response(t) * self.weights)
            arrival = t + self.delay
            if self.burst:
                arrival = math.ceil(arrival / self.burst) * self.burst
            self.eeg.push(arrival, sample)
