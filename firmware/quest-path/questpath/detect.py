"""Was the wearer looking at the square that was flickering?

A square flickering at f Hz in front of the eyes drives the visual cortex at f
and its harmonics (the steady-state visual evoked potential, SSVEP). The
response is strongest for the square the wearer is looking at.

One flash gives one score:
  1. the EEG of the flash is kept between 6 and 40 Hz;
  2. canonical correlation finds the mix of electrodes that best follows
     sines and cosines at f and 2f, and how well it follows them (rho);
  3. the same is done at nearby rates where nothing flickers; the score is
     how far rho at f stands above those, in units of their spread (z).

A row is decided from the scores of its squares over one or more rounds: the
square in front must be clearly above nothing AND clearly above the runner-up,
since the neighbours of the square being looked at respond a little too.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

CONTROL_STEPS_HZ = (1.3, 2.1, 2.9, 3.7, 4.5, 5.3, 6.1)
ALPHA_BAND_HZ = (8.0, 11.6)
CONTROLS = 6


def control_rates(hz: float, low: float, top: float):
    """Rates near the flicker rate at which nothing flickers, to compare
    against: the six nearest that stay inside the band and outside the alpha
    band, where the brain's own idle rhythm would pass for a response."""
    options = []
    for step in CONTROL_STEPS_HZ:
        for rate in (hz + step, hz - step):
            if low + 0.5 <= rate <= top - 0.5 and not ALPHA_BAND_HZ[0] <= rate <= ALPHA_BAND_HZ[1] \
                    and abs(rate - 2 * hz) > 1.0 and abs(rate - hz / 2) > 1.0:
                options.append((step, rate))
    return tuple(rate for _, rate in sorted(options)[:CONTROLS])


@dataclass
class DetectConfig:
    low_hz: float = 6.0
    high_hz: float = 40.0
    harmonics: int = 2
    skip_s: float = 0.5           # start of a flash left out: onset and transport delay
    tail_s: float = 0.1           # kept after the flash ends, for the same delay
    min_cover: float = 0.85       # a flash with more of its EEG missing is not scored
    max_rms_uv: float = 400.0     # an electrode noisier than this is left out
    threshold: float = 3.0        # the winner's combined z must reach this ...
    margin: float = 1.5           # ... and beat the runner-up's by this
    min_rounds: int = 1
    max_rounds: int = 4


def bandlimit(x: np.ndarray, rate: float, low: float, high: float, whiten: bool = True) -> np.ndarray:
    """Keep low..high Hz of each column (linear trend removed, ends tapered).

    With `whiten`, each column's spectrum is first divided by its own running
    median over about 5 Hz. EEG is not flat: it is stronger at low rates and
    has an alpha hump near 10 Hz, so without this a rate near 10 Hz would look
    like a response whenever the wearer relaxes. A flicker response is one
    narrow line and survives the median untouched."""
    n = len(x)
    t = np.linspace(-1.0, 1.0, n)
    design = np.stack([np.ones(n), t], axis=1)
    x = x - design @ np.linalg.lstsq(design, x, rcond=None)[0]
    ramp = max(2, int(0.05 * n))
    taper = np.ones(n)
    taper[:ramp] = 0.5 - 0.5 * np.cos(np.pi * np.arange(ramp) / ramp)
    taper[-ramp:] = taper[:ramp][::-1]
    spectrum = np.fft.rfft(x * taper[:, None], axis=0)
    freqs = np.fft.rfftfreq(n, 1.0 / rate)
    if whiten:
        half = max(2, int(round(2.5 * n / rate)))          # bins in 2.5 Hz
        size = np.abs(spectrum)
        padded = np.pad(size, ((half, half), (0, 0)), mode="edge")
        windows = np.lib.stride_tricks.sliding_window_view(padded, 2 * half + 1, axis=0)
        level = np.median(windows, axis=-1)
        spectrum = spectrum / np.maximum(level, 1e-12 * max(1.0, float(size.max())))
    spectrum[(freqs < low) | (freqs > high)] = 0.0
    return np.fft.irfft(spectrum, n=n, axis=0)


def references(n: int, rate: float, hz: float, harmonics: int, top_hz: float) -> np.ndarray:
    t = np.arange(n) / rate
    cols = []
    for h in range(1, harmonics + 1):
        if h * hz <= top_hz:
            cols += [np.sin(2 * np.pi * h * hz * t), np.cos(2 * np.pi * h * hz * t)]
    return np.stack(cols, axis=1)


def canonical_correlation(x: np.ndarray, y: np.ndarray) -> float:
    """Largest canonical correlation between the columns of x and of y."""
    qx, rx = np.linalg.qr(x - x.mean(axis=0))
    qy, _ = np.linalg.qr(y - y.mean(axis=0))
    keep = np.abs(np.diag(rx)) > 1e-9 * max(1.0, np.abs(rx).max())
    if not keep.any():
        return 0.0
    s = np.linalg.svd(qx[:, keep].T @ qy, compute_uv=False)
    return float(min(1.0, s[0]))


def strength(rho):
    """log(explained / unexplained variance) for a correlation rho. On noise
    this is far closer to a bell curve than rho itself, whose long upper tail
    would let noise pass for a response a few times in a hundred."""
    r2 = np.clip(np.asarray(rho, float) ** 2, 1e-6, 1.0 - 1e-6)
    return np.log(r2 / (1.0 - r2))


@dataclass
class FlashScore:
    ok: bool
    rho: float = 0.0
    control: tuple = ()
    excess: float = 0.0
    z: float = 0.0
    channels: int = 0
    why: str = ""


class Scorer:
    """Scores flashes and keeps the running spread of the control rates, so
    that z means the same thing from one flash to the next."""

    def __init__(self, config: DetectConfig | None = None):
        self.config = config or DetectConfig()
        self.channels = None                 # electrodes to use; None = every usable one
        self._deviations = []

    @staticmethod
    def _spread_of(deviations) -> float:
        if len(deviations) < 12:
            return 0.8                       # a typical value until there is data
        d = np.asarray(deviations[-600:])
        return float(max(1.4826 * np.median(np.abs(d - np.median(d))), 0.1))

    def spread(self) -> float:
        return self._spread_of(self._deviations)

    def _measure(self, x: np.ndarray, rate: float, hz: float, channels):
        """(rho at the flicker rate, rho at each control rate, electrodes
        used), or a string saying why the flash cannot be scored."""
        c = self.config
        if x.ndim != 2 or len(x) < int(0.6 * rate) or rate <= 0:
            return "too little EEG"
        top = min(c.high_hz, 0.45 * rate)
        others = control_rates(hz, c.low_hz, top)
        if not c.low_hz < hz < top or len(others) < 3:
            return "flicker rate outside the band"
        raw = bandlimit(x, rate, c.low_hz, top, whiten=False)
        rms = np.sqrt((raw ** 2).mean(axis=0))
        good = (rms > 1e-6) & (rms < c.max_rms_uv)       # flat or wildly noisy electrodes out
        if channels is not None:
            wanted = np.zeros(x.shape[1], bool)
            wanted[[k for k in channels if k < x.shape[1]]] = True
            if (good & wanted).any():
                good &= wanted                           # else fall back on whatever works
        if not good.any():
            return "no usable electrode"
        y = bandlimit(x[:, good], rate, c.low_hz, top)
        n = len(y)
        rho = canonical_correlation(y, references(n, rate, hz, c.harmonics, top))
        control = tuple(canonical_correlation(y, references(n, rate, other, c.harmonics, top))
                        for other in others)
        return rho, control, int(good.sum())

    @staticmethod
    def _excess(rho, control):
        """How far the flicker rate stands above the control rates, scaled so
        that dividing by the spread of the returned deviations gives a z-score:
        a control's deviation from the controls' own mean is a little narrower
        than one rate's deviation from the mean of n others."""
        levels = strength(control)
        n = len(levels)
        mean = float(levels.mean())
        deviations = [float(v - mean) / np.sqrt(1.0 - 1.0 / n) for v in levels]
        return float((strength(rho) - mean) / np.sqrt(1.0 + 1.0 / n)), deviations

    def score(self, x: np.ndarray, rate: float, hz: float) -> FlashScore:
        measured = self._measure(x, rate, hz, self.channels)
        if isinstance(measured, str):
            return FlashScore(False, why=measured)
        rho, control, used = measured
        excess, deviations = self._excess(rho, control)
        self._deviations.extend(deviations)
        return FlashScore(True, rho, control, excess, excess / self.spread(), used)

    def choose_channels(self, flicker, steady, width: int):
        """Pick the electrodes that tell flicker from no flicker best.

        `flicker` and `steady` are lists of (EEG, rate, flicker rate) from the
        signal check. Tried: every electrode alone, the best two together, and
        all of them; the choice with the widest gap between flicker and no
        flicker (in units of its own noise) is kept. An electrode with poor
        contact, or one that only picks up blinks, is left out this way.
        Returns the gap of each choice."""
        def separation(channels):
            deviations, on, off = [], [], []
            for group, sink in ((flicker, on), (steady, off)):
                for x, rate, hz in group:
                    measured = self._measure(x, rate, hz, channels)
                    if isinstance(measured, str):
                        continue
                    rho, control, _ = measured
                    excess, devs = self._excess(rho, control)
                    deviations.extend(devs)
                    sink.append(excess)
            if len(on) < 2 or len(off) < 2:
                return float("-inf"), deviations
            return float((np.median(on) - np.median(off)) / self._spread_of(deviations)), deviations

        results = {}
        singles = []
        for k in range(width):
            results[(k,)] = separation([k])
            singles.append((results[(k,)][0], k))
        singles.sort(reverse=True)
        if width >= 3:
            pair = tuple(sorted(k for _, k in singles[:2]))
            results[pair] = separation(list(pair))
        results[tuple(range(width))] = separation(None)
        best = max(results, key=lambda key: (round(results[key][0], 1), -len(key)))
        if results[best][0] == float("-inf"):
            return {}
        self.channels = None if len(best) == width else list(best)
        self._deviations = list(results[best][1])
        return {key: value[0] for key, value in results.items()}


@dataclass
class RowEvidence:
    """Scores of one row's squares, round by round."""
    cols: int
    config: DetectConfig = field(default_factory=DetectConfig)
    scores: dict = field(default_factory=dict)

    def add(self, col: int, z: float) -> None:
        self.scores.setdefault(col, []).append(z)

    def combined(self):
        """Per square: sum of z over its flashes / sqrt(count), which stays
        near N(0, 1) for a square nobody looks at however many rounds run."""
        return {col: float(np.sum(z) / np.sqrt(len(z))) for col, z in self.scores.items() if z}

    def rounds(self) -> int:
        return min((len(self.scores.get(col, [])) for col in range(self.cols)), default=0)

    def decide(self):
        """("pick", col), ("none", None) or ("more", None)."""
        c = self.config
        done = self.rounds()
        z = self.combined()
        if done >= c.min_rounds and len(z) == self.cols:
            order = sorted(z, key=z.get, reverse=True)
            best, second = order[0], order[1]
            if z[best] >= c.threshold and z[best] - z[second] >= c.margin:
                return "pick", best
        if done >= c.max_rounds:
            return "none", None
        return "more", None
