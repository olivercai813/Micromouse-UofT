"""Band powers from raw EEG, updated on every sample.

Each band has its own causal band-pass filter (eighth-order Butterworth, run
as four biquads). The filtered sample is squared and averaged with a short
exponential window, so a band power exists after every sample and no block
of signal has to be collected first. A notch ahead of the filters takes out
the mains hum.

The same object also judges each electrode from the last second of signal:
flat, stuck at a rail, far too noisy to be EEG, or usable.
"""

from __future__ import annotations

import math

import numpy as np

ELECTRODES = ("TP9", "AF7", "AF8", "TP10")
BANDS = {"theta": (4.0, 8.0), "alpha": (8.0, 13.0),
         "beta": (13.0, 30.0), "gamma": (30.0, 45.0)}

OK, FLAT, RAILED, WILD = range(4)
STATE_NAMES = ("ok", "flat", "railed", "wild")


def bandpass_sections(low, high, rate, poles=4):
    """Biquads, rows of [b0, b1, b2, a1, a2], forming a Butterworth
    band-pass with unit gain at the centre of the band. `poles` is the
    order of the low-pass it is derived from (even); the band-pass has
    twice that order and takes that many biquads."""
    if not 0.0 < low < high < rate / 2.0:
        raise ValueError(f"band {low}-{high} Hz does not fit a {rate} Hz stream")
    if poles < 2 or poles % 2:
        raise ValueError("the number of poles must be even")
    # Band edges moved to where the bilinear transform will put them back.
    edge_low = 2.0 * rate * math.tan(math.pi * low / rate)
    edge_high = 2.0 * rate * math.tan(math.pi * high / rate)
    width, centre_sq = edge_high - edge_low, edge_low * edge_high
    centre_hz = rate / math.pi * math.atan(math.sqrt(centre_sq) / (2.0 * rate))
    delay = np.exp(-2j * math.pi * centre_hz / rate)  # z^-1 at the centre
    sections = []
    # The low-pass poles above the real axis; their mirror images give the
    # conjugates, which each biquad below already includes.
    for k in range(poles // 2):
        pole = np.exp(1j * math.pi * (2 * k + poles + 1) / (2 * poles))
        root = np.sqrt((pole * width) ** 2 - 4.0 * centre_sq + 0j)
        for analog in ((pole * width + root) / 2.0, (pole * width - root) / 2.0):
            digital = (2.0 * rate + analog) / (2.0 * rate - analog)
            a1, a2 = -2.0 * digital.real, abs(digital) ** 2
            # Zeros at 0 Hz and at half the sample rate: numerator 1 - z^-2.
            gain = abs((1.0 - delay ** 2) / (1.0 + a1 * delay + a2 * delay ** 2))
            sections.append([1.0 / gain, 0.0, -1.0 / gain, a1, a2])
    return np.array(sections)


def notch_section(freq, rate, sharpness=12.0):
    """One biquad, [b0, b1, b2, a1, a2], that removes a single frequency."""
    turn = 2.0 * math.pi * freq / rate
    alpha = math.sin(turn) / (2.0 * sharpness)
    middle = -2.0 * math.cos(turn)
    return np.array([1.0, middle, 1.0, middle, 1.0 - alpha]) / (1.0 + alpha)


def response(sections, freqs, rate):
    """Gain of a cascade of biquads at each frequency in `freqs` (Hz)."""
    delay = np.exp(-2j * np.pi * np.asarray(freqs, float) / rate)
    total = np.ones_like(delay)
    for b0, b1, b2, a1, a2 in np.atleast_2d(sections):
        total *= (b0 + b1 * delay + b2 * delay ** 2) / (1.0 + a1 * delay + a2 * delay ** 2)
    return np.abs(total)


class BandMeter:
    """Running band power of every electrode.

    rate    samples per second of the stream
    mains   hum frequency to notch out (60 in North America, 50 elsewhere,
            0 or None for no notch)
    window  time constant, in seconds, of the average over the squared signal
    rails   the two ends of the amplifier's range, in the stream's units
    """

    def __init__(self, rate, mains=60.0, window=0.5, bands=BANDS,
                 rails=(0.0, 1682.0)):
        self.rate = float(rate)
        self.names = tuple(bands)
        self.rails = rails
        coefficients = np.stack([bandpass_sections(low, high, self.rate)
                                 for low, high in bands.values()])
        self._coef = coefficients[..., None]          # [band, section, 5, 1]
        self._notch = notch_section(mains, self.rate) if mains else None
        self._gain = 1.0 - math.exp(-1.0 / (window * self.rate))
        self._warm = int(self.rate * max(1.5, 4.0 * window))
        self._size = int(round(self.rate))             # one second of history
        self.reset()

    def reset(self):
        """Forget everything, e.g. after a gap in the stream."""
        count = len(ELECTRODES)
        self._state = np.zeros((len(self.names), self._coef.shape[1], 2, count))
        self._notch_state = np.zeros((2, count))
        self._origin = None
        self._raw = np.zeros((self._size, count))
        self._clean = np.zeros((self._size, count))
        self._at = 0
        self.power = np.zeros((len(self.names), count))
        self.seen = 0

    @property
    def ready(self):
        """True once the filters and the averages have settled."""
        return self.seen >= self._warm

    def push(self, block):
        """Take in rows of samples, one column per electrode."""
        block = np.atleast_2d(np.asarray(block, dtype=float))
        if block.shape[1] != len(ELECTRODES):
            raise ValueError("a row needs one value per electrode")
        if self._origin is None:
            # The amplifier's offset would hit the filters as a step.
            self._origin = block[0].copy()
        coef, state, notch, hum = self._coef, self._state, self._notch, self._notch_state
        for raw in block:
            x = raw - self._origin
            if notch is not None:
                y = notch[0] * x + hum[0]
                hum[0] = notch[1] * x - notch[3] * y + hum[1]
                hum[1] = notch[2] * x - notch[4] * y
                x = y
            self._raw[self._at] = raw
            self._clean[self._at] = x
            self._at = (self._at + 1) % self._size
            signal = x
            for section in range(coef.shape[1]):
                c, memory = coef[:, section], state[:, section]
                out = c[:, 0] * signal + memory[:, 0]
                memory[:, 0] = c[:, 1] * signal - c[:, 3] * out + memory[:, 1]
                memory[:, 1] = c[:, 2] * signal - c[:, 4] * out
                signal = out
            self.power += self._gain * (signal * signal - self.power)
        self.seen += len(block)

    def log_power(self):
        """log10 band power, shape [band, electrode]."""
        return np.log10(np.maximum(self.power, 1e-12))

    def electrodes(self, wild=200.0, flat=1.0):
        """(state, spread) per electrode over the last second. The spread is
        a robust one (scaled median deviation, mains removed), so a blink
        does not condemn a forehead electrode; `wild` is its upper limit
        and `flat` the smallest peak-to-peak swing a live electrode shows."""
        filled = min(self.seen, self._size)
        count = len(ELECTRODES)
        if filled < self._size // 4:
            return np.full(count, FLAT), np.zeros(count)
        raw, clean = self._raw[:filled], self._clean[:filled]
        spread = 1.4826 * np.median(np.abs(clean - np.median(clean, axis=0)), axis=0)
        low, high = self.rails
        margin = 0.01 * (high - low)
        at_rail = ((raw <= low + margin) | (raw >= high - margin)).mean(axis=0)
        state = np.full(count, OK)
        state[spread > wild] = WILD
        state[np.ptp(raw, axis=0) < flat] = FLAT
        state[at_rail > 0.1] = RAILED
        return state, spread
