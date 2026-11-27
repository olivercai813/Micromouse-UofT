"""The recent EEG, with a sample clock rebuilt from when samples arrived.

Samples reach the laptop over Bluetooth, a phone and Wi-Fi, in bursts: several
may land in the same millisecond after a pause of half a second. Their arrival
times are therefore a poor clock. But the headband samples at a steady rate,
so sample number k was taken at  k / rate + constant.  The constant is found
from the samples that got through fastest: the smallest (arrival - k / rate)
seen in the last few seconds. Times given out by this buffer are on the
laptop's clock and run a steady transport delay (a tenth of a second or so)
behind the moment the brain produced the signal; the detector leaves room.

A silence longer than `gap_s` starts a new stretch with its own constant,
because samples may have been lost.
"""

from __future__ import annotations

import threading

import numpy as np

KNOWN_RATES = (256.0, 64.0, 128.0, 220.0, 250.0, 500.0)
CHANNELS = ("TP9", "AF7", "AF8", "TP10", "AUX", "AUX2", "AUX3", "AUX4")


class EEGBuffer:
    def __init__(self, keep_s: float = 90.0, gap_s: float = 1.0, window_s: float = 8.0, rate: float = 0.0):
        self.keep_s = keep_s
        self.gap_s = gap_s
        self.window_s = window_s
        self.rate = float(rate)          # 0 = work it out from the stream
        self.rate_locked = rate > 0
        self._lock = threading.Lock()
        self._arrival = []               # arrival time of every kept sample
        self._index = []                 # its number within its stretch
        self._stretch = []               # which stretch it belongs to
        self._values = []
        self._count = 0                  # samples in the present stretch
        self._stretch_id = 0
        self._first_arrival = None       # of the present stretch
        self.width = 0
        self.total = 0

    # ---- filling
    def push(self, arrival: float, values) -> None:
        with self._lock:
            if self.width == 0:
                self.width = len(values)
            row = list(values[:self.width]) + [0.0] * (self.width - len(values))
            if self._arrival and arrival - self._arrival[-1] > self.gap_s:
                self._stretch_id += 1
                self._count = 0
                self._first_arrival = None
            if self._first_arrival is None:
                self._first_arrival = arrival
            self._arrival.append(arrival)
            self._index.append(self._count)
            self._stretch.append(self._stretch_id)
            self._values.append(row)
            self._count += 1
            self.total += 1
            if not self.rate_locked and self._count >= 64 and self._count % 32 == 0:
                self._estimate_rate(arrival)
            if len(self._arrival) > 4096 and self._arrival[0] < arrival - self.keep_s - 5.0:
                cut = int(np.searchsorted(self._arrival, arrival - self.keep_s))
                for seq in (self._arrival, self._index, self._stretch, self._values):
                    del seq[:cut]

    def _estimate_rate(self, arrival: float) -> None:
        """The headband's sample rate, from the fastest arrivals.

        Counting samples per second of arrival is thrown off by bursts. At the
        right rate the fastest arrivals keep the same lead over (k / rate) from
        the start of the stream to its end; at a wrong rate that lead drifts.
        The standard rate with the least drift is taken; if none fits, the
        rate between the fastest arrivals of the first and last thirds is."""
        span = arrival - self._first_arrival
        if span < 3.0:
            return
        m = min(self._count, len(self._arrival))
        a = np.asarray(self._arrival[-m:], float)
        k = np.asarray(self._index[-m:], float)
        third = max(8, m // 3)

        def drift(rate):
            lag = a - k / rate
            return abs(lag[-third:].min() - lag[:third].min())

        best = min(KNOWN_RATES, key=drift)
        if drift(best) < 0.03 + 0.004 * span:
            self.rate = best
        else:
            crude = (m - 1) / (a[-1] - a[0])
            lag = a - k / crude
            i, j = int(np.argmin(lag[:third])), m - third + int(np.argmin(lag[-third:]))
            self.rate = float((k[j] - k[i]) / (a[j] - a[i])) if a[j] > a[i] else crude
        if span >= 10.0:
            self.rate_locked = True

    # ---- reading
    def _times(self, around: float):
        """Rebuilt sample times and the samples, as arrays, for a question
        about the moment `around`."""
        arrival = np.asarray(self._arrival, float)
        if len(arrival) == 0 or self.rate <= 0:
            return np.zeros(0), np.zeros((0, max(self.width, 1)))
        index = np.asarray(self._index, float)
        stretch = np.asarray(self._stretch)
        ideal = index / self.rate
        lag = arrival - ideal
        times = np.empty_like(arrival)
        for s in np.unique(stretch):
            sel = np.flatnonzero(stretch == s)
            a = arrival[sel]
            lg = lag[sel]
            # one constant for the whole stretch as seen from `around`: the
            # fastest arrival within a few seconds of it. One constant keeps
            # the samples of a flash evenly spaced.
            near = (a >= around - self.window_s) & (a <= around + 1.0)
            times[sel] = ideal[sel] + (lg[near].min() if near.any() else lg.min())
        return times, np.asarray(self._values, float)

    def latest(self) -> float:
        """Rebuilt time of the newest sample (-inf with no data)."""
        with self._lock:
            if not self._arrival or self.rate <= 0:
                return float("-inf")
            last = self._stretch[-1]
            newest = self._arrival[-1]
            n = len(self._arrival)
            k = n - 1
            lag = float("inf")
            # only the tail of the present stretch matters
            while k >= 0 and self._stretch[k] == last and self._arrival[k] >= newest - self.window_s:
                lag = min(lag, self._arrival[k] - self._index[k] / self.rate)
                k -= 1
            return self._index[-1] / self.rate + lag

    def window(self, start: float, end: float):
        """Samples taken between two times: (array samples x channels, rate,
        fraction of the span actually covered)."""
        with self._lock:
            times, values = self._times(end)
            rate = self.rate
        if len(times) == 0 or rate <= 0:
            return np.zeros((0, max(self.width, 1))), rate, 0.0
        sel = (times >= start) & (times < end)
        data = values[sel]
        cover = len(data) / max(1.0, (end - start) * rate)
        return data, rate, float(min(cover, 1.0))

    def quality(self, seconds: float = 2.0):
        """(samples per second lately, rms of each channel in microvolts)."""
        with self._lock:
            arrival = np.asarray(self._arrival, float)
            values = np.asarray(self._values, float)
        if len(arrival) < 8:
            return 0.0, []
        sel = arrival >= arrival[-1] - seconds
        recent = values[sel]
        span = max(arrival[-1] - arrival[sel][0], 1e-3)
        rate = (sel.sum() - 1) / span
        centred = recent - recent.mean(axis=0)
        return float(rate), [float(v) for v in np.sqrt((centred ** 2).mean(axis=0))]

    def names(self):
        return list(CHANNELS[:self.width])
