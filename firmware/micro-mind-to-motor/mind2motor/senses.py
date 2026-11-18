"""What is known about the wearer right now.

The headband sources (Bluetooth, OSC, the made-up wearer) all deliver into
one Senses object: blocks of raw EEG and gravity vectors. The session asks
it once per tick for a Look: band powers and the state of each electrode.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

from .dsp import ELECTRODES, OK, STATE_NAMES, BandMeter
from .tilt import Gravity

GAP_S = 0.5      # a pause this long in the stream restarts the filters


class Look(NamedTuple):
    log_power: np.ndarray    # [band, electrode]
    usable: np.ndarray       # [electrode] bool
    state: np.ndarray        # [electrode] one of dsp.OK, FLAT, RAILED, WILD
    spread: np.ndarray       # [electrode] robust spread of the last second

    def contact(self):
        return "  ".join(f"{name} {STATE_NAMES[code]}"
                         for name, code in zip(ELECTRODES, self.state))


class Senses:
    def __init__(self, mains=60.0, wild=200.0, rails=(0.0, 1682.0), recorder=None):
        self.mains, self.wild, self.rails = mains, wild, rails
        self.recorder = recorder      # called with (now, block) when set
        self.meter = None
        self.gravity = Gravity()
        self.first_eeg = None
        self.last_eeg = None
        self.trouble = ""             # set by a source that cannot deliver

    def on_eeg(self, now, block, rate):
        """Rows of raw EEG that arrived at `now`, sampled at `rate` Hz."""
        if self.meter is None or self.meter.rate != rate:
            self.meter = BandMeter(rate, mains=self.mains, rails=self.rails)
        elif self.last_eeg is not None and now - self.last_eeg > GAP_S:
            self.meter.reset()
        self.meter.push(block)
        self.first_eeg = now if self.first_eeg is None else self.first_eeg
        self.last_eeg = now
        if self.recorder is not None:
            self.recorder(now, block)

    def on_gravity(self, now, xyz):
        self.gravity.update(now, xyz)

    def look(self, now):
        """Band powers and electrode states, or None while there is no
        fresh, settled signal."""
        if (self.meter is None or self.last_eeg is None or
                now - self.last_eeg > GAP_S or not self.meter.ready):
            return None
        state, spread = self.meter.electrodes(wild=self.wild)
        return Look(self.meter.log_power(), state == OK, state, spread)
