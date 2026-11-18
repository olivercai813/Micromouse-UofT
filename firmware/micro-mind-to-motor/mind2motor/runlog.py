"""What a run leaves behind, in one folder:

  ticks.csv   one row per tick: what was asked of the wearer, what the
              decoder made of the signal, what was sent to the car and what
              the board said it was doing
  meta.json   the settings and the calibration
  eeg.f32     optional: the raw EEG as little-endian float32 rows of
              (time, TP9, AF7, AF8, TP10), so a run can be decoded again
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import time

import numpy as np

from .dsp import ELECTRODES

COLUMNS = ["t", "stage", "cue", "trial", "raw_level", "level", "go", "armed",
           "lean", "turn", "left", "right", "ack_left", "ack_right", "board",
           "contact", *(f"spread_{name}" for name in ELECTRODES)]


class RunLog:
    def __init__(self, folder=None, record=False, keep=False):
        self.folder = Path(folder) if folder else None
        self.rows = [] if keep else None     # the ticks, kept in memory on request
        self._rows = self._raw = self._file = None
        if self.folder:
            self.folder.mkdir(parents=True, exist_ok=True)
            self._file = open(self.folder / "ticks.csv", "w", newline="", buffering=1)
            self._rows = csv.DictWriter(self._file, COLUMNS, restval="")
            self._rows.writeheader()
            if record:
                self._raw = open(self.folder / "eeg.f32", "wb")

    @staticmethod
    def new_folder(root="runs"):
        return Path(root) / time.strftime("%Y%m%d-%H%M%S")

    def tick(self, **fields):
        if self.rows is not None:
            self.rows.append(fields)
        if self._rows:
            self._rows.writerow(fields)

    def raw(self, now, block):
        if self._raw:
            block = np.atleast_2d(block)
            rows = np.column_stack([np.full(len(block), now), block])
            self._raw.write(rows.astype("<f4").tobytes())

    def meta(self, **fields):
        if self.folder:
            path = self.folder / "meta.json"
            saved = json.loads(path.read_text()) if path.exists() else {}
            saved.update(fields)
            path.write_text(json.dumps(saved, indent=2))

    def close(self):
        for handle in (self._file, self._raw):
            if handle:
                handle.close()
        self._rows = self._raw = self._file = None
