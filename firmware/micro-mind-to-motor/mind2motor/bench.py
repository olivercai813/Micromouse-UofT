"""A whole session without a headband, a car or a clock.

A made-up wearer feeds raw EEG through the real filters, decoder and
session, the session drives a model of the board, and time is a number
that goes up by a tick. A twenty-cue test, five minutes on the real car,
takes a few seconds here, so the controls can be repeated over many seeds:

  follows   the wearer does what the cues say: the car should match them
  inverts   the wearer does the opposite: the car should match almost never
  ignores   the wearer pays no attention: the score should sit at chance
            and the p-value should not be small

The bench checks the software's logic and statistics. It says nothing about
how separable a real person's relaxed and focused EEG is.
"""

from __future__ import annotations

import random

from .board import ModelLink
from .console import Silent
from .runlog import RunLog
from .senses import Senses
from .session import Config, Session, Stage
from .sources import SyntheticWearer

TICK_S = 0.1


def rehearse(config=None, wearer=None, seed=0, log=None, car=None, limit_s=7200.0):
    """Run a session to its end in made-up time. Returns (session, log,
    console); the log keeps its rows in memory."""
    config = config or Config(armed=True, trials=3)
    wearer = wearer or SyntheticWearer(seed=seed)
    log = log or RunLog(keep=True)
    console = Silent(wearer.instruct)
    senses = Senses()
    session = Session(senses, car or ModelLink(), console, log, config, random.Random(seed))
    now = 0.0
    while session.stage is not Stage.DONE and now < limit_s:
        now += TICK_S
        wearer.advance(now, senses)
        session.tick(now)
    return session, log, console
