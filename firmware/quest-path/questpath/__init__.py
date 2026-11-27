"""Trace a path with your eyes in a Meta Quest; the Micromouse drives it.

The headset shows a lane of rows, three squares to a row. The squares of the
row ahead of the mouse flicker one at a time; the wearer looks at the square
the mouse should go to. The laptop knows which square was flickering when,
looks for the flicker rate in the EEG that MuseLog streams from a Muse
headband (a steady-state visual evoked potential), and sends the mouse there.

    python -m questpath demo        everything simulated, no hardware
    python -m questpath run ...     headset page + MuseLog + the mouse
"""

__version__ = "0.1.0"
