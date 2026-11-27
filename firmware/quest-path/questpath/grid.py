"""The lane of squares and how the mouse gets from one square to the next.

The lane is `rows` rows of `cols` squares (three by default), each `cell_mm`
on a side. Row 0 is the row the mouse starts in; rows count away from the
start. Column 0 is on the left as seen from the start, looking up the lane.

Headings are in degrees, 0 = up the lane, positive = to the left, which is the
firmware's convention for TURN and POSE.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Cell = tuple  # (row, col)


@dataclass(frozen=True)
class Lane:
    rows: int = 8
    cols: int = 3
    cell_mm: float = 180.0
    start_col: int = 1

    def __post_init__(self):
        if self.rows < 2:
            raise ValueError("the lane needs at least two rows")
        if self.cols < 2:
            raise ValueError("a row needs at least two squares")
        if not 0 <= self.start_col < self.cols:
            raise ValueError("the start square is outside the row")
        if self.cell_mm <= 0:
            raise ValueError("the square size must be positive")

    @property
    def start(self) -> Cell:
        return (0, self.start_col)

    def centre_mm(self, cell: Cell):
        """Centre of a square, in millimetres from the centre of the start
        square: (forward, left)."""
        row, col = cell
        return row * self.cell_mm, (self.start_col - col) * self.cell_mm


@dataclass(frozen=True)
class Turn:
    """Turn on the spot until the mouse faces this heading."""
    heading: float


@dataclass(frozen=True)
class Forward:
    """Drive straight ahead this many millimetres."""
    mm: float


def plan_step(lane: Lane, here: Cell, there: Cell, style: str = "square"):
    """Moves that take the mouse from the centre of `here`, facing up the
    lane, to the centre of `there` in the next row, facing up the lane again.

    style "square":   only straight runs and quarter turns: sideways along its
                      own row first, then forward into the next row.
    style "diagonal": turn towards the square, drive straight to it, turn back.
    """
    row, col = here
    to_row, to_col = there
    if to_row != row + 1:
        raise ValueError("a step goes to the next row")
    if not 0 <= to_col < lane.cols:
        raise ValueError("that square is outside the row")
    sideways = col - to_col            # squares to the left (negative = right)
    if sideways == 0:
        return [Turn(0.0), Forward(lane.cell_mm)]
    if style == "diagonal":
        heading = math.degrees(math.atan2(sideways, 1.0))
        return [Turn(heading), Forward(lane.cell_mm * math.hypot(1.0, sideways)), Turn(0.0)]
    if style != "square":
        raise ValueError("style is 'square' or 'diagonal'")
    return [Turn(90.0 if sideways > 0 else -90.0), Forward(lane.cell_mm * abs(sideways)),
            Turn(0.0), Forward(lane.cell_mm)]


def run_moves(moves, x=0.0, y=0.0, heading=0.0):
    """Where a perfect mouse ends up: (forward mm, left mm, heading)."""
    for move in moves:
        if isinstance(move, Turn):
            heading = move.heading
        else:
            x += move.mm * math.cos(math.radians(heading))
            y += move.mm * math.sin(math.radians(heading))
    return x, y, heading


def wrap180(angle: float) -> float:
    """The same angle in (-180, 180]."""
    angle = math.fmod(angle, 360.0)
    if angle > 180.0:
        angle -= 360.0
    if angle <= -180.0:
        angle += 360.0
    return angle


def board(lane: Lane, mouse: Cell, path, active_row=None) -> str:
    """The lane as text, far end at the top: M = mouse, o = path, . = square."""
    lines = []
    on_path = set(path)
    for row in range(lane.rows - 1, -1, -1):
        cells = []
        for col in range(lane.cols):
            cell = (row, col)
            cells.append("M" if cell == mouse else "o" if cell in on_path else ".")
        mark = "  <- look here" if row == active_row else ""
        lines.append("  " + " ".join(cells) + mark)
    return "\n".join(lines)
