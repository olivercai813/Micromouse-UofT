import math

import pytest

from questpath.grid import Forward, Lane, Turn, board, plan_step, run_moves, wrap180


def test_lane_geometry():
    lane = Lane(rows=5, cell_mm=180, start_col=1)
    assert lane.start == (0, 1)
    assert lane.centre_mm((0, 1)) == (0, 0)
    assert lane.centre_mm((2, 0)) == (360, 180)       # two rows on, one square to the left
    assert lane.centre_mm((1, 2)) == (180, -180)


@pytest.mark.parametrize("rows, cols, start", [(1, 3, 1), (4, 1, 0), (4, 3, 3)])
def test_lane_refuses_nonsense(rows, cols, start):
    with pytest.raises(ValueError):
        Lane(rows=rows, cols=cols, start_col=start)


@pytest.mark.parametrize("style", ["square", "diagonal"])
@pytest.mark.parametrize("col, to_col", [(1, 1), (1, 0), (1, 2), (0, 2), (2, 0), (0, 0)])
def test_every_step_lands_on_its_square_facing_up_the_lane(style, col, to_col):
    lane = Lane(rows=4, cell_mm=180)
    here, there = (1, col), (2, to_col)
    x0, y0 = lane.centre_mm(here)
    x, y, heading = run_moves(plan_step(lane, here, there, style), x0, y0, 0.0)
    want = lane.centre_mm(there)
    assert math.isclose(x, want[0], abs_tol=1e-6)
    assert math.isclose(y, want[1], abs_tol=1e-6)
    assert heading == 0.0


def test_square_style_uses_only_quarter_turns():
    lane = Lane()
    for to_col in (0, 1, 2):
        for move in plan_step(lane, (0, 1), (1, to_col), "square"):
            if isinstance(move, Turn):
                assert move.heading in (0.0, 90.0, -90.0)


def test_left_is_a_left_turn_and_right_a_right_turn():
    lane = Lane()
    assert plan_step(lane, (0, 1), (1, 0))[0] == Turn(90.0)       # + = left, as in the firmware
    assert plan_step(lane, (0, 1), (1, 2))[0] == Turn(-90.0)
    assert plan_step(lane, (0, 1), (1, 1)) == [Turn(0.0), Forward(180.0)]


def test_a_step_goes_one_row_on_and_stays_in_the_lane():
    lane = Lane()
    with pytest.raises(ValueError):
        plan_step(lane, (0, 1), (2, 1))
    with pytest.raises(ValueError):
        plan_step(lane, (0, 1), (1, 3))
    with pytest.raises(ValueError):
        plan_step(lane, (0, 1), (1, 0), "zigzag")


def test_wrap180():
    assert wrap180(190) == -170
    assert wrap180(-190) == 170
    assert wrap180(360) == 0
    assert wrap180(180) == 180
    assert wrap180(-180) == 180


def test_board_text():
    lane = Lane(rows=3)
    text = board(lane, (1, 0), [(0, 1), (1, 0)], active_row=2)
    lines = text.splitlines()
    assert lines[0].strip().startswith(". . .") and "look here" in lines[0]
    assert lines[1].strip() == "M . ."
    assert lines[2].strip() == ". o ."
