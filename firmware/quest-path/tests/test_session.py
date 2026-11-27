"""Whole sessions in simulated time: wearer, headset and mouse all made up."""

import math

import pytest

from questpath.grid import Lane
from questpath.rehearsal import Rehearsal, parse_path
from questpath.session import SessionConfig


def run(path, rows=None, seed=1, **options):
    gaze = parse_path(path)
    config = SessionConfig(lane=Lane(rows=rows or max(gaze) + 1), **options.pop("config", {}))
    rehearsal = Rehearsal(gaze, config, seed=seed, **options)
    return rehearsal, rehearsal.run()


def cells(summary):
    return [tuple(c) for c in summary["path"]]


def test_parse_path():
    assert parse_path("M, l ,R,-,2") == {1: 1, 2: 0, 3: 2, 4: None, 5: 2}
    with pytest.raises(ValueError):
        parse_path("M,X")


def test_the_mouse_follows_the_traced_path():
    rehearsal, summary = run("M,L,L,R,M")
    assert summary["outcome"] == "finished"
    assert cells(summary) == [(0, 1), (1, 1), (2, 0), (3, 0), (4, 2), (5, 1)]
    assert summary["mouse"] == [5, 1]
    assert math.dist(summary["mouse_mm"], summary["target_mm"]) < 10       # mm
    assert abs(summary["mouse_heading"]) <= 3
    assert summary["calibration"]["usable"]
    # the wearer's first square is to the left: the first turn is a left turn
    turns = [c for c in rehearsal.board_model.sent if c.startswith("TURN")]
    assert turns[0].split()[1] == "90"


def test_straight_ahead_needs_no_turns():
    rehearsal, summary = run("M,M,M")
    assert summary["outcome"] == "finished"
    assert not [c for c in rehearsal.board_model.sent if c.startswith("TURN")]
    assert math.isclose(rehearsal.board_model.x, 540, abs_tol=6)


def test_looking_away_keeps_the_mouse_still():
    rehearsal, summary = run("M,-", config={"max_idle": 2})
    assert summary["outcome"].startswith("gave up")
    assert cells(summary) == [(0, 1), (1, 1)]            # the first row was traced, the second never
    assert summary["mouse"] == [1, 1]
    assert math.isclose(rehearsal.board_model.x, 180, abs_tol=6)
    assert [d["pick"] for d in summary["decisions"]] == [1, None, None]
    assert rehearsal.board_model.mode == "idle"


def test_no_response_at_all_never_moves_the_mouse():
    rehearsal, summary = run("M,L", config={"max_idle": 2, "calib_blocks": 0},
                             wearer={"look_uv": 0.0})
    assert summary["outcome"].startswith("gave up")
    assert summary["mouse"] == [0, 1]
    assert not [c for c in rehearsal.board_model.sent if c.startswith(("V ", "TURN"))]


def test_trace_first_then_drive():
    rehearsal, summary = run("L,R,M", config={"drive": "end"})
    assert summary["outcome"] == "finished"
    assert cells(summary) == [(0, 1), (1, 0), (2, 2), (3, 1)]
    assert summary["mouse"] == [3, 1]
    assert math.dist(summary["mouse_mm"], summary["target_mm"]) < 10
    # nothing moved until the last row was chosen
    states = rehearsal.display.states
    first_move = next(i for i, s in enumerate(states) if s["phase"] == "moving")
    assert all(s["mouse"] == [0, 1] for s in states[:first_move])
    assert len(states[first_move]["path"]) == 4


def test_diagonal_style():
    rehearsal, summary = run("L,R", config={"style": "diagonal"})
    assert summary["outcome"] == "finished"
    assert math.dist(summary["mouse_mm"], summary["target_mm"]) < 12
    turns = [int(c.split()[1]) for c in rehearsal.board_model.sent if c.startswith("TURN")]
    assert turns[0] == 45                                # one square left, one forward


def test_a_mouse_that_does_not_answer_ends_the_session_cleanly():
    rehearsal, summary = run("M,M", mouse={"silent": True})
    assert summary["outcome"].startswith("error")
    assert "HELLO" in summary["outcome"]
    assert rehearsal.board_model.sent[-1] == "STOP"


def test_stop_from_the_headset_stops_the_mouse():
    gaze = parse_path("M,M,M,M")
    rehearsal = Rehearsal(gaze, SessionConfig(lane=Lane(rows=5)), seed=2)
    # press Stop about 40 s in
    pressed = []

    def press(now):
        if not pressed and now > rehearsal.clock.t - 1 and now - 100.0 > 40.0:
            pressed.append(now)
            rehearsal.display.buttons.append("stop")
    rehearsal.clock.listeners.append(press)
    summary = rehearsal.run()
    assert summary["outcome"] == "stopped from the headset"
    assert rehearsal.board_model.sent[-1] == "STOP"
    assert summary["mouse"][0] < 4


def test_the_headset_is_told_what_to_show():
    rehearsal, summary = run("M,L")
    phases = [s["phase"] for s in rehearsal.display.states]
    assert phases[0] == "wait" and "calib" in phases and "trace" in phases and "moving" in phases
    assert phases[-1] == "done"
    last = rehearsal.display.states[-1]
    assert last["mouse"] == [2, 0] and last["rows"] == 3 and last["cols"] == 3
    tracing = [s for s in rehearsal.display.states if s["phase"] == "trace"]
    assert [s["active_row"] for s in tracing][:2] == [1, 2]


def test_every_flash_is_remembered_with_its_square():
    rehearsal, summary = run("M,R", config={"calib_blocks": 0})
    flashes = [r for r in rehearsal.session.records if r["kind"] == "flash"]
    first_round = flashes[:3]
    assert sorted(f["col"] for f in first_round) == [0, 1, 2]       # each square once, in shuffled order
    assert all(f["row"] == 1 and f["hz"] == 12.0 and f["ok"] for f in first_round)
    best = max(first_round, key=lambda f: f["z"])
    assert best["col"] == 1                                         # the square the wearer looked at
    decisions = [r for r in rehearsal.session.records if r["kind"] == "decision"]
    assert [d["pick"] for d in decisions] == [1, 2]


def test_other_headsets_and_streams():
    for options in ({"refresh": 90.0}, {"refresh": 120.0}, {"wearer": {"burst": 0.5}}):
        rehearsal, summary = run("L,M", seed=4, **options)
        assert summary["outcome"] == "finished", options
        assert cells(summary) == [(0, 1), (1, 0), (2, 1)], options


def test_a_weak_signal_gives_a_warning_not_a_guess():
    rehearsal, summary = run("M,M", config={"max_idle": 1}, wearer={"look_uv": 0.3})
    assert not summary["calibration"]["usable"]
    assert any("WARNING" in line for line in rehearsal.lines)
    assert summary["mouse"] == [0, 1]


def test_stop_interrupts_a_move_in_progress():
    gaze = parse_path("L,M")
    rehearsal = Rehearsal(gaze, SessionConfig(lane=Lane(rows=3), calib_blocks=0), seed=3)
    pressed = []

    def press(now):
        if not pressed and rehearsal.board_model.mode == "V":       # the mouse is driving
            pressed.append(now)
            rehearsal.display.buttons.append("stop")
    rehearsal.clock.listeners.append(press)
    summary = rehearsal.run()
    assert summary["outcome"] == "stopped from the headset"
    sent = rehearsal.board_model.sent
    assert sent[-1] == "STOP" and rehearsal.board_model.mode == "idle"
    # it stopped part-way: the mouse never reached the square
    assert math.dist((rehearsal.board_model.x, rehearsal.board_model.y), (180.0, 180.0)) > 30
