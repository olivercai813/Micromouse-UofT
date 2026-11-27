import math

import pytest

from questpath.clock import SimClock
from questpath.grid import Forward, Lane, Turn, plan_step
from questpath.mouse import Mouse, MouseConfig, MouseError, SimMouse


def rig(config=None, **board):
    clock = SimClock()
    model = SimMouse(clock, **board)
    return clock, model, Mouse(model, clock, config)


def test_connect_speaks_the_firmware_protocol():
    clock, model, mouse = rig()
    mouse.connect()
    mouse.zero()
    assert model.sent[:3] == ["HELLO", "TELEM 2000", "ZERO_ENC"]


def test_forward_uses_a_timed_closed_loop_run():
    clock, model, mouse = rig()
    mouse.connect()
    mouse.zero()
    covered = mouse.forward(180)
    runs = [c for c in model.sent if c.startswith("V ")]
    assert runs == ["V 150 0 1200"]                 # 180 mm at 150 mm/s
    assert math.isclose(covered, 180, abs_tol=1.0)
    assert math.isclose(model.x, 180, abs_tol=1.0) and abs(model.y) < 1e-6


def test_long_runs_are_split_under_the_firmware_limit():
    clock, model, mouse = rig(MouseConfig(speed_mms=100, trim=False))
    mouse.connect()
    mouse.forward(1500)                             # 15 s at 100 mm/s: more than one command
    runs = [c for c in model.sent if c.startswith("V ")]
    assert len(runs) == 2 and all(int(c.split()[3]) <= 9000 for c in runs)
    assert math.isclose(model.x, 1500, abs_tol=2.0)


def test_a_slow_mouse_is_trimmed_from_its_own_odometry():
    clock, model, mouse = rig(speed_gain=0.85)
    mouse.connect()
    covered = mouse.forward(180)
    assert len([c for c in model.sent if c.startswith("V ")]) == 2       # the run and one correction
    assert abs(covered - 180) < 6
    untrimmed_clock, untrimmed_model, untrimmed = rig(MouseConfig(trim=False), speed_gain=0.85)
    untrimmed.connect()
    untrimmed.forward(180)
    assert abs(untrimmed_model.x - 153) < 2


def test_turns_go_to_an_absolute_heading_and_correct_their_own_error():
    clock, model, mouse = rig(turn_overshoot=0.07)   # every turn goes 7 % too far
    mouse.connect()
    mouse.zero()
    left = mouse.turn_to(90)
    assert abs(left) <= 3.0 and abs(model.heading - 90) <= 3.0
    assert [c.split()[1] for c in model.sent if c.startswith("TURN")][:2] == ["90", "-6"]
    mouse.turn_to(0)
    assert abs(model.heading) <= 3.0


def test_no_turn_is_sent_when_already_facing_the_right_way():
    clock, model, mouse = rig()
    mouse.connect()
    mouse.turn_to(0)
    assert not [c for c in model.sent if c.startswith("TURN")]


@pytest.mark.parametrize("style", ["square", "diagonal"])
def test_a_planned_step_ends_on_the_square(style):
    lane = Lane(cell_mm=180)
    clock, model, mouse = rig()
    mouse.connect()
    mouse.zero()
    mouse.execute(plan_step(lane, (0, 1), (1, 0), style))
    assert math.isclose(model.x, 180, abs_tol=3) and math.isclose(model.y, 180, abs_tol=3)
    assert abs(model.heading) <= 3


def test_telemetry_lines_are_ignored():
    clock, model, mouse = rig(telemetry_s=0.2)
    mouse.connect()
    assert mouse.pose()["imu"] == 1.0
    mouse.forward(90)
    assert math.isclose(model.x, 90, abs_tol=1.0)


def test_a_silent_board_is_an_error_not_a_hang():
    clock, model, mouse = rig(silent=True)
    with pytest.raises(MouseError, match="HELLO"):
        mouse.connect()


def test_wheels_in_the_air_turn_is_reported():
    clock, model, mouse = rig(stuck_turns=True)
    mouse.connect()
    with pytest.raises(MouseError, match="turn"):
        mouse.turn_to(90)


def test_a_refused_command_is_an_error():
    clock, model, mouse = rig()
    mouse.connect()
    mouse._send("FLY 3")
    with pytest.raises(MouseError, match="refused"):
        mouse._wait(lambda s: s.startswith("ACK"), 1.0, "ACK")


def test_stop_never_raises_and_close_stops():
    clock, model, mouse = rig(silent=True)
    mouse.stop()                                     # no answer: still returns
    clock2, model2, mouse2 = rig()
    mouse2.connect()
    mouse2.close()
    assert model2.sent[-1] == "STOP" and model2.closed


def test_the_board_stops_itself_when_the_time_is_up():
    clock, model, mouse = rig()
    model.write("V 200 0 500")
    clock.sleep(2.0)
    lines = model.lines()
    assert "ACK V" in lines and "EVENT MOTOR_TIMEOUT" in lines
    assert math.isclose(model.x, 100, abs_tol=1.0)   # 0.5 s at 200 mm/s, then nothing


def test_unknown_moves_are_refused():
    clock, model, mouse = rig()
    mouse.connect()
    with pytest.raises(MouseError):
        mouse.execute(["dance"])
    mouse.execute([Turn(0), Forward(10)])
