"""Whole sessions with a made-up wearer, on a made-up clock."""

import random

import pytest

from mind2motor import score
from mind2motor.bench import rehearse
from mind2motor.board import STOP, ModelLink, Wheels, Wire
from mind2motor.session import Config, Stage, make_plan
from mind2motor.sources import SyntheticWearer
from helpers import MusecarBoard, Rig, quick


def moving(row):
    return (row.get("left") or 0) + (row.get("right") or 0) > 0


def test_plan_alternates_and_never_repeats_a_steering_cue():
    for seed in range(30):
        plan = make_plan(random.Random(seed), 5, 4)
        kinds = [cue.kind for cue in plan]
        drive, steer = kinds[:10], kinds[10:]
        assert sorted(drive) == ["focus"] * 5 + ["relax"] * 5
        assert all(a != b for a, b in zip(drive, drive[1:]))
        assert sorted(steer) == ["centre"] * 4 + ["left"] * 4 + ["right"] * 4
        assert all(a != b for a, b in zip(steer, steer[1:]))
        assert all(9 <= cue.seconds <= 17 for cue in plan[:10])
    firsts = {make_plan(random.Random(seed), 2)[0].kind for seed in range(30)}
    assert firsts == {"focus", "relax"}


def test_a_wearer_who_follows_the_cues_drives_the_car():
    session, log, console = rehearse(quick(steer_trials=2), SyntheticWearer(seed=2), seed=2)
    assert session.ended == "test"
    result = score.analyse(log.rows)
    assert result["cues_scored"] == 14 and result["cues_interrupted"] == 0
    assert result["drive"]["steady_accuracy"] > 0.95
    assert result["drive"]["reached"] == 8
    assert result["drive"]["chance"]["p_value"] < 0.05
    assert result["steer"]["steady_accuracy"] > 0.9
    assert result["link"]["with_board_status"] == 1.0
    assert session.calibration.held_out_accuracy > 0.9
    assert session.tilt is not None


def test_a_wearer_who_does_the_opposite_scores_near_zero():
    session, log, _ = rehearse(quick(steer=False), SyntheticWearer(seed=2, behaviour="inverts"), seed=2)
    result = score.analyse(log.rows)
    assert result["drive"]["steady_accuracy"] < 0.15
    assert result["drive"]["reached"] == 0
    assert result["drive"]["chance"]["p_value"] > 0.9
    assert result["drive"]["bits_per_cue"] == 0.0


def test_a_wearer_who_ignores_the_cues_scores_at_chance():
    accuracies, small = [], 0
    for seed in range(4):
        _, log, _ = rehearse(quick(steer=False, trials=6),
                             SyntheticWearer(seed=seed, behaviour="ignores"), seed=seed)
        drive = score.analyse(log.rows)["drive"]
        accuracies.append(drive["steady_accuracy"])
        small += drive["chance"]["p_value"] < 0.05
    assert 0.3 < sum(accuracies) / len(accuracies) < 0.7
    assert small <= 1


def test_states_that_do_not_differ_are_flagged_at_calibration():
    rig = Rig(quick(steer=False), seed=1, contrast=0.0)
    assert rig.run(until=Stage.DRIVING)
    assert rig.session.calibration.held_out_accuracy < 0.7
    assert rig.said("overlap a lot")


def test_nothing_moves_before_the_operator_says_so():
    rig = Rig(quick(armed=False, steer=False, trials=0), seed=3)
    rig.run(seconds=6)
    assert rig.session.stage is Stage.READY and rig.said("Press space to begin")
    rig.run(seconds=20)
    assert rig.session.stage is Stage.READY          # still waiting
    rig.tick(" ")
    assert rig.run(until=Stage.PAUSED, limit=60)
    rig.wearer.instruct("focus", rig.now)
    rig.run(seconds=8)
    assert rig.session.latch.on                      # the decoder says go
    assert not any(moving(row) for row in rig.rows)  # but the car is held
    rig.tick(" ")
    rig.run(seconds=2)
    assert rig.session.stage is Stage.DRIVING and moving(rig.rows[-1])
    assert rig.car.model.duty == (24000, 24000)
    rig.tick(" ")
    rig.run(seconds=0.5)
    assert rig.session.stage is Stage.PAUSED and rig.car.model.duty == (0, 0)
    rig.tick("q")
    assert rig.session.stage is Stage.DONE and rig.session.ended == "quit"


def test_no_headband_means_only_stop_frames():
    rig = Rig(quick(), seed=3)
    rig.signal = False
    rig.run(seconds=30)
    assert rig.session.stage is Stage.SEARCH
    assert rig.said("Nothing yet")
    assert rig.car.model.duty == (0, 0)
    assert all(row["left"] == 0 and row["right"] == 0 for row in rig.rows)


def test_losing_one_electrode_while_driving_changes_nothing():
    rig = Rig(quick(steer=False, trials=5), seed=4)
    assert rig.run(until=Stage.DRIVING)
    rig.wearer.dropouts.append((1, rig.now + 10.0, rig.now + 45.0))    # AF7 off the skin
    rig.run()
    result = score.analyse(rig.rows)
    assert result["drive"]["steady_accuracy"] > 0.95
    assert any(row.get("contact") == "0200" for row in rig.rows)       # it was noticed


def test_losing_every_electrode_stops_the_car():
    rig = Rig(quick(steer=False, trials=0), seed=5)
    assert rig.run(until=Stage.DRIVING)
    rig.wearer.instruct("focus", rig.now)
    rig.run(seconds=8)
    assert rig.car.model.duty == (24000, 24000)
    for electrode in range(4):
        rig.wearer.dropouts.append((electrode, rig.now, rig.now + 10.0))
    rig.run(seconds=2.5)
    assert rig.car.model.duty == (0, 0)
    assert rig.rows[-1]["go"] == 0
    rig.run(seconds=15)                              # contact returns, the wearer still focused
    assert rig.car.model.duty == (24000, 24000)


def test_a_dead_stream_stops_the_car_within_a_second():
    rig = Rig(quick(steer=False, trials=0), seed=5)
    assert rig.run(until=Stage.DRIVING)
    rig.wearer.instruct("focus", rig.now)
    rig.run(seconds=8)
    assert rig.car.model.duty == (24000, 24000)
    rig.signal = False                               # the headband falls silent
    rig.run(seconds=1.2)
    assert rig.car.model.duty == (0, 0)


def test_a_pause_voids_the_cue_and_gives_it_again():
    rig = Rig(quick(armed=False, steer=False, trials=2), seed=6)
    rig.run(until=Stage.READY)
    rig.tick(" ")
    assert rig.run(until=Stage.PAUSED, limit=60)
    rig.tick(" ")
    rig.run(seconds=3)
    interrupted = rig.session.cue
    rig.tick(" ")                                    # pause in the middle of the first cue
    assert rig.session.stage is Stage.PAUSED and rig.session.cue is None
    rig.run(seconds=2)
    rig.tick(" ")
    rig.run(seconds=1)
    assert rig.session.cue == interrupted and rig.session.trial == 2
    rig.run()
    assert rig.session.ended == "test"
    result = score.analyse(rig.rows)
    assert result["cues_interrupted"] == 1 and result["cues_scored"] == 4


class Unpluggable(ModelLink):
    def __init__(self):
        super().__init__()
        self.out = False

    def send(self, wheels, now):
        if self.out:
            self.sent = STOP
            return False
        return super().send(wheels, now)


def test_an_unplugged_car_does_not_set_off_by_itself():
    car = Unpluggable()
    rig = Rig(quick(armed=False, steer=False, trials=0), seed=7, car=car)
    rig.run(until=Stage.READY)
    rig.tick(" ")
    assert rig.run(until=Stage.PAUSED, limit=60)
    rig.tick(" ")
    rig.wearer.instruct("focus", rig.now)
    rig.run(seconds=8)
    assert car.model.duty == (24000, 24000)
    car.out = True
    rig.run(seconds=0.3)
    assert rig.session.stage is Stage.PAUSED and rig.said("unplugged")
    car.out = False
    rig.run(seconds=5)
    assert rig.session.latch.on and car.model.duty == (0, 0)    # focused, plugged in, still held
    rig.tick(" ")
    rig.run(seconds=1)
    assert car.model.duty == (24000, 24000)


def test_recalibrating_starts_the_recordings_again():
    rig = Rig(quick(armed=False, steer=False, trials=0), seed=8)
    rig.run(until=Stage.READY)
    rig.tick(" ")
    assert rig.run(until=Stage.PAUSED, limit=60)
    first = rig.session.calibration
    rig.tick("r")
    assert rig.session.stage is Stage.RELAX
    assert rig.run(until=Stage.PAUSED, limit=60)
    assert rig.session.calibration is not first


def test_steering_slows_the_inner_wheel():
    rig = Rig(quick(trials=0), seed=9)
    assert rig.run(until=Stage.DRIVING)
    rig.wearer.instruct("focus", rig.now)
    rig.run(seconds=8)
    assert rig.car.model.duty == (24000, 24000)
    rig.wearer.instruct("left", rig.now)
    rig.run(seconds=3)
    left, right = rig.car.model.duty
    assert right == 24000 and left < 12000           # bearing left
    rig.wearer.instruct("right", rig.now)
    rig.run(seconds=3)
    left, right = rig.car.model.duty
    assert left == 24000 and right < 12000
    rig.wearer.instruct("centre", rig.now)
    rig.run(seconds=3)
    assert rig.car.model.duty == (24000, 24000)


def test_no_accelerometer_means_no_steering_but_still_driving():
    class NoMotion(SyntheticWearer):
        def advance(self, now, senses):
            keep = senses.on_gravity
            senses.on_gravity = lambda *_: None
            super().advance(now, senses)
            senses.on_gravity = keep

    session, log, console = rehearse(quick(steer_trials=2), NoMotion(seed=2), seed=2)
    assert session.ended == "test" and session.tilt is None
    assert any("no steering" in note for note in console.notes)
    result = score.analyse(log.rows)
    assert "steer" not in result and result["drive"]["steady_accuracy"] > 0.95


class MusecarLink(Wire):
    """The musecar firmware's serial behaviour in place of the board."""

    def __init__(self):
        super().__init__()
        self.model = MusecarBoard()

    def send(self, wheels, now):
        due = int(round(now * 1000.0)) - self.model.ms
        if due > 0:
            self.model.run(due)
        self.model.feed(self.encode(wheels))
        self.decode(self.model.take_output(), now)
        return True

    def close(self):
        pass


def test_the_session_also_drives_the_musecar_firmware():
    car = MusecarLink()
    rig = Rig(quick(trials=3, steer_trials=1), seed=10, car=car)
    rig.run()
    assert rig.session.ended == "test" and car.dialect == "digits"
    result = score.analyse(rig.rows)
    assert result["drive"]["steady_accuracy"] > 0.9
    assert set(car.model.history) >= {0, 1, 3, 4}    # stopped, forward, both turns
    assert {(row["left"], row["right"]) for row in rig.rows} <= {
        (0, 0), (75, 75), (25, 75), (75, 25)}        # only what that firmware can do


def test_run_folder_holds_the_log_the_calibration_and_the_raw_signal(tmp_path):
    import json
    import numpy as np
    from mind2motor.runlog import RunLog
    from mind2motor.senses import Senses
    from mind2motor.console import Silent
    from mind2motor.session import Session

    wearer = SyntheticWearer(seed=11)
    log = RunLog(tmp_path / "run", record=True, keep=True)
    senses = Senses(recorder=log.raw)
    session = Session(senses, ModelLink(), Silent(wearer.instruct), log,
                      quick(trials=2, steer=False), random.Random(11))
    now = 0.0
    while session.stage is not Stage.DONE:
        now += 0.1
        wearer.advance(now, senses)
        session.tick(now)
    log.close()
    result = score.report(tmp_path / "run")
    assert result["drive"]["steady_accuracy"] > 0.9
    assert json.loads((tmp_path / "run" / "report.json").read_text()) == result
    meta = json.loads((tmp_path / "run" / "meta.json").read_text())
    assert len(meta["calibration"]["weights"]) == 4
    raw = np.fromfile(tmp_path / "run" / "eeg.f32", dtype="<f4").reshape(-1, 5)
    assert len(raw) == pytest.approx(now * 256, abs=30)
    assert 700 < raw[:, 1:].mean() < 950
    assert score.analyse(log.rows)["drive"]["steady_accuracy"] == result["drive"]["steady_accuracy"]
