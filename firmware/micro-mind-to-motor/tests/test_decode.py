import numpy as np
import pytest

from mind2motor.decode import Calibration, CalibrationError, Latch, auc, fit

ALL = np.ones(4, bool)
# Each electrode sits at its own level, as real ones do.
OFFSETS = np.array([0.0, 0.6, -0.4, 0.2])


def recording(rng, alpha, beta, ticks=200, noise=0.08, usable=ALL):
    """Ticks of log power [theta, alpha, beta, gamma] x 4 electrodes."""
    base = np.array([1.0, alpha, beta, 0.2])[:, None] + OFFSETS[None, :]
    return [(base + rng.normal(0, noise, (4, 4)), usable.copy()) for _ in range(ticks)]


def calibrated(seed=0, **kwargs):
    """Two rounds, as the session records them."""
    rng = np.random.default_rng(seed)
    relaxed = [recording(rng, alpha=1.6, beta=0.8, ticks=100, **kwargs) for _ in range(2)]
    focused = [recording(rng, alpha=1.1, beta=1.0, ticks=100, **kwargs) for _ in range(2)]
    return fit(relaxed, focused), rng


def test_auc():
    assert auc([3, 4, 5], [0, 1, 2]) == 1.0
    assert auc([0, 1, 2], [3, 4, 5]) == 0.0
    assert auc([1, 1], [1, 1]) == 0.5
    assert auc([1, 2, 3, 4], [2.5]) == 0.5
    assert np.isnan(auc([], [1.0]))


def test_levels_run_from_relaxed_to_focused():
    found, rng = calibrated()
    relaxed = [found.level(*tick) for tick in recording(rng, 1.6, 0.8, ticks=50)]
    focused = [found.level(*tick) for tick in recording(rng, 1.1, 1.0, ticks=50)]
    assert np.median(relaxed) == pytest.approx(0.0, abs=0.1)
    assert np.median(focused) == pytest.approx(1.0, abs=0.1)
    assert found.held_out_accuracy > 0.95 and found.held_out_auc > 0.99
    assert found.separation > 3


def test_direction_of_the_change_does_not_matter():
    """Some wearers' beta/alpha goes down when they concentrate."""
    rng = np.random.default_rng(3)
    found = fit([recording(rng, alpha=1.1, beta=1.0)], [recording(rng, alpha=1.6, beta=0.8)])
    assert found.level(*recording(rng, 1.6, 0.8, ticks=1)[0]) > 0.7
    assert found.level(*recording(rng, 1.1, 1.0, ticks=1)[0]) < 0.3


def test_losing_an_electrode_does_not_shift_the_level():
    found, rng = calibrated(noise=0.01)
    log_power, _ = recording(rng, 1.1, 1.0, ticks=1, noise=0.0)[0]
    everything = found.level(log_power, ALL)
    for lost in range(4):
        usable = ALL.copy()
        usable[lost] = False
        assert found.level(log_power, usable) == pytest.approx(everything, abs=0.05)


def test_a_gain_change_on_one_electrode_does_not_shift_the_level():
    found, rng = calibrated(noise=0.01)
    log_power, _ = recording(rng, 1.1, 1.0, ticks=1, noise=0.0)[0]
    louder = log_power.copy()
    louder[:, 2] += 0.5                 # every band of AF8 three times stronger
    assert found.level(louder, ALL) == pytest.approx(found.level(log_power, ALL), abs=1e-9)


def test_an_electrode_off_during_calibration_is_left_out():
    usable = np.array([True, False, True, True])
    found, rng = calibrated(usable=usable)
    assert np.isnan(found.baseline[:, 1]).all()
    assert not np.isnan(found.baseline[:, [0, 2, 3]]).any()
    log_power, _ = recording(rng, 1.1, 1.0, ticks=1)[0]
    log_power[:, 1] = 50.0              # whatever it reads later is ignored
    assert found.level(log_power, ALL) == pytest.approx(1.0, abs=0.3)
    assert found.level(log_power, np.array([False, True, False, False])) is None


def test_no_usable_electrode_gives_no_level():
    found, rng = calibrated()
    assert found.level(recording(rng, 1.1, 1.0, ticks=1)[0][0], np.zeros(4, bool)) is None


def test_recordings_that_cannot_be_used():
    rng = np.random.default_rng(4)
    with pytest.raises(CalibrationError, match="Too little"):
        fit([recording(rng, 1.6, 0.8, ticks=10)], [recording(rng, 1.1, 1.0, ticks=10)])
    with pytest.raises(CalibrationError, match="No electrode"):
        fit([recording(rng, 1.6, 0.8, usable=np.zeros(4, bool))],
            [recording(rng, 1.1, 1.0, usable=np.zeros(4, bool))])
    same = recording(rng, 1.6, 0.8, noise=0.0)
    with pytest.raises(CalibrationError, match="looked the same"):
        fit([same], [same])


def test_states_that_do_not_differ_are_reported_as_such():
    rng = np.random.default_rng(5)
    found = fit([recording(rng, 1.4, 0.9), recording(rng, 1.4, 0.9)],
                [recording(rng, 1.4, 0.9), recording(rng, 1.4, 0.9)])
    assert 0.3 < found.held_out_accuracy < 0.7


def drifting(start, ticks=100, total=400):
    """Recordings whose alpha only creeps upward with time, whatever the
    wearer is doing: no state in them at all."""
    rng = np.random.default_rng(start)
    return [(np.array([1.0, 1.4 + 0.4 * (start + i) / total, 0.9, 0.2])[:, None]
             + OFFSETS[None, :] + rng.normal(0, 0.03, (4, 4)), ALL.copy())
            for i in range(ticks)]


def test_a_drift_passes_for_a_state_with_one_round_but_not_with_two():
    one = fit([drifting(0, ticks=200)], [drifting(200, ticks=200)])
    assert one.validation == "stretches of one recording"
    assert one.held_out_accuracy > 0.8              # fooled: later is not "focused"
    two = fit([drifting(0), drifting(200)], [drifting(100), drifting(300)])
    assert two.validation == "whole rounds"
    assert two.held_out_accuracy < 0.6              # not fooled


def test_saved_calibration_reads_the_same():
    found, rng = calibrated(usable=np.array([True, True, False, True]))
    again = Calibration.from_dict(found.to_dict())
    tick = recording(rng, 1.3, 0.9, ticks=1)[0]
    assert again.level(*tick) == pytest.approx(found.level(*tick), abs=1e-3)


def test_latch_has_two_thresholds():
    latch = Latch(go=0.65, stop=0.35, smooth=0.0, rest=0.0)
    assert latch.update(0.0, 0.5) is False          # in between: stays stopped
    assert latch.update(0.1, 0.7) is True
    assert latch.update(0.2, 0.5) is True           # in between: stays going
    assert latch.update(0.3, 0.3) is False


def test_latch_smooths_the_level():
    latch = Latch(smooth=1.0, rest=0.0)
    latch.update(0.0, 0.0)
    assert latch.update(0.1, 1.0) is False          # one high tick is not enough
    now = 0.1
    while not latch.on:
        now += 0.1
        latch.update(now, 1.0)
    assert 0.8 < now < 1.5


def test_latch_rests_after_a_stop_but_never_delays_one():
    latch = Latch(smooth=0.0, rest=0.5)
    assert latch.update(0.0, 1.0) is True
    assert latch.update(0.1, 0.0) is False          # stopped at once
    assert latch.update(0.3, 1.0) is False          # still resting
    assert latch.update(0.7, 1.0) is True


def test_latch_stops_when_the_signal_is_lost():
    latch = Latch(smooth=0.0, rest=0.0, lost=0.6)
    latch.update(0.0, 1.0)
    assert latch.update(0.4, None) is True          # a short gap is ridden out
    assert latch.update(0.8, None) is False
    assert latch.update(3.0, None) is False and latch.level is None


def test_latch_thresholds_must_be_ordered():
    with pytest.raises(ValueError):
        Latch(go=0.3, stop=0.6)
