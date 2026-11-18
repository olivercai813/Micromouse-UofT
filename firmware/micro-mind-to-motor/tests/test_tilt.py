import math

import numpy as np
import pytest

from mind2motor.board import STOP, Wheels
from mind2motor.tilt import Gravity, TiltError, fit_tilt, mix, shape


def poses(degrees, frame=np.eye(3), noise=0.0, seed=0):
    rng = np.random.default_rng(seed)

    def hold(angle):
        a = math.radians(angle)
        return [frame @ (np.array([0.0, math.sin(a), math.cos(a)]) + rng.normal(0, noise, 3))
                for _ in range(20)]

    return hold(0), hold(degrees), hold(-degrees)


def test_lean_is_signed_and_scaled_whatever_way_the_headband_sits():
    frame = np.linalg.qr(np.random.default_rng(7).normal(size=(3, 3)))[0]
    upright, left, right = poses(20, frame, noise=0.005)
    found = fit_tilt(upright, left, right)
    assert found.lean(np.median(left, axis=0)) == pytest.approx(1.0, abs=0.05)
    assert found.lean(np.median(right, axis=0)) == pytest.approx(-1.0, abs=0.05)
    assert found.lean(np.median(upright, axis=0)) == pytest.approx(0.0, abs=0.05)
    half = frame @ np.array([0.0, math.sin(math.radians(10)), math.cos(math.radians(10))])
    assert found.lean(half) == pytest.approx(0.5, abs=0.06)


def test_a_nod_is_not_a_turn():
    upright, left, right = poses(20)
    found = fit_tilt(upright, left, right)
    nod = np.array([math.sin(math.radians(25)), 0.0, math.cos(math.radians(25))])
    assert abs(found.lean(nod)) < 0.05


def test_tilts_that_cannot_be_told_apart():
    upright, left, right = poses(2)
    with pytest.raises(TiltError, match="looked the same"):
        fit_tilt(upright, left, right)
    with pytest.raises(TiltError, match="stopped"):
        fit_tilt(upright, left[:2], right)


def test_shape_has_a_dead_zone_and_saturates():
    assert shape(0.1) == 0.0 and shape(-0.19) == 0.0
    assert shape(1.0) == 1.0 and shape(-2.0) == -1.0
    assert shape(0.6) == pytest.approx(0.5)
    assert shape(-0.6) == pytest.approx(-0.5)


def test_mix():
    assert mix(False, 1.0, 75) == STOP
    assert mix(True, 0.0, 75) == Wheels(75, 75)
    assert mix(True, 1.0, 75) == Wheels(25, 75)         # left turn: left wheel slow
    assert mix(True, -1.0, 75) == Wheels(75, 25)
    assert mix(True, 0.5, 75) == Wheels(50, 75)
    assert mix(True, -3.0, 100) == Wheels(100, 33)


def test_gravity_smooths_and_goes_stale():
    gravity = Gravity(window=0.3, stale=1.0)
    assert gravity.value(0.0) is None
    gravity.update(0.0, [0, 0, 1])
    gravity.update(0.1, [0, 1, 0])
    between = gravity.value(0.1)
    assert 0.2 < between[1] < 0.4 and 0.6 < between[2] < 0.8
    assert gravity.value(1.2) is None
    gravity.update(5.0, [1, 0, 0])                      # after a gap: starts afresh
    assert list(gravity.value(5.0)) == [1, 0, 0]
