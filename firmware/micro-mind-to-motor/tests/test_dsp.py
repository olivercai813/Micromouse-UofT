import numpy as np
import pytest

from mind2motor.dsp import (BANDS, FLAT, OK, RAILED, WILD, BandMeter,
                            bandpass_sections, notch_section, response)

RATE = 256.0


def tone(hz, amplitude, seconds, offset=840.0):
    t = np.arange(int(seconds * RATE)) / RATE
    wave = offset + amplitude * np.sin(2 * np.pi * hz * t)
    return np.tile(wave[:, None], (1, 4))


@pytest.mark.parametrize("low, high", BANDS.values())
def test_bandpass_is_a_butterworth(low, high):
    sections = bandpass_sections(low, high, RATE)
    centre = np.sqrt(low * high)
    gains = response(sections, [low, centre, high, low / 4, min(high * 3, 120)], RATE)
    assert gains[0] == pytest.approx(np.sqrt(0.5), abs=2e-3)     # -3 dB at each edge
    assert gains[2] == pytest.approx(np.sqrt(0.5), abs=2e-3)
    assert gains[1] == pytest.approx(1.0, abs=0.02)
    assert gains[3] < 0.01 and gains[4] < 0.05


def test_two_pole_version_agrees_with_scipy_too():
    signal = pytest.importorskip("scipy.signal")
    freqs = np.linspace(0.5, 120.0, 200)
    reference = signal.butter(2, [8.0, 13.0], btype="bandpass", fs=RATE, output="sos")
    _, expected = signal.sosfreqz(reference, worN=freqs, fs=RATE)
    assert np.allclose(response(bandpass_sections(8.0, 13.0, RATE, poles=2), freqs, RATE),
                       np.abs(expected), atol=1e-6)


@pytest.mark.parametrize("low, high", BANDS.values())
def test_bandpass_agrees_with_scipy(low, high):
    signal = pytest.importorskip("scipy.signal")
    freqs = np.linspace(0.5, 120.0, 400)
    reference = signal.butter(4, [low, high], btype="bandpass", fs=RATE, output="sos")
    _, expected = signal.sosfreqz(reference, worN=freqs, fs=RATE)
    ours = response(bandpass_sections(low, high, RATE), freqs, RATE)
    assert np.allclose(ours, np.abs(expected), atol=1e-6)


def test_band_does_not_fit_the_rate():
    with pytest.raises(ValueError):
        bandpass_sections(30.0, 45.0, 64.0)


def test_notch_removes_the_mains_only():
    gains = response(notch_section(60.0, RATE), [10.0, 40.0, 60.0], RATE)
    assert gains[0] == pytest.approx(1.0, abs=0.01)
    assert gains[1] > 0.9
    assert gains[2] < 1e-6


def test_a_tone_lands_in_its_band():
    meter = BandMeter(RATE)
    meter.push(tone(10.0, 20.0, 6.0))
    power = dict(zip(meter.names, meter.power[:, 0]))
    assert meter.ready
    assert power["alpha"] == pytest.approx(20.0 ** 2 / 2, rel=0.1)
    assert power["theta"] < power["alpha"] / 20
    assert power["beta"] < power["alpha"] / 20
    assert power["gamma"] < power["alpha"] / 1000


def test_mains_hum_stays_out_of_gamma():
    with_notch, without = BandMeter(RATE), BandMeter(RATE, mains=None)
    hum = tone(60.0, 50.0, 6.0)
    with_notch.push(hum)
    without.push(hum)
    gamma = with_notch.names.index("gamma")
    assert with_notch.power[gamma, 0] < 0.01
    assert without.power[gamma, 0] > 100 * with_notch.power[gamma, 0]


def test_block_size_does_not_matter():
    rng = np.random.default_rng(0)
    signal = 800 + rng.normal(0, 20, (1024, 4))
    whole, pieces = BandMeter(RATE), BandMeter(RATE)
    whole.push(signal)
    for start in range(0, 1024, 12):
        pieces.push(signal[start:start + 12])
    assert np.allclose(whole.power, pieces.power)


def test_not_ready_until_settled_and_again_after_a_reset():
    meter = BandMeter(RATE)
    meter.push(tone(10.0, 20.0, 1.0))
    assert not meter.ready
    meter.push(tone(10.0, 20.0, 2.0))
    assert meter.ready
    meter.reset()
    assert not meter.ready and not meter.power.any()


def test_electrode_states():
    rng = np.random.default_rng(1)
    signal = 800 + rng.normal(0, 15, (512, 4))
    signal[:, 1] = 800.0                            # nothing coming in
    signal[:, 2] = 0.0                              # stuck at the bottom rail
    signal[:, 3] = 800 + rng.normal(0, 400, 512)    # far too noisy
    signal[:, 3] = np.clip(signal[:, 3], 100, 1500)
    meter = BandMeter(RATE)
    meter.push(signal)
    state, spread = meter.electrodes()
    assert list(state) == [OK, FLAT, RAILED, WILD]
    assert spread[0] == pytest.approx(15, rel=0.3)


def test_a_blink_does_not_condemn_an_electrode():
    rng = np.random.default_rng(2)
    signal = 800 + rng.normal(0, 15, (512, 4))
    signal[300:360, 1] += 150 * np.hanning(60)      # a blink on AF7
    meter = BandMeter(RATE)
    meter.push(signal)
    assert meter.electrodes()[0][1] == OK
