import numpy as np
import pytest

from questpath.detect import (DetectConfig, RowEvidence, Scorer, bandlimit, canonical_correlation,
                              control_rates, references, strength)
from questpath.display import pick_rate

RATE = 256.0


def eeg(seconds, hz=None, amp=0.0, channels=5, seed=0, rate=RATE, alpha=4.0):
    """Noise like a headband's (broadband, alpha, drift), plus a response on
    the last electrode."""
    rng = np.random.default_rng(seed)
    n = int(seconds * rate)
    t = np.arange(n) / rate
    x = rng.normal(0, 8, (n, channels)) + np.cumsum(rng.normal(0, 0.15, (n, channels)), axis=0)
    x += (alpha * np.sin(2 * np.pi * 10 * t + rng.uniform(0, 6)))[:, None]
    if hz:
        x[:, -1] += amp * (np.sin(2 * np.pi * hz * t + 1.0) + 0.5 * np.sin(4 * np.pi * hz * t))
    return x


@pytest.mark.parametrize("refresh, want", [(72, 12.0), (90, 15.0), (120, 12.0), (60, 15.0), (144, 12.0)])
def test_flicker_rates_are_exact_and_clear_of_alpha(refresh, want):
    hz, k = pick_rate(refresh, 13.0)
    assert hz == want and hz * 2 * k == refresh


def test_control_rates_avoid_alpha_and_the_harmonic():
    for hz in (12.0, 15.0):
        others = control_rates(hz, 6.0, 40.0)
        assert len(others) == 6
        assert all(not 8.0 <= r <= 11.6 for r in others)
        assert all(abs(r - hz) >= 1.2 and abs(r - 2 * hz) > 1.0 for r in others)
    assert len(control_rates(12.0, 6.0, 28.8)) == 6      # the app's reduced 64 Hz stream


def test_bandlimit_keeps_the_band_and_drops_the_rest():
    t = np.arange(1024) / RATE
    x = (np.sin(2 * np.pi * 12 * t) + 5 * np.sin(2 * np.pi * 60 * t) + 20 * t)[:, None]
    y = bandlimit(x, RATE, 6, 40, whiten=False)
    spectrum = np.abs(np.fft.rfft(y[:, 0]))
    freqs = np.fft.rfftfreq(1024, 1 / RATE)
    assert spectrum[np.argmin(abs(freqs - 12))] > 50 * spectrum[np.argmin(abs(freqs - 60))]
    assert abs(y.mean()) < 0.05


def test_canonical_correlation_ends_of_the_scale():
    t = np.arange(512) / RATE
    refs = references(512, RATE, 12.0, 2, 40.0)
    assert refs.shape == (512, 4)
    assert canonical_correlation(np.sin(2 * np.pi * 12 * t + 0.4)[:, None], refs) > 0.999
    rng = np.random.default_rng(0)
    assert canonical_correlation(rng.normal(size=(512, 1)), refs) < 0.3
    assert canonical_correlation(np.zeros((512, 2)), refs) == 0.0


def test_a_response_scores_high_and_silence_scores_low():
    scorer = Scorer()
    for seed in range(20):                               # let it learn the spread
        scorer.score(eeg(2.6, seed=100 + seed), RATE, 12.0)
    hits = np.array([scorer.score(eeg(2.6, 12.0, 3.0, seed=seed), RATE, 12.0).z for seed in range(20)])
    misses = np.array([scorer.score(eeg(2.6, seed=50 + seed), RATE, 12.0).z for seed in range(20)])
    assert np.median(hits) > 3.0
    assert abs(np.median(misses)) < 1.0
    assert (hits[:, None] > misses[None, :]).mean() > 0.97       # nearly every response beats every silence


def test_scores_on_noise_behave_like_z_scores():
    scorer = Scorer()
    z = np.array([scorer.score(eeg(2.6, seed=seed), RATE, 15.0).z for seed in range(700)])[100:]
    assert abs(z.mean()) < 0.35
    assert 0.8 < z.std() < 1.25
    assert (z > 3.0).mean() < 0.01                       # a bell curve gives 0.0013


def test_alpha_does_not_pass_for_a_response():
    scorer = Scorer()
    for seed in range(20):
        scorer.score(eeg(2.6, seed=seed), RATE, 12.0)
    strong_alpha = [scorer.score(eeg(2.6, seed=200 + seed, alpha=15.0), RATE, 12.0).z for seed in range(30)]
    assert np.median(strong_alpha) < 1.0 and max(strong_alpha) < 3.5


def test_flashes_that_cannot_be_scored_say_why():
    scorer = Scorer()
    assert scorer.score(eeg(0.3), RATE, 12.0).why == "too little EEG"
    assert scorer.score(np.zeros((600, 4)), RATE, 12.0).why == "no usable electrode"
    assert scorer.score(eeg(2.6), RATE, 45.0).why == "flicker rate outside the band"
    assert not scorer.score(eeg(2.6) * 1000, RATE, 12.0).ok       # every electrode off the head


def test_a_dead_electrode_is_left_out():
    scorer = Scorer()
    x = eeg(2.6, 12.0, 4.0)
    x[:, 1] = 0.0
    x[:, 2] *= 500
    score = scorer.score(x, RATE, 12.0)
    assert score.ok and score.channels == 3


def test_the_signal_check_finds_the_electrode_that_carries_the_flicker():
    scorer = Scorer()
    flicker = [(eeg(2.6, 12.0, 2.5, seed=s), RATE, 12.0) for s in range(4)]
    steady = [(eeg(2.6, seed=10 + s), RATE, 12.0) for s in range(4)]
    tried = scorer.choose_channels(flicker, steady, 5)
    assert scorer.channels is not None and 4 in scorer.channels
    assert tried[(4,)] > tried[(0,)]
    # and scoring then uses it, even if it is asked about a narrower recording
    assert scorer.score(eeg(2.6, 12.0, 2.5, seed=77), RATE, 12.0).channels == len(scorer.channels)
    assert scorer.score(eeg(2.6, seed=78, channels=4), RATE, 12.0).ok


def test_strength_is_monotonic():
    values = strength(np.array([0.1, 0.3, 0.6, 0.9]))
    assert np.all(np.diff(values) > 0)


def test_row_decision_rules():
    config = DetectConfig(threshold=3.0, margin=1.5, min_rounds=1, max_rounds=3)
    row = RowEvidence(3, config)
    assert row.decide() == ("more", None)
    for col, z in ((0, 0.2), (1, 5.0), (2, -0.5)):
        row.add(col, z)
    assert row.rounds() == 1 and row.decide() == ("pick", 1)

    close = RowEvidence(3, config)                       # two squares both respond: not clear
    for col, z in ((0, 4.0), (1, 4.5), (2, 0.0)):
        close.add(col, z)
    assert close.decide() == ("more", None)

    weak = RowEvidence(3, config)                        # builds up over rounds
    for _ in range(2):
        for col, z in ((0, 0.0), (1, 2.4), (2, 0.1)):
            weak.add(col, z)
    assert weak.decide() == ("pick", 1)                  # 4.8 / sqrt(2) = 3.4

    nothing = RowEvidence(3, config)
    for _ in range(3):
        for col in range(3):
            nothing.add(col, 0.3)
    assert nothing.decide() == ("none", None)


def test_a_half_finished_round_is_not_decided():
    row = RowEvidence(3, DetectConfig())
    row.add(1, 9.0)
    row.add(0, 0.0)
    assert row.decide() == ("more", None)
