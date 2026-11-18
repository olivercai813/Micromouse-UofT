"""The scoring, on logs written by hand so the right answer is known."""

import random

import pytest

from mind2motor import score


def log(cues, car, lag=1.0, seconds=12.0, with_board=True, jitter=None):
    """Rows for a test in which `car(cue kind, seconds into the cue)` says
    whether the car is moving. With `jitter` each cue lasts a random time
    between seconds - jitter and seconds + jitter, as in a real test."""
    rows, now = [], 0.0
    rng = random.Random(0)
    for trial, kind in enumerate(cues, start=1):
        length = seconds if jitter is None else rng.uniform(seconds - jitter, seconds + jitter)
        for tick in range(int(length * 10)):
            into = tick / 10
            on = car(kind, into, lag)
            row = dict(t=f"{now:.2f}", stage="driving", cue=kind, trial=trial, armed=1,
                       left=75 if on else 0, right=75 if on else 0,
                       level=0.9 if on else 0.1)
            if with_board:
                row.update(ack_left=row["left"], ack_right=row["right"])
            rows.append(row)
            now += 0.1
    rows.append(dict(t=f"{now:.2f}", stage="done", armed=0, left=0, right=0))
    return rows


CUES = ["focus", "relax"] * 5


def obedient(kind, into, lag):
    return (kind == "focus") == (into >= lag)


def test_a_car_that_follows_scores_full_marks():
    result = score.analyse(log(CUES, obedient))
    drive = result["drive"]
    assert result["cues_scored"] == 10
    assert drive["steady_accuracy"] == 1.0 and drive["steady_accuracy_ci95"] == [1.0, 1.0]
    assert drive["reached"] == 10 and drive["bits_per_cue"] == 1.0
    assert drive["by_cue"]["focus"]["seconds_to_reach_median"] == pytest.approx(1.0)
    assert drive["false_starts_per_minute"] == 0 and drive["dropouts_per_minute"] == 0
    assert drive["level_auc"] == 1.0
    assert result["link"]["with_board_status"] == 1.0


def test_following_cues_of_random_length_is_significant():
    drive = score.analyse(log(CUES * 2, obedient, seconds=13.0, jitter=3.0))["drive"]
    assert drive["steady_accuracy"] == 1.0
    assert drive["chance"]["p_value"] < 0.01


def test_following_cues_of_fixed_length_cannot_be_significant():
    """Why the cues are of random length: with a fixed length the car's
    record, slid along by two cues, fits just as well, and nothing shows
    that it was the cues and not a rhythm the car was following."""
    drive = score.analyse(log(CUES * 2, obedient, seconds=13.0))["drive"]
    assert drive["steady_accuracy"] == 1.0
    assert drive["chance"]["p_value"] > 0.05


def test_a_handful_of_cues_gets_no_p_value():
    result = score.analyse(log(["focus", "relax", "focus", "relax"], obedient, jitter=3.0))
    assert result["drive"]["steady_accuracy"] == 1.0
    assert result["drive"]["chance"]["p_value"] is None
    assert "No p-value" in score.describe(result)


def test_a_car_that_never_moves_or_always_moves_scores_half():
    for constant in (False, True):
        drive = score.analyse(log(CUES, lambda *_: constant))["drive"]
        assert drive["steady_accuracy"] == 0.5
        assert drive["chance"]["p_value"] == 1.0
        assert drive["reached"] == 5 and drive["bits_per_cue"] == 0.0


def test_a_car_that_does_the_opposite_scores_nothing():
    drive = score.analyse(log(CUES, lambda kind, into, lag: not obedient(kind, into, lag)))["drive"]
    assert drive["steady_accuracy"] == 0.0 and drive["reached"] == 0
    assert drive["chance"]["p_value"] > 0.9


def test_a_car_on_its_own_rhythm_is_not_significant():
    """Starts and stops every 7 s whatever the cue: right half the time,
    and no better aligned with the cues than at any other delay."""
    clock = {"t": 0.0}

    def metronome(kind, into, lag):
        clock["t"] += 0.1
        return int(clock["t"] // 7) % 2 == 0

    rng = random.Random(1)
    rows, now = [], 0.0
    for trial, kind in enumerate(CUES * 2, start=1):
        for _ in range(int(rng.uniform(10, 16) * 10)):
            on = metronome(kind, 0, 0)
            rows.append(dict(t=f"{now:.2f}", stage="driving", cue=kind, trial=trial, armed=1,
                             left=75 * on, right=75 * on, ack_left=75 * on, ack_right=75 * on))
            now += 0.1
    drive = score.analyse(rows)["drive"]
    assert 0.3 < drive["steady_accuracy"] < 0.7
    assert drive["chance"]["p_value"] > 0.05


def test_a_slow_car_is_right_in_the_end_but_did_not_reach_in_time():
    drive = score.analyse(log(CUES, obedient, lag=7.0, seconds=20.0))["drive"]
    assert drive["steady_accuracy"] == 1.0
    assert drive["reached"] == 0
    assert drive["by_cue"]["relax"]["seconds_to_reach_median"] == pytest.approx(7.0)


def test_passing_through_the_asked_state_is_not_reaching_it():
    """Still moving from the cue before, then stops for good: under
    "focus" that is a carry-over, not a response."""
    def carry_over(kind, into, lag):
        return into < 3.0

    drive = score.analyse(log(["focus", "focus", "focus"], carry_over))["drive"]
    assert drive["reached"] == 0 and drive["by_cue"]["focus"]["seconds_to_reach_median"] is None


def test_flips_are_counted_by_direction():
    def twitchy(kind, into, lag):
        if kind == "relax":
            return 8.0 <= into < 8.5            # one false start in the steady half
        return not 9.0 <= into < 9.5            # one dropout

    drive = score.analyse(log(CUES, twitchy))["drive"]
    # five cues of each kind, each with 5.9 s of steady half and one flip
    assert drive["false_starts_per_minute"] == pytest.approx(60 * 5 / (5 * 5.9), abs=0.1)
    assert drive["dropouts_per_minute"] == pytest.approx(60 * 5 / (5 * 5.9), abs=0.1)
    assert 0.85 < drive["steady_accuracy"] < 0.95


def test_the_frame_sent_stands_in_when_the_board_said_nothing():
    result = score.analyse(log(CUES, obedient, with_board=False))
    assert result["drive"]["steady_accuracy"] == 1.0
    assert result["link"]["with_board_status"] == 0.0


def test_the_board_is_believed_over_the_frame():
    rows = log(CUES, obedient)
    for row in rows:
        if row.get("cue"):
            row["ack_left"] = row["ack_right"] = 0      # the board never moved
    assert score.analyse(rows)["drive"]["steady_accuracy"] == 0.5


def test_an_interrupted_cue_is_left_out():
    rows = log(["focus", "relax", "focus", "relax"], obedient)
    cut = next(i for i, row in enumerate(rows) if row.get("trial") == 2) + 40
    rows[cut:cut] = [dict(t="0", stage="paused", armed=0, left=0, right=0)]
    for row in rows[cut + 1:]:
        if row.get("trial") == 2:
            row["trial"] = 99                           # the cue given again
    result = score.analyse(rows)
    assert result["cues_interrupted"] == 1 and result["cues_scored"] == 4


def test_steering_is_scored_on_the_decoded_turn():
    rows, now = [], 0.0
    turns = {"left": 0.8, "right": -0.6, "centre": 0.0}
    for trial, kind in enumerate(["left", "centre", "right", "left", "right", "centre"], 1):
        for tick in range(70):
            turn = turns[kind] if tick >= 12 else 0.0 if kind != "centre" else 0.5
            rows.append(dict(t=f"{now:.2f}", stage="driving", cue=kind, trial=trial, armed=1,
                             left=0, right=0, turn=turn))
            now += 0.1
    result = score.analyse(rows)
    assert "drive" not in result
    assert result["steer"]["steady_accuracy"] == 1.0 and result["steer"]["reached"] == 6
    assert result["steer"]["by_cue"]["left"]["seconds_to_reach_median"] == pytest.approx(1.2)


def test_a_log_without_cues():
    rows = [dict(t="0.1", stage="driving", armed=1, left=75, right=75)] * 50
    assert score.analyse(rows) == dict(cues_scored=0, cues_interrupted=0)


def test_interval_and_information_helpers():
    assert score._wilson(0, 0) is None
    low, high = score._wilson(8, 10)
    assert 0.44 < low < 0.5 and 0.94 < high < 0.97
    assert score._bits(0.5) == 0.0 and score._bits(0.3) == 0.0 and score._bits(1.0) == 1.0
    assert score._bits(0.9) == pytest.approx(0.531, abs=0.001)


def test_the_text_mentions_what_matters():
    text = score.describe(score.analyse(log(CUES, obedient)))
    assert "100% of steady ticks" in text and "10 of 10 cues" in text and "p-value" in text
