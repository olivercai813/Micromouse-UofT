import math
import socket
import time

import numpy as np

from questpath import osc
from questpath.clock import RealClock
from questpath.eeg import EEGBuffer


def test_osc_message_round_trip():
    data = osc.message("/muse/eeg", [1.5, -2.0, 3.25, 4.0, 5.0])
    assert len(data) % 4 == 0
    assert osc.parse(data) == [("/muse/eeg", [1.5, -2.0, 3.25, 4.0, 5.0])]


def test_osc_bundle_and_rubbish():
    one = osc.message("/muse/eeg", [1, 2, 3, 4])
    two = osc.message("/muse/elements/alpha_absolute", [0.5, 0.5, 0.5, 0.5])
    bundle = b"#bundle\x00" + b"\x00" * 8
    for part in (one, two):
        bundle += len(part).to_bytes(4, "big") + part
    parsed = osc.parse(bundle)
    assert [address for address, _ in parsed] == ["/muse/eeg", "/muse/elements/alpha_absolute"]
    assert osc.parse(b"not osc at all") == []
    assert osc.parse(b"") == []
    assert osc.parse(one[:10]) == []


def test_only_raw_eeg_counts_as_eeg():
    assert osc.is_eeg("/muse/eeg")
    assert osc.is_eeg("/eeg")
    assert not osc.is_eeg("/muse/elements/alpha_absolute")
    assert not osc.is_eeg("/person1/eeg")            # MuseLog's averaged band powers
    assert not osc.is_eeg("/muse/acc")


def feed(buffer, rate, seconds, delay=0.08, burst=0.0, start=50.0, lose=()):
    """Samples of a 12 Hz sine taken at `rate`; returns their true times."""
    times = []
    n = int(rate * seconds)
    for k in range(n):
        if k in lose:
            continue
        t = start + k / rate
        arrival = t + delay
        if burst:
            arrival = math.ceil(arrival / burst) * burst
        buffer.push(arrival, [math.sin(2 * math.pi * 12 * t)] * 5)
        times.append(t)
    return times


def test_rate_is_recognised():
    for rate in (256.0, 64.0):
        buffer = EEGBuffer()
        feed(buffer, rate, 12)
        assert buffer.rate == rate and buffer.rate_locked
        assert buffer.names() == ["TP9", "AF7", "AF8", "TP10", "AUX"]


def test_sample_clock_is_steady_however_bursty_the_delivery():
    for burst in (0.0, 0.25, 0.75):
        buffer = EEGBuffer()
        feed(buffer, 256.0, 12, delay=0.08, burst=burst)
        x, rate, cover = buffer.window(55.0, 57.0)
        assert cover > 0.99 and abs(len(x) - 512) <= 1, (burst, len(x))
        # the samples in the window are the ones taken 55.0 - delay .. 57.0 - delay:
        # their 12 Hz phase says how far the rebuilt clock is off
        t = np.arange(len(x)) / rate
        phase = np.angle(np.sum(x[:, 0] * np.exp(-2j * np.pi * 12 * t)))
        shift = (phase + math.pi / 2) / (2 * math.pi * 12)        # sin -> -90 degrees
        got = (shift - (55.0 - 0.08 - 50.0)) % (1 / 12)
        got = min(got, 1 / 12 - got)
        assert got < 0.012, "clock off by %.1f ms with %.2f s bursts" % (got * 1000, burst)


def test_latest_follows_the_stream():
    buffer = EEGBuffer()
    assert buffer.latest() == float("-inf")
    feed(buffer, 256.0, 5)
    assert abs(buffer.latest() - (55.0 + 0.08)) < 0.02


def test_missing_eeg_shows_as_low_cover():
    buffer = EEGBuffer()
    feed(buffer, 256.0, 9, lose=set(range(256 * 5, 256 * 6)))     # one second never arrives
    _, _, cover = buffer.window(54.9, 56.3)
    assert cover < 0.5
    _, _, cover = buffer.window(51.0, 52.5)
    assert cover > 0.98
    _, _, cover = buffer.window(57.0, 58.5)                       # and the stream is usable again after it
    assert cover > 0.98


def test_a_long_silence_starts_a_new_stretch():
    buffer = EEGBuffer()
    feed(buffer, 256.0, 4, start=50.0)
    feed(buffer, 256.0, 4, start=60.0)
    x, _, cover = buffer.window(61.0, 62.0)
    assert cover > 0.98                              # timed from its own arrivals, not the old count
    x, _, cover = buffer.window(56.0, 58.0)
    assert cover == 0.0


def test_quality_reports_rate_and_rms():
    buffer = EEGBuffer()
    feed(buffer, 256.0, 4)
    rate, rms = buffer.quality()
    assert abs(rate - 256) < 3
    assert all(abs(v - 0.707) < 0.03 for v in rms)


def test_old_samples_are_dropped():
    buffer = EEGBuffer(keep_s=5.0)
    feed(buffer, 256.0, 40)
    assert len(buffer._arrival) < 256 * 30


def test_receiver_takes_eeg_from_a_real_socket():
    got = []
    clock = RealClock()
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    receiver = osc.OscReceiver(port, lambda t, values: got.append(values), clock).start()
    try:
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for k in range(20):
            sender.sendto(osc.message("/muse/eeg", [k, 1, 2, 3, 4]), ("127.0.0.1", port))
            sender.sendto(osc.message("/muse/elements/horseshoe", [1, 1, 1, 1]), ("127.0.0.1", port))
        end = time.time() + 3
        while len(got) < 20 and time.time() < end:
            time.sleep(0.02)
    finally:
        receiver.close()
    assert len(got) == 20 and got[5] == [5.0, 1.0, 2.0, 3.0, 4.0]
    assert receiver.other == 20 and receiver.senders == {"127.0.0.1"}
