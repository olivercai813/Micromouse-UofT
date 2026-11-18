import struct

import numpy as np
import pytest

from mind2motor.dsp import BandMeter
from mind2motor.senses import Senses
from mind2motor.sources import (G_PER_COUNT, UV_PER_COUNT, OscHeadband, RowBuilder,
                                SyntheticWearer, control_frame, eeg_packet,
                                motion_packet, osc_messages)


def pack_eeg(sequence, counts):
    """Twelve 12-bit values behind a 16-bit counter, as the headband packs them."""
    bits = "".join(f"{value:012b}" for value in counts)
    return struct.pack(">H", sequence) + int(bits, 2).to_bytes(18, "big")


def test_eeg_packet():
    counts = [0, 1, 2047, 2048, 4095, 1365, 2730, 7, 4000, 16, 255, 256]
    sequence, samples = eeg_packet(pack_eeg(513, counts))
    assert sequence == 513
    assert np.allclose(samples, np.array(counts) * UV_PER_COUNT)
    assert samples.max() == pytest.approx(1682.0, abs=0.5)


def test_motion_packet():
    data = struct.pack(">H9h", 7, 100, -200, 16384, 300, -400, 16384, 200, -300, 16384)
    assert np.allclose(motion_packet(data), np.array([200, -300, 16384]) * G_PER_COUNT)
    assert motion_packet(data)[2] == pytest.approx(1.0)


def test_control_frame():
    assert control_frame("d") == b"\x02d\n"
    assert control_frame("p21") == b"\x04p21\n"


def test_rows_wait_for_all_four_electrodes():
    rows = RowBuilder()
    samples = [np.full(12, float(e)) for e in range(4)]
    assert rows.add(0, 10, samples[0]) is None
    assert rows.add(2, 10, samples[2]) is None
    assert rows.add(1, 11, samples[1]) is None       # the next packet starts early
    assert rows.add(1, 10, samples[1]) is None
    block = rows.add(3, 10, samples[3])
    assert block.shape == (12, 4)
    assert np.array_equal(block[0], [0.0, 1.0, 2.0, 3.0])


def test_rows_give_up_on_a_packet_that_lost_an_electrode():
    rows = RowBuilder(keep=3)
    for sequence in range(65530, 65546):             # across the counter's wrap
        for electrode in range(4):
            if sequence == 65531 and electrode == 2:
                continue                              # one notification never came
            rows.add(electrode, sequence % 65536, np.zeros(12))
    assert rows.dropped == 1 and len(rows._open) == 0


def osc(address, *values):
    def text(s):
        raw = s.encode() + b"\0"
        return raw + b"\0" * (-len(raw) % 4)
    tags = "," + "".join("f" if isinstance(v, float) else "i" for v in values)
    body = b"".join(struct.pack(">f" if isinstance(v, float) else ">i", v) for v in values)
    return text(address) + text(tags) + body


def test_osc_message_and_bundle():
    single = osc("/muse/eeg", 800.5, 810.25, 790.0, 805.0, 1.0)
    assert list(osc_messages(single)) == [("/muse/eeg", [800.5, 810.25, 790.0, 805.0, 1.0])]
    other = osc("/muse/acc", 0.0, 0.5, 1.0)
    bundle = (b"#bundle\0" + bytes(8) + struct.pack(">i", len(single)) + single
              + struct.pack(">i", len(other)) + other)
    assert [address for address, _ in osc_messages(bundle)] == ["/muse/eeg", "/muse/acc"]
    assert list(osc_messages(osc("/muse/elements/horseshoe", 1, 2, 4, 1)))[0][1] == [1, 2, 4, 1]


@pytest.mark.parametrize("junk", [b"", b"\0\0\0\0", b"not osc at all", b"/muse/eeg\0\0\0,ffff\0\0\0\x01",
                                  b"#bundle\0" + bytes(8) + b"\x7f\xff\xff\xff"])
def test_osc_junk_is_skipped(junk):
    assert list(osc_messages(junk)) == []


def stream_osc(source, clock, rate, seconds):
    for index in range(int(rate * seconds)):
        clock[0] = index / rate
        source.datagram_received(osc("/muse/eeg", 800.0, 801.0, 802.0, 803.0), None)


def test_osc_stream_rate_is_measured():
    clock, notes = [0.0], []
    senses = Senses()
    source = OscHeadband(senses, notes.append, lambda: clock[0])
    stream_osc(source, clock, 256, 6.0)
    assert source.rate == 256.0 and senses.meter.rate == 256.0
    assert senses.look(clock[0]) is not None
    source.datagram_received(osc("/muse/acc", 0.0, 0.0, 1.0), None)
    assert senses.gravity.value(clock[0])[2] == pytest.approx(1.0)


def test_osc_stream_at_a_quarter_rate_is_refused_with_a_reason():
    clock, notes = [0.0], []
    senses = Senses()
    source = OscHeadband(senses, notes.append, lambda: clock[0])
    stream_osc(source, clock, 64, 7.0)
    assert source.rate is None and senses.meter is None
    assert "64 Hz" in senses.trouble and len(notes) == 1


def test_made_up_wearer_is_repeatable_and_has_more_alpha_relaxed():
    def alpha_and_beta(cue, seed=3):
        wearer, senses = SyntheticWearer(seed=seed), Senses()
        wearer.instruct(cue, 0.0, calibrating=True)
        for tick in range(1, 101):
            wearer.advance(tick / 10, senses)
        power = dict(zip(senses.meter.names, senses.meter.power.mean(axis=1)))
        return power["alpha"], power["beta"], senses

    relaxed_alpha, relaxed_beta, senses = alpha_and_beta("relax")
    focused_alpha, focused_beta, _ = alpha_and_beta("focus")
    assert relaxed_alpha > 3 * focused_alpha
    assert focused_beta > 1.3 * relaxed_beta
    assert alpha_and_beta("relax")[0] == relaxed_alpha          # same seed, same signal
    assert alpha_and_beta("relax", seed=4)[0] != relaxed_alpha
    look = senses.look(10.0)
    assert look.usable.all() and (look.spread < 40).all()


def test_made_up_wearer_can_have_no_difference_between_states():
    def alpha(cue):
        wearer, senses = SyntheticWearer(seed=3, contrast=0.0), Senses()
        wearer.instruct(cue, 0.0, calibrating=True)
        for tick in range(1, 101):
            wearer.advance(tick / 10, senses)
        return senses.meter.power.mean()

    assert alpha("relax") == pytest.approx(alpha("focus"), rel=1e-9)


def test_made_up_wearer_tilts_about_a_tilted_headband():
    wearer, senses = SyntheticWearer(seed=1), Senses()
    held = {}
    now = 0.0
    for cue in ("upright", "left", "right"):
        wearer.instruct(cue, now, calibrating=True)
        for _ in range(40):
            now += 0.1
            wearer.advance(now, senses)
        held[cue] = senses.gravity.value(now)
    for vector in held.values():
        assert np.linalg.norm(vector) == pytest.approx(1.0, abs=0.05)
    assert np.linalg.norm(held["left"] - held["right"]) == pytest.approx(
        2 * np.sin(np.radians(22)), abs=0.05)


def test_dropped_electrode_reads_as_railed():
    wearer, senses = SyntheticWearer(seed=1), Senses()
    wearer.dropouts.append((1, 3.0, 99.0))
    for tick in range(1, 61):
        wearer.advance(tick / 10, senses)
    assert senses.look(6.0).contact() == "TP9 ok  AF7 railed  AF8 ok  TP10 ok"


def test_a_gap_in_the_stream_restarts_the_filters():
    wearer, senses = SyntheticWearer(seed=1), Senses()
    for tick in range(1, 41):
        wearer.advance(tick / 10, senses)
    assert senses.look(4.0) is not None
    assert senses.look(4.7) is None                 # nothing for 0.7 s: stale
    senses.on_eeg(6.0, np.full((12, 4), 800.0), 256.0)   # the stream comes back
    assert isinstance(senses.meter, BandMeter) and senses.meter.seen == 12
    assert senses.look(6.0) is None                 # and has to settle again
