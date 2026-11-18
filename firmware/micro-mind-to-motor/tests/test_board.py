"""One set of expectations for two boards: the Python model the session is
rehearsed against, and the compiled firmware on an emulated processor.
The firmware cases are skipped when unicorn or the image is missing."""

from pathlib import Path
import random
import re
import shutil
import sys

import pytest

from mind2motor.board import (BUTTON_MS, FULL, IDENT, PER_PERCENT, PINS_REVERSE,
                              RISE_PER_MS, BoardModel)

FIRMWARE = Path(__file__).resolve().parents[1] / "firmware"
sys.path.insert(0, str(FIRMWARE))

FORWARD = 0b0110


def emulated():
    pytest.importorskip("unicorn")
    import emulate
    if not emulate.IMAGE.is_file():
        pytest.skip("firmware/build/mind2motor.bin has not been built")
    return emulate.Emulated()


@pytest.fixture(params=["model", "firmware"])
def board(request):
    return BoardModel() if request.param == "model" else emulated()


def status(board):
    """The newest status line as (who, left, right, rejected)."""
    lines = [line for line in board.take_output().decode().splitlines() if line.startswith("S ")]
    who, left, right, rejected = lines[-1].split()[1:]
    return who, int(left), int(right), int(rejected)


def drive(board, text, ms, every=100):
    """Repeat a frame every 100 ms for `ms`."""
    for _ in range(ms // every):
        board.feed(text)
        board.run(every)


def test_boot_is_idle_and_says_who_it_is(board):
    board.run(250)
    out = board.take_output()
    assert out.startswith(IDENT)
    assert out.count(b"S idle 0 0 0\n") == 2
    assert board.duty == (0, 0) and board.pins == 0


def test_a_frame_drives_both_wheels(board):
    drive(board, b"W75,75\n", 400)
    assert board.duty == (75 * PER_PERCENT, 75 * PER_PERCENT)
    assert board.pins == FORWARD
    assert status(board) == ("pc", 75, 75, 0)


def test_wheels_take_separate_duties(board):
    drive(board, b"W75,25\n", 400)
    assert board.duty == (75 * PER_PERCENT, 25 * PER_PERCENT)
    drive(board, b"W100,0\n", 400)
    assert board.duty == (FULL, 0) and board.pins == 0b0010
    assert status(board) == ("pc", 100, 0, 0)


def test_negative_duties_reverse_the_wheels(board):
    drive(board, b"W-60,-60\n", 400)
    assert board.duty == (-60 * PER_PERCENT, -60 * PER_PERCENT)
    assert board.pins == PINS_REVERSE
    assert status(board) == ("pc", -60, -60, 0)


def test_a_duty_rises_gradually_and_falls_at_once(board):
    board.run(10)
    board.feed(b"W100,100\n")
    board.run(20)
    partway = board.duty[0]
    assert 15 * RISE_PER_MS <= partway <= 20 * RISE_PER_MS
    board.run(100)
    board.feed(b"W100,100\n")
    board.run(100)
    assert board.duty == (FULL, FULL)
    board.feed(b"W10,100\n")
    board.run(3)
    assert board.duty == (10 * PER_PERCENT, FULL)
    board.feed(b"W0,0\n")
    board.run(3)
    assert board.duty == (0, 0) and board.pins == 0


def test_silence_stops_the_car(board):
    drive(board, b"W75,75\n", 300)
    board.run(280)                      # 380 ms after the last frame
    assert board.duty[0] == 75 * PER_PERCENT
    board.run(40)                       # 420 ms after it
    assert board.duty == (0, 0) and board.pins == 0
    board.run(100)
    assert status(board)[0] == "idle"


@pytest.mark.parametrize("line", [b"W101,0", b"W0,101", b"W-101,0", b"W0,-101",
                                  b"W75", b"W75,", b"W,75", b"X75,75",
                                  b"W 75,75", b"W75,75,75", b"W1000,5", b"w75,75", b"1",
                                  b"W+5,5", b"W75;75", b"W75,75 ", b"W075,0750",
                                  b"W10,10W10,10W10,10"])
def test_lines_that_are_not_frames_are_counted_and_do_nothing(board, line):
    for _ in range(6):
        board.feed(line + b"\n")
        board.run(100)
    assert board.duty == (0, 0) and board.pins == 0
    assert status(board) == ("idle", 0, 0, 6)


def test_lines_that_are_not_frames_do_not_keep_the_car_going(board):
    drive(board, b"W75,75\n", 300)
    drive(board, b"W999,999\n", 500)
    assert board.duty == (0, 0)
    assert status(board)[0] == "idle"


@pytest.mark.parametrize("line", [b"W075,075\n", b"W75,75\r\n", b"\n\nW75,75\n", b"W75,75\n\r\n"])
def test_frames_written_slightly_differently(board, line):
    drive(board, line, 400)
    assert status(board) == ("pc", 75, 75, 0)


def test_question_mark_gets_the_identification(board):
    board.run(50)
    board.take_output()
    board.feed(b"?\n")
    board.run(20)
    assert board.take_output() == IDENT


def test_a_damaged_byte_costs_only_its_own_line(board):
    drive(board, b"W50,50\n", 300)
    board.feed(b"W10", damaged=True)
    board.feed(b"0,100\n")
    board.run(100)
    assert status(board) == ("pc", 50, 50, 1)       # the damaged frame was not obeyed
    drive(board, b"W20,20\n", 200)
    assert status(board) == ("pc", 20, 20, 1)


def test_the_newest_frame_of_a_burst_wins(board):
    board.feed(b"W10,10\nW20,20\nW0,0\nW60,40\n")
    board.run(300)
    assert status(board) == ("pc", 60, 40, 0)


def test_button_drives_when_no_frames_come(board):
    board.run(100)
    board.button = True
    board.run(BUTTON_MS - 5)
    assert board.duty == (0, 0)                     # not held long enough yet
    board.run(250)
    assert board.duty == (75 * PER_PERCENT, 75 * PER_PERCENT) and board.pins == FORWARD
    assert status(board) == ("btn", 75, 75, 0)
    board.button = False
    board.run(BUTTON_MS + 5)
    assert board.duty == (0, 0) and board.pins == 0


def test_button_is_ignored_while_frames_come(board):
    board.button = True
    drive(board, b"W0,0\n", 500)
    assert board.duty == (0, 0)
    assert status(board) == ("pc", 0, 0, 0)
    board.run(700)                                  # the frames stop: the button takes over
    assert status(board) == ("btn", 75, 75, 0)


def test_a_glitch_on_the_button_is_not_a_press(board):
    board.run(100)
    for _ in range(20):
        board.button = True
        board.run(3)
        board.button = False
        board.run(3)
    assert board.duty == (0, 0)


def test_status_comes_ten_times_a_second(board):
    board.run(1000)
    lines = board.take_output().decode().splitlines()
    assert sum(line.startswith("S ") for line in lines) == 10


def test_model_and_firmware_tell_the_same_story():
    """A long made-up exchange, fed to both; every status line must agree
    (a duty still rising may differ by the millisecond a frame took to
    arrive)."""
    firmware, model = emulated(), BoardModel()
    rng = random.Random(5)
    frames = [b"W75,75\n", b"W0,0\n", b"W100,30\n", b"W20,90\n", b"junk\n", b"W101,5\n",
              b"?\n", b"", b"", b"W55,55\r\n"]
    for step in range(300):
        frame = rng.choice(frames)
        button = rng.random() < 0.15
        for board in (firmware, model):
            board.button = button
            board.feed(frame)
            board.run(rng.choice((40, 100, 100, 100, 250)) if board is firmware else 0)
        model.run(firmware.ms - model.ms)
    ours = re.findall(r"S (\w+) (-?\d+) (-?\d+) (\d+)", model.take_output().decode())
    theirs = re.findall(r"S (\w+) (-?\d+) (-?\d+) (\d+)", firmware.take_output().decode())
    assert len(ours) == len(theirs) > 250
    for mine, yours in zip(ours, theirs):
        assert mine[0] == yours[0] and mine[3] == yours[3]
        assert abs(int(mine[1]) - int(yours[1])) <= 1 and abs(int(mine[2]) - int(yours[2])) <= 1
    assert {line[0] for line in ours} == {"pc", "btn", "idle"}


def test_firmware_sets_the_chip_up_as_documented():
    emulate = pytest.importorskip("emulate")
    board = emulated()
    board.run(5)
    peek = board.peek
    assert peek(emulate.RCC_AHB1ENR) == 0x0E and peek(emulate.RCC_APB1ENR) == 0x00040001
    # PB0..PB3 outputs, PB10 and PB11 on the timer, PB4 left as it was.
    assert peek(emulate.PORTB + emulate.MODE) == 0x00A00255
    assert peek(emulate.PORTB + emulate.ALT_HIGH) == 0x00001100
    assert peek(emulate.PORTC + emulate.PULL) == 0x08000000
    assert peek(emulate.PORTD + emulate.MODE) == 0x000A0000
    assert peek(emulate.PORTD + emulate.ALT_HIGH) == 0x00000077
    tim2 = emulate.TIM2
    assert peek(tim2 + 0x28) == 0 and peek(tim2 + 0x2C) == FULL - 1     # 16 MHz / 32000 = 500 Hz
    assert peek(tim2 + 0x1C) == 0x6868 and peek(tim2 + 0x20) == 0x1100
    assert peek(tim2) == 0x81
    assert peek(emulate.USART3_BRR) == 139 and peek(emulate.USART3_CR1) == 0x200C
    assert peek(emulate.SYST_RVR) == 15999 and peek(emulate.SYST_CSR) & 7 == 5


def test_firmware_survives_a_receiver_overrun():
    board = emulated()
    drive(board, b"W50,50\n", 300)
    board.feed(b"W90,")
    board.overrun()                                 # the rest of this frame was lost
    board.feed(b"W90,90\n")
    board.run(100)
    assert status(board) == ("pc", 50, 50, 1)
    drive(board, b"W30,30\n", 200)
    assert status(board) == ("pc", 30, 30, 1)


def test_committed_image_is_what_the_source_builds(tmp_path):
    """Only where the compiler is installed."""
    build = pytest.importorskip("build")
    import os
    gcc = shutil.which("arm-none-eabi-gcc") or os.environ.get("ARM_GCC_BIN")
    if not gcc:
        pytest.skip("no arm-none-eabi-gcc here")
    committed = (FIRMWARE / "build" / "mind2motor.bin").read_bytes()
    build.build(os.environ.get("ARM_GCC_BIN"), quiet=True)
    assert (FIRMWARE / "build" / "mind2motor.bin").read_bytes() == committed
