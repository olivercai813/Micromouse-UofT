"""The link to the board: which firmware is there, what gets sent to each."""

import time

import pytest

from mind2motor.board import STOP, CarLink, ModelLink, NoCar, Wheels, Wire, frame
from helpers import MusecarBoard, TcpBoard


def test_frames():
    assert frame(Wheels(75, 25)) == b"W75,25\n"
    assert frame(Wheels(-60, -40)) == b"W-60,-40\n"
    assert frame(STOP) == b"W0,0\n"


def test_only_stop_is_sent_until_the_board_is_known():
    wire = Wire()
    assert wire.encode(Wheels(75, 75)) == b"W0,0\n" and wire.sent == STOP
    wire.decode(b"I mind2motor 1\nS idle 0 0 0\n", now=1.0)
    assert wire.dialect == "wheels"
    assert wire.encode(Wheels(75, 60)) == b"W75,60\n" and wire.sent == Wheels(75, 60)


def test_status_lines_are_read_whole():
    wire = Wire()
    wire.decode(b"S pc 7", now=1.0)
    assert wire.heard is None
    wire.decode(b"5 25 3\nS p", now=1.1)
    heard = wire.ack(1.1)
    assert (heard.who, heard.wheels, heard.rejected) == ("pc", Wheels(75, 25), 3)
    assert wire.describe(1.1) == "board pc 75/25"
    assert wire.ack(2.0) is None and wire.describe(2.0) == "board silent"


def test_signed_status_lines_are_understood():
    wire = Wire()
    wire.decode(b"S pc -60 -40 0\n", now=1.0)
    assert wire.ack(1.0).wheels == Wheels(-60, -40)
    assert wire.describe(1.0) == "board pc -60/-40"


def test_noise_on_the_line_is_not_a_status():
    wire = Wire()
    wire.decode(b"\xff\xfeS pc x y z\nhello\nS pc 1 2\n", now=1.0)
    assert wire.dialect is None and wire.heard is None


def test_musecar_firmware_is_recognised_and_given_digits():
    wire = Wire()
    wire.decode(b"SERIAL sig=0 cmd=0 ccr3=0 ccr4=0\r\nPWM pclk=16000000 psc=0\r\n", now=1.0)
    assert wire.dialect == "digits"
    assert wire.encode(STOP) == b"0"
    assert wire.encode(Wheels(75, 75)) == b"1" and wire.sent == Wheels(75, 75)
    assert wire.encode(Wheels(100, 100)) == b"2"
    assert wire.encode(Wheels(25, 75)) == b"3" and wire.sent == Wheels(25, 75)
    assert wire.encode(Wheels(75, 25)) == b"4"
    wire.decode(b"SERIAL sig=3 cmd=3 ccr3=7500 ccr4=22500\r\n", now=2.0)
    assert wire.ack(2.5).wheels == Wheels(25, 75)
    assert "musecar firmware" in wire.describe(2.5)


def test_a_turn_for_the_musecar_firmware_is_taken_late_and_dropped_early():
    wire = Wire()
    wire.decode(b"SERIAL sig=0 cmd=0\r\n", now=1.0)
    assert wire.encode(Wheels(55, 75)) == b"1"      # a slight turn: still straight
    assert wire.encode(Wheels(45, 75)) == b"3"      # past half way
    assert wire.encode(Wheels(55, 75)) == b"3"      # eases off: the turn is kept
    assert wire.encode(Wheels(65, 75)) == b"1"      # nearly straight: dropped
    assert wire.encode(Wheels(75, 45)) == b"4"
    assert wire.encode(STOP) == b"0"
    assert wire.encode(Wheels(75, 55)) == b"1"      # a stop forgets the turn


def test_button_only_firmware_is_called_out():
    wire = Wire()
    wire.decode(b"BUTTON raw=0 cmd=0 ccr3=0 ccr4=0\r\n", now=1.0)
    assert "ignores the PC" in wire.describe(1.2)


def test_no_car_accepts_everything():
    car = NoCar()
    assert car.send(Wheels(75, 75), 1.0) and car.sent == Wheels(75, 75)
    assert car.ack(1.0) is None and car.describe(1.0) == "no car"


def test_model_link_runs_on_the_callers_clock():
    car = ModelLink()
    for tick in range(1, 12):
        car.send(Wheels(60, 40), tick * 0.1)
    assert car.model.ms == 1000
    assert car.ack(1.1).wheels == Wheels(60, 40)
    for tick in range(12, 20):                      # the caller goes quiet
        car.model.run(100)
    assert car.model.duty == (0, 0)


def run_link(car, wheels, seconds):
    end = time.monotonic() + seconds
    reached = []
    while time.monotonic() < end:
        reached.append(car.send(wheels, time.monotonic()))
        time.sleep(0.05)
    return reached


def test_real_link_to_this_firmwares_protocol():
    pytest.importorskip("serial")
    from mind2motor.board import BoardModel
    server = TcpBoard(BoardModel())
    car = CarLink(server.url, baudrate=9600)
    try:
        assert all(run_link(car, Wheels(70, 30), 1.0))
        now = time.monotonic()
        assert car.dialect == "wheels"
        assert car.ack(now).wheels == Wheels(70, 30) and car.ack(now).who == "pc"
        assert car.describe(now) == "board pc 70/30"
    finally:
        car.close()
        server.stop = True


def test_real_link_to_the_musecar_firmwares_protocol():
    pytest.importorskip("serial")
    board = MusecarBoard()
    server = TcpBoard(board)
    car = CarLink(server.url)
    try:
        run_link(car, Wheels(75, 75), 0.8)
        assert car.dialect == "digits"
        assert car.ack(time.monotonic()).wheels == Wheels(75, 75)
        run_link(car, Wheels(25, 75), 0.4)
        assert car.ack(time.monotonic()).wheels == Wheels(25, 75)
        assert board.history[0] == 0 and 1 in board.history and 3 in board.history
    finally:
        car.close()
        server.stop = True


def test_real_link_notices_the_cable_and_starts_over():
    pytest.importorskip("serial")
    from mind2motor.board import BoardModel
    server = TcpBoard(BoardModel())
    car = CarLink(server.url)
    try:
        run_link(car, Wheels(50, 50), 0.6)
        assert car.dialect == "wheels"
        server.hang_up = True
        reached = run_link(car, Wheels(50, 50), 1.0)
        assert False in reached                     # the loss was reported
        reached = run_link(car, Wheels(50, 50), 2.5)
        assert reached[-1] and car.dialect == "wheels"
    finally:
        car.close()
        server.stop = True


def test_a_port_that_does_not_exist_is_reported_not_raised():
    pytest.importorskip("serial")
    car = CarLink("socket://127.0.0.1:9")           # nothing listens there
    assert car.send(STOP, 0.0) is False
    assert car.describe(0.0) == "CAR UNPLUGGED"
    assert car.send(STOP, 0.5) is False             # not retried within the second
    car.close()
