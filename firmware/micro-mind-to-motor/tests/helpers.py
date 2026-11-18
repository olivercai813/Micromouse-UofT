"""Shared by the tests."""

import random
import socket
import threading
import time

from mind2motor.board import ModelLink, Wire
from mind2motor.console import Silent
from mind2motor.runlog import RunLog
from mind2motor.senses import Senses
from mind2motor.session import Config, Session, Stage
from mind2motor.sources import SyntheticWearer

TICK = 0.1


def quick(**changes):
    """A short session: brief calibration, brief cues, no waiting for keys."""
    config = Config(armed=True, calibrate_s=8.0, trials=4, hold=(5.0, 11.0),
                    steer_hold=(4.0, 6.0))
    for name, value in changes.items():
        setattr(config, name, value)
    return config


class Rig:
    """A session with a made-up wearer on a made-up clock, advanced by hand
    so a test can press keys, pull electrodes or unplug the car midway."""

    def __init__(self, config=None, seed=0, car=None, wearer=None, **wearer_options):
        self.wearer = wearer or SyntheticWearer(seed=seed, **wearer_options)
        self.senses = Senses()
        self.ui = Silent(self.wearer.instruct)
        self.log = RunLog(keep=True)
        self.car = car or ModelLink()
        self.session = Session(self.senses, self.car, self.ui, self.log,
                               config or quick(), random.Random(seed))
        self.now = 0.0
        self.signal = True

    def tick(self, key=""):
        self.now = round(self.now + TICK, 6)
        if self.signal:
            self.wearer.advance(self.now, self.senses)
        self.session.tick(self.now, key)

    def run(self, seconds=None, until=None, limit=900.0):
        """Tick for `seconds`, or until the session reaches a stage."""
        end = self.now + (seconds if seconds is not None else limit)
        while self.now < end - 1e-9 and self.session.stage is not Stage.DONE:
            if until is not None and self.session.stage is until:
                return True
            self.tick()
        return until is None or self.session.stage is until

    @property
    def rows(self):
        return self.log.rows

    def said(self, text):
        return any(text in note for note in self.ui.notes)


class MusecarBoard:
    """The serial behaviour of the firmware in github.com/Libritor/musecar,
    as its README states it: single digits 0 to 4, anything else ignored,
    idle after 500 ms without a digit, and a status line on every change
    and once a second ("SERIAL sig=N cmd=N ..." while digits arrive,
    "BUTTON raw=0 cmd=0 ..." otherwise)."""

    DUTY = {0: (0, 0), 1: (22500, 22500), 2: (30000, 30000), 3: (7500, 22500),
            4: (22500, 7500)}

    def __init__(self):
        self.ms = 0
        self.command = 0
        self.history = []
        self._live = False
        self._heard = 0
        self._reported = (None, None)
        self._report_at = 0
        self._out = bytearray()

    def feed(self, data):
        for byte in data:
            if 48 <= byte <= 52:
                self.command, self._live, self._heard = byte - 48, True, self.ms

    def run(self, ms):
        for _ in range(ms):
            self.ms += 1
            if self._live and self.ms - self._heard > 500:
                self._live, self.command = False, 0
            if (self._live, self.command) != self._reported or self.ms - self._report_at >= 1000:
                self._reported, self._report_at = (self._live, self.command), self.ms
                self.history.append(self.command)
                left, right = self.DUTY[self.command]
                head = f"SERIAL sig={self.command}" if self._live else "BUTTON raw=0"
                self._out += (f"{head} cmd={self.command} ccr3={left} ccr4={right}\r\n"
                              f"PWM pclk=16000000 psc=0 arr=29999 ccr3={left} ccr4={right}\r\n"
                              ).encode()

    def take_output(self):
        data, self._out = bytes(self._out), bytearray()
        return data


class TcpBoard(threading.Thread):
    """A board model behind a TCP port, in real time, for links opened with
    a pyserial URL (socket://127.0.0.1:<port>)."""

    def __init__(self, model):
        super().__init__(daemon=True)
        self.model = model
        self._server = socket.socket()
        self._server.bind(("127.0.0.1", 0))
        self._server.listen(1)
        self.port = self._server.getsockname()[1]
        self.url = f"socket://127.0.0.1:{self.port}"
        self.hang_up = False        # close the connection, as if unplugged
        self.stop = False
        self.start()

    def run(self):
        self._server.settimeout(0.05)
        while not self.stop:
            try:
                peer, _ = self._server.accept()
            except socket.timeout:
                continue
            peer.setblocking(False)
            began, lived = time.monotonic(), 0
            while not self.stop and not self.hang_up:
                try:
                    data = peer.recv(4096)
                    if not data:
                        break
                    self.model.feed(data)
                except BlockingIOError:
                    pass
                except OSError:
                    break
                due = int((time.monotonic() - began) * 1000) - lived
                if due > 0:
                    self.model.run(due)
                    lived += due
                out = self.model.take_output()
                if out:
                    try:
                        peer.sendall(out)
                    except OSError:
                        break
                time.sleep(0.002)
            peer.close()
            self.hang_up = False
        self._server.close()
