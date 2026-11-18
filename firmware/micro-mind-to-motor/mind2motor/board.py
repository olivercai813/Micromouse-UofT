"""The car's board: what is said to it, what it answers, and a model of it.

Wire format of this repository's firmware, normally 9600 baud over the HC-05,
8N1, text lines ending in a newline:

  to the board     W<left>,<right>   signed wheel duties in percent, -100 to 100,
                                     positive forward and negative backward,
                                     repeated at least every 400 ms
                   ?                 ask for the identification line
  from the board   S <who> <left> <right> <rejected>
                                     ten times a second: who is driving
                                     (pc, btn or idle), the duties now
                                     applied, lines rejected so far
                   I mind2motor <n>  identification, at start and on "?"

The board stops the wheels by itself when the frames stop coming. Raising a
duty is spread over up to 200 ms; lowering it is immediate.

The link also speaks to the firmware of the earlier car program
(github.com/Libritor/musecar), which takes single digits 0 to 4 and reports
"SERIAL ... cmd=N". Which firmware is on the board is read from its own
status lines; until one has been seen only the stop frame is sent, which
both firmwares read as stop.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import NamedTuple

ST_VENDOR = 0x0483              # USB vendor id of the board's ST-LINK
FULL = 32000                    # timer counts at 100 % duty
PER_PERCENT = FULL // 100
RISE_PER_MS = FULL // 200       # a duty may rise this many counts per ms
LINK_TIMEOUT_MS = 400
STATUS_EVERY_MS = 100
BUTTON_PERCENT = 75
BUTTON_MS = 20
LINE_ROOM = 12                  # longest line the board keeps
IDENT = b"I mind2motor 1\n"
PINS_FORWARD = 0b0110           # PB1 and PB2 high, PB0 and PB3 low
PINS_REVERSE = 0b1001           # PB0 and PB3 high, PB1 and PB2 low


class Wheels(NamedTuple):
    """Duty of the left and the right wheel, percent."""
    left: int
    right: int


STOP = Wheels(0, 0)
# What the digits of the musecar firmware do.
DIGIT_WHEELS = {0: STOP, 1: Wheels(75, 75), 2: Wheels(100, 100),
                3: Wheels(25, 75), 4: Wheels(75, 25)}


def frame(wheels):
    """The line that asks this repository's firmware for these duties."""
    return b"W%d,%d\n" % (wheels.left, wheels.right)


class BoardModel:
    """The firmware's behaviour, one millisecond at a time. The session is
    rehearsed against it, and the tests hold the compiled firmware to the
    same expectations."""

    def __init__(self):
        self.ms = 0
        self.button = False
        self.duty = (0, 0)          # timer counts, left and right
        self.pins = 0               # PB3..PB0
        self.rejected = 0
        self._line = bytearray()
        self._spoiled = False
        self._pc_live = False
        self._pc_heard = 0
        self._pc_duty = (0, 0)
        self._press = 0
        self._down = False
        self._out = bytearray(IDENT)

    def feed(self, data, damaged=False):
        """Bytes arriving from the PC; `damaged` marks a receive error."""
        for byte in data:
            if damaged:
                self._spoiled = True
            if byte == 0x0A:
                if self._spoiled:
                    self.rejected += 1
                elif self._line:
                    self._accept(bytes(self._line))
                self._line.clear()
                self._spoiled = False
            elif byte == 0x0D:
                pass
            elif len(self._line) < LINE_ROOM:
                self._line.append(byte)
            else:
                self._spoiled = True

    def _accept(self, line):
        wanted = re.fullmatch(rb"W(-?\d{1,3}),(-?\d{1,3})", line)
        if wanted and abs(int(wanted[1])) <= 100 and abs(int(wanted[2])) <= 100:
            self._pc_duty = (int(wanted[1]) * PER_PERCENT, int(wanted[2]) * PER_PERCENT)
            self._pc_heard = self.ms
            self._pc_live = True
        elif line == b"?":
            self._out += IDENT
        else:
            self.rejected += 1

    def run(self, ms):
        for _ in range(ms):
            self.ms += 1
            if self._pc_live and self.ms - self._pc_heard > LINK_TIMEOUT_MS:
                self._pc_live = False
            if self.button:
                self._press = min(self._press + 1, BUTTON_MS)
                self._down = self._down or self._press == BUTTON_MS
            else:
                self._press = max(self._press - 1, 0)
                self._down = self._down and self._press > 0
            if self._pc_live:
                want = self._pc_duty
            elif self._down:
                want = (BUTTON_PERCENT * PER_PERCENT,) * 2
            else:
                want = (0, 0)
            self.duty = tuple(min(have + RISE_PER_MS, goal) if have < goal else goal
                              for have, goal in zip(self.duty, want))
            self.pins = ((0b0010 if self.duty[0] > 0 else 0b0001 if self.duty[0] < 0 else 0)
                         | (0b0100 if self.duty[1] > 0 else 0b1000 if self.duty[1] < 0 else 0))
            if self.ms % STATUS_EVERY_MS == 0:
                who = "pc" if self._pc_live else "btn" if self._down else "idle"
                self._out += b"S %s %d %d %d\n" % (
                    who.encode(), self.duty[0] // PER_PERCENT,
                    self.duty[1] // PER_PERCENT, self.rejected)

    def take_output(self):
        data, self._out = bytes(self._out), bytearray()
        return data


@dataclass
class Heard:
    """The newest status line of the board."""
    at: float
    wheels: Wheels
    who: str            # pc, btn or idle
    rejected: int = 0


class Wire:
    """Encoding and decoding shared by every kind of link."""

    def __init__(self):
        self.forget()

    def forget(self):
        """Nothing is known about the board, e.g. after it was replugged."""
        self.dialect = None        # "wheels", "digits" or None
        self.heard = None
        self.sent = STOP           # what the last frame really asked for
        self._text = b""
        self._bear = 0

    def encode(self, wheels):
        if self.dialect == "wheels":
            self.sent = Wheels(*wheels)
            return frame(self.sent)
        if self.dialect == "digits":
            digit = self._digit(wheels)
            self.sent = DIGIT_WHEELS[digit]
            return b"%d" % digit
        self.sent = STOP
        return frame(STOP)

    def _digit(self, wheels):
        """The nearest of the musecar firmware's five commands. A turn is
        taken once the inner wheel is under two thirds of the outer one and
        dropped once it is back above four fifths."""
        if min(wheels) < 0:
            self._bear = 0
            return 0
        outer, inner = max(wheels), min(wheels)
        if outer == 0:
            self._bear = 0
            return 0
        if inner < 0.67 * outer:
            self._bear = 3 if wheels.left < wheels.right else 4
        elif inner > 0.8 * outer:
            self._bear = 0
        return self._bear or (2 if outer >= 90 else 1)

    def decode(self, data, now):
        self._text += data
        *lines, self._text = self._text.split(b"\n")
        for line in lines:
            text = line.strip().decode("ascii", "replace")
            ours = re.fullmatch(r"S (pc|btn|idle) (-?\d+) (-?\d+) (\d+)", text)
            theirs = re.match(r"(SERIAL|BUTTON)\b.*?\bcmd=(\d)", text)
            if ours:
                self.dialect = "wheels"
                self.heard = Heard(now, Wheels(int(ours[2]), int(ours[3])),
                                   ours[1], int(ours[4]))
            elif text.startswith("I mind2motor"):
                self.dialect = "wheels"
            elif theirs:
                self.dialect = "digits"
                self.heard = Heard(now, DIGIT_WHEELS.get(int(theirs[2]), STOP),
                                   "pc" if theirs[1] == "SERIAL" else "btn")

    def ack(self, now):
        """The board's newest status, or None when it is older than the
        board's own reporting interval allows."""
        fresh = 0.5 if self.dialect == "wheels" else 1.5
        if self.heard is None or now - self.heard.at > fresh:
            return None
        return self.heard

    def describe(self, now):
        heard = self.ack(now)
        if heard is None:
            return "board silent"
        text = f"board {heard.who} {heard.wheels.left}/{heard.wheels.right}"
        if self.dialect == "digits":
            if heard.who == "btn":
                return "board ignores the PC (button-only firmware?)"
            text += " (musecar firmware)"
        return text


class NoCar(Wire):
    """Stands in when there is no car: everything is accepted."""

    def send(self, wheels, now):
        self.sent = Wheels(*wheels)
        return True

    def ack(self, now):
        return None

    def describe(self, now):
        return "no car"

    def close(self):
        pass


class ModelLink(Wire):
    """A BoardModel in place of the board, on whatever clock the caller
    uses, so a session can be rehearsed faster than real time."""

    def __init__(self, model=None):
        super().__init__()
        self.model = model or BoardModel()
        self._zero = None

    def send(self, wheels, now):
        if self._zero is None:
            self._zero = now
        due = int(round((now - self._zero) * 1000.0)) - self.model.ms
        if due > 0:
            self.model.run(due)
        self.model.feed(self.encode(wheels))
        self.decode(self.model.take_output(), now)
        return True

    def close(self):
        pass


class CarLink(Wire):
    """The real board on a serial port: "auto" for whichever port is an
    ST-LINK, or a port name or pyserial URL. ``baudrate`` is 115200 for the
    original USB/ST-LINK firmware and 9600 for the Micromouse HC-05. The
    port is retried once a second while the link is unavailable."""

    def __init__(self, port="auto", baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = int(baudrate)
        self.device = None
        self._serial = None
        self._retry_at = 0.0

    def _open(self):
        import serial
        from serial.tools import list_ports
        device = self.port
        if device == "auto":
            found = [p.device for p in list_ports.comports() if p.vid == ST_VENDOR]
            if not found:
                raise serial.SerialException("no ST-LINK on USB")
            device = found[0]
        self._serial = serial.serial_for_url(device, baudrate=self.baudrate, timeout=0,
                                             write_timeout=0.3)
        self.device = device
        self.forget()

    def send(self, wheels, now):
        """Send one frame and take in what the board printed. False while
        the board cannot be reached."""
        import serial
        try:
            if self._serial is None:
                if now < self._retry_at:
                    return False
                self._retry_at = now + 1.0
                self._open()
            self._serial.write(self.encode(wheels))
            self.decode(self._serial.read(4096), now)
        except (serial.SerialException, OSError):
            self._drop()
            return False
        return True

    def _drop(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self.forget()

    def describe(self, now):
        if self._serial is None:
            return "CAR UNPLUGGED"
        if self.dialect is None:
            return "board not identified yet"
        return super().describe(now)

    def close(self):
        if self._serial is not None:
            try:
                self._serial.write(self.encode(STOP))
            except Exception:
                pass
            self._drop()


def open_car(spec, baudrate=9600):
    """A link from the --car option: none, model, auto, or a serial port."""
    if spec == "none":
        return NoCar()
    if spec == "model":
        return ModelLink()
    return CarLink(spec, baudrate)
