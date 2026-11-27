"""Driving the Micromouse through the `micromouseFirmware` Bluetooth protocol.

The firmware (firmware/micromouseFirmware) takes newline-terminated commands
over the HC-05 at 9600 baud. This module uses the closed-loop ones:

    HELLO                    -> HELLO MICROMOUSE_HUB 2 ...
    ZERO_ENC                 -> ACK ZERO_ENC          (distance and heading to 0)
    POSE                     -> POSE HEAD h RATE r BIAS b DIST d VEL v ENCHEAD e IMU i
    TURN <deg> [ms]          -> ACK TURN, then EVENT TURN_DONE or EVENT MOTOR_TIMEOUT
    V <mm/s> <deg/s> <ms>    -> ACK V, then EVENT MOTOR_TIMEOUT when the time is up
    STOP                     -> ACK STOP
    TELEM <ms>               -> ACK TELEM             (telemetry lines start "T,")

Every moving command carries its own time limit, enforced by the firmware, so
the mouse stops by itself if this program or the link dies.

`SimMouse` stands in for the board in tests and rehearsals: it speaks the same
lines and keeps track of where a mouse would really be.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .grid import Forward, Turn, wrap180


class MouseError(RuntimeError):
    pass


@dataclass
class MouseConfig:
    speed_mms: int = 150          # straight-line speed asked of the firmware
    trim: bool = True             # correct a straight run from the POSE distance
    trim_mm: float = 12.0         # ...when it is off by more than this
    trim_speed_mms: int = 80
    heading_tol_deg: float = 3.0  # do not bother turning for less than this
    turn_tries: int = 3
    reply_s: float = 2.0          # how long to wait for an ACK
    max_run_ms: int = 9000        # the firmware refuses more than 10 s


# ------------------------------------------------------------------ transports

class SerialTransport:
    """A COM port (the HC-05's outgoing Bluetooth port)."""

    def __init__(self, port: str, baud: int = 9600):
        try:
            import serial
        except ImportError as exc:           # pragma: no cover - depends on the machine
            raise MouseError("pyserial is not installed: pip install pyserial") from exc
        try:
            self.port = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=1.0)
        except Exception as exc:
            raise MouseError("cannot open %s: %s" % (port, exc)) from exc
        self.buffer = bytearray()

    def write(self, text: str) -> None:
        self.port.write((text + "\n").encode("ascii"))

    def lines(self):
        chunk = self.port.read(512)
        if chunk:
            self.buffer.extend(chunk)
        out = []
        while b"\n" in self.buffer:
            raw, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            out.append(raw.decode("ascii", errors="replace").strip())
        return out

    def close(self) -> None:
        try:
            self.port.close()
        except Exception:
            pass


class SimMouse:
    """A board that is not there: the firmware's replies and a mouse's motion.

    speed_gain      the real speed as a fraction of the speed asked for
    turn_overshoot  fraction by which every turn overshoots (0.07 = 7 %)
    Faults for tests: `silent` (never answers), `stuck_turns` (turns never
    finish, as with the wheels in the air).
    """

    TURN_RATE = 180.0

    def __init__(self, clock, speed_gain=1.0, turn_overshoot=0.0, telemetry_s=0.0,
                 silent=False, stuck_turns=False):
        self.clock = clock
        self.speed_gain = speed_gain
        self.turn_overshoot = turn_overshoot
        self.telemetry_s = telemetry_s
        self.silent = silent
        self.stuck_turns = stuck_turns
        self.x = self.y = 0.0            # forward and left of the start, mm
        self.heading = 0.0               # true heading, degrees, + = left
        self.dist = 0.0
        self.mode = "idle"
        self.speed = self.rate = 0.0
        self.turn_left = 0.0
        self.until = 0.0
        self.settle = None
        self.out = []
        self.sent = []                   # every command received, for tests
        self.t = clock.now()
        self.next_telemetry = self.t + telemetry_s if telemetry_s else None
        self.closed = False

    # -- the model
    def _advance(self) -> None:
        now = self.clock.now()
        while self.t < now - 1e-9:
            dt = min(0.002, now - self.t)
            self.t += dt
            if self.mode == "V":
                v = self.speed * self.speed_gain
                self.heading += self.rate * dt
                self.x += v * dt * math.cos(math.radians(self.heading))
                self.y += v * dt * math.sin(math.radians(self.heading))
                self.dist += v * dt
            elif self.mode == "TURN" and not self.stuck_turns:
                step = self.TURN_RATE * dt
                if abs(self.turn_left) > step:
                    move = math.copysign(step, self.turn_left)
                    self.heading += move
                    self.turn_left -= move
                elif self.settle is None:
                    self.heading += self.turn_left
                    self.turn_left = 0.0
                    self.settle = self.t + 0.05
                elif self.t >= self.settle:
                    self.mode = "idle"
                    self.settle = None
                    self.out.append("EVENT TURN_DONE")
            if self.mode != "idle" and self.t >= self.until:
                self.mode = "idle"
                self.settle = None
                self.out.append("EVENT MOTOR_TIMEOUT")
            if self.next_telemetry is not None and self.t >= self.next_telemetry:
                self.next_telemetry += self.telemetry_s
                self.out.append("T," + ",".join(["0"] * 22))

    # -- the transport face
    def write(self, text: str) -> None:
        self._advance()
        line = text.strip()
        self.sent.append(line)
        if self.silent:
            return
        parts = line.split()
        try:
            if line in ("HELLO", "?"):
                self.out.append("HELLO MICROMOUSE_HUB 2 MAX_PWM=71.0")
            elif line == "STOP":
                self.mode = "idle"
                self.settle = None
                self.out.append("ACK STOP")
            elif line == "ZERO_ENC":
                self.dist = 0.0
                self.heading = 0.0
                self.x = self.y = 0.0
                self.out.append("ACK ZERO_ENC")
            elif line == "POSE":
                self.out.append("POSE HEAD %.1f RATE 0.0 BIAS 0.0 DIST %.1f VEL 0.0 ENCHEAD %.1f IMU 1"
                                % (self.heading, self.dist, self.heading))
            elif parts[0] == "TELEM" and len(parts) == 2:
                self.telemetry_s = max(0.2, min(2.0, int(parts[1]) / 1000.0))
                self.next_telemetry = self.t + self.telemetry_s
                self.out.append("ACK TELEM")
            elif parts[0] == "V" and len(parts) == 4:
                self.speed, self.rate = float(int(parts[1])), float(int(parts[2]))
                ms = min(int(parts[3]), 10000)
                if ms <= 0:
                    self.mode = "idle"
                else:
                    self.mode = "V"
                    self.until = self.t + ms / 1000.0
                self.out.append("ACK V")
            elif parts[0] == "TURN" and len(parts) in (2, 3):
                ms = int(parts[2]) if len(parts) == 3 else 3000
                if ms <= 0 or ms > 10000:
                    ms = 3000
                self.turn_left = int(parts[1]) * (1.0 + self.turn_overshoot)
                self.mode = "TURN"
                self.settle = None
                self.until = self.t + ms / 1000.0
                self.out.append("ACK TURN")
            else:
                self.out.append("ERR UNKNOWN_COMMAND")
        except (ValueError, IndexError):
            self.out.append("ERR UNKNOWN_COMMAND")

    def lines(self):
        self._advance()
        out, self.out = self.out, []
        return out

    def close(self) -> None:
        self.closed = True


# ---------------------------------------------------------------------- driver

class Mouse:
    """Turns and straight runs on the real (or simulated) board."""

    def __init__(self, transport, clock, config: MouseConfig | None = None, log=None, watch=None):
        self.transport = transport
        self.clock = clock
        self.config = config or MouseConfig()
        self.log = log or (lambda text: None)
        self.watch = watch               # called while waiting; may raise to abandon a move
        self.connected = False

    # -- line plumbing
    def _send(self, command: str) -> None:
        self.log("-> " + command)
        self.transport.write(command)

    def _drain(self) -> None:
        for line in self.transport.lines():
            if line and not line.startswith("T,"):
                self.log("<- " + line + "  (late)")

    def _wait(self, wanted, seconds: float, what: str, errors=True):
        """The first line for which `wanted(line)` is true, within `seconds`."""
        deadline = self.clock.now() + seconds
        while True:
            for line in self.transport.lines():
                if not line or line.startswith("T,"):
                    continue
                self.log("<- " + line)
                if wanted(line):
                    return line
                if errors and line.startswith("ERR"):
                    raise MouseError("the mouse refused (%s) while waiting for %s" % (line, what))
            if self.clock.now() >= deadline:
                raise MouseError("no answer from the mouse: waited %.1f s for %s" % (seconds, what))
            if self.watch is not None:
                self.watch()
            self.clock.sleep(0.01)

    # -- commands
    def connect(self) -> None:
        last = None
        for _ in range(3):
            self._drain()
            self._send("HELLO")
            try:
                self._wait(lambda s: s.startswith("HELLO MICROMOUSE_HUB"), self.config.reply_s, "HELLO")
                break
            except MouseError as exc:
                last = exc
        else:
            raise MouseError("the mouse does not answer HELLO: is micromouseFirmware flashed, the "
                             "battery on, and this the HC-05's outgoing COM port? (%s)" % last)
        # fewer telemetry lines leave the slow link free for our commands
        self._send("TELEM 2000")
        self._wait(lambda s: s.startswith("ACK TELEM"), self.config.reply_s, "ACK TELEM")
        self.connected = True

    def zero(self) -> None:
        """Call the mouse's present spot and heading the origin."""
        self._drain()
        self._send("ZERO_ENC")
        self._wait(lambda s: s.startswith("ACK ZERO_ENC"), self.config.reply_s, "ACK ZERO_ENC")

    def pose(self) -> dict:
        self._drain()
        self._send("POSE")
        line = self._wait(lambda s: s.startswith("POSE "), self.config.reply_s, "POSE")
        words = line.split()[1:]
        try:
            return {words[i].lower(): float(words[i + 1]) for i in range(0, len(words) - 1, 2)}
        except ValueError as exc:
            raise MouseError("cannot read the mouse's POSE line: %r" % line) from exc

    def stop(self) -> None:
        """Stop now. Never raises: this is what runs when something went wrong."""
        watch, self.watch = self.watch, None
        try:
            self._send("STOP")
            self._wait(lambda s: s.startswith("ACK STOP"), 1.0, "ACK STOP", errors=False)
        except Exception:
            pass
        finally:
            self.watch = watch

    def turn_to(self, heading: float) -> float:
        """Face `heading` (degrees from the start heading, + = left).
        Returns the heading error left over."""
        error = 0.0
        for attempt in range(self.config.turn_tries):
            error = wrap180(heading - self.pose()["head"])
            if abs(error) <= self.config.heading_tol_deg:
                return error
            degrees = int(round(error))
            limit_ms = int(min(self.config.max_run_ms, 1500 + abs(degrees) / 180.0 * 1000 * 2))
            self._send("TURN %d %d" % (degrees, limit_ms))
            self._wait(lambda s: s.startswith("ACK TURN"), self.config.reply_s, "ACK TURN")
            self._wait(lambda s: s.startswith("EVENT TURN_DONE") or s.startswith("EVENT MOTOR_TIMEOUT"),
                       limit_ms / 1000.0 + 2.0, "the end of the turn")
        error = wrap180(heading - self.pose()["head"])
        if abs(error) > 3 * self.config.heading_tol_deg:
            raise MouseError("the mouse did not finish its turn (%.0f degrees short): are the wheels "
                             "on the floor and the gyro calibrated?" % error)
        return error

    def _run(self, speed: int, mm: float) -> None:
        left = abs(mm)
        direction = 1 if mm >= 0 else -1
        while left > 0.5:
            ms = int(round(left / speed * 1000.0))
            ms = max(1, min(ms, self.config.max_run_ms))
            self._send("V %d 0 %d" % (direction * speed, ms))
            self._wait(lambda s: s.startswith("ACK V"), self.config.reply_s, "ACK V")
            self._wait(lambda s: s.startswith("EVENT MOTOR_TIMEOUT"), ms / 1000.0 + 2.0, "the end of the run")
            left -= speed * ms / 1000.0

    def forward(self, mm: float) -> float:
        """Drive straight `mm`. Returns the distance the encoders say was
        covered."""
        start = self.pose()["dist"]
        self._run(self.config.speed_mms, mm)
        self.clock.sleep(0.15)                       # let it coast to rest
        covered = self.pose()["dist"] - start
        short = mm - covered
        if self.config.trim and self.config.trim_mm < abs(short) < 0.6 * abs(mm):
            self._run(self.config.trim_speed_mms, short)
            self.clock.sleep(0.15)
            covered = self.pose()["dist"] - start
        return covered

    def execute(self, moves) -> None:
        for move in moves:
            if isinstance(move, Turn):
                self.turn_to(move.heading)
            elif isinstance(move, Forward):
                self.forward(move.mm)
            else:
                raise MouseError("unknown move %r" % (move,))

    def close(self) -> None:
        if self.connected:
            self.stop()
        self.transport.close()
