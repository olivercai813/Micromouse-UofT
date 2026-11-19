from __future__ import annotations

import queue
import threading
from dataclasses import dataclass

import serial
from serial.tools import list_ports


@dataclass(frozen=True)
class LinkEvent:
    kind: str
    value: str


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


class SerialLink:
    def __init__(self, events: queue.Queue[LinkEvent]) -> None:
        self.events = events
        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self, port: str, baud: int = 9600) -> None:
        self.disconnect(send_stop=False)
        serial_port = serial.Serial(port, baudrate=baud, timeout=0.15, write_timeout=0.5)
        stop_event = threading.Event()
        self._serial = serial_port
        self._stop = stop_event
        self._thread = threading.Thread(
            target=self._reader,
            args=(serial_port, stop_event),
            name="micromouse-serial",
            daemon=True,
        )
        self._thread.start()
        self.events.put(LinkEvent("connected", port))

    def send(self, command: str) -> None:
        if not self.connected:
            raise RuntimeError("Bluetooth serial port is not connected")
        payload = (command.rstrip("\r\n") + "\n").encode("ascii")
        with self._write_lock:
            assert self._serial is not None
            self._serial.write(payload)

    def disconnect(self, send_stop: bool = True) -> None:
        if send_stop and self.connected:
            try:
                self.send("STOP")
                assert self._serial is not None
                self._serial.flush()
            except (OSError, serial.SerialException, RuntimeError):
                pass
        stop_event = self._stop
        serial_port = self._serial
        reader_thread = self._thread
        stop_event.set()
        device = serial_port.port if serial_port is not None else ""
        if serial_port is not None:
            try:
                serial_port.close()
            except serial.SerialException:
                pass
        self._serial = None
        self._thread = None
        if reader_thread is not None and reader_thread is not threading.current_thread():
            reader_thread.join(timeout=0.5)
        if device:
            self.events.put(LinkEvent("disconnected", str(device)))

    def _reader(self, serial_port: serial.Serial, stop_event: threading.Event) -> None:
        buffer = bytearray()
        unexpected_error = ""
        while not stop_event.is_set() and serial_port.is_open:
            try:
                chunk = serial_port.read(128)
                if not chunk:
                    continue
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    self.events.put(LinkEvent("line", raw.decode("ascii", errors="replace").rstrip("\r")))
            except (OSError, serial.SerialException) as exc:
                unexpected_error = str(exc)
                break
        if not stop_event.is_set() and self._serial is serial_port:
            if unexpected_error:
                self.events.put(LinkEvent("error", unexpected_error))
            try:
                serial_port.close()
            except serial.SerialException:
                pass
            self._serial = None
            self._thread = None
            self.events.put(LinkEvent("disconnected", str(serial_port.port)))
