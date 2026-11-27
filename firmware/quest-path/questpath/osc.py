"""Just enough OSC to read a phone app's EEG stream, with no dependency.

MuseLog (and Mind Monitor) send one UDP message per sample:

    /muse/eeg  f f f f [f ...]     TP9, AF7, AF8, TP10, then AUX if present

Messages may come singly or wrapped in bundles.
"""

from __future__ import annotations

import socket
import struct
import threading


def _padded(n: int) -> int:
    return (n + 4) & ~3


def _string(data: bytes, at: int):
    end = data.index(b"\x00", at)
    return data[at:end].decode("utf-8", errors="replace"), at + _padded(end - at)


def parse(data: bytes):
    """All (address, [arguments]) in one datagram. Unreadable data gives []."""
    try:
        if data.startswith(b"#bundle\x00"):
            out, at = [], 16
            while at + 4 <= len(data):
                size = struct.unpack_from(">i", data, at)[0]
                at += 4
                if size <= 0 or at + size > len(data):
                    break
                out.extend(parse(data[at:at + size]))
                at += size
            return out
        address, at = _string(data, 0)
        if not address.startswith("/") or at >= len(data) or data[at:at + 1] != b",":
            return []
        tags, at = _string(data, at)
        args = []
        for tag in tags[1:]:
            if tag == "f":
                args.append(struct.unpack_from(">f", data, at)[0])
                at += 4
            elif tag == "i":
                args.append(struct.unpack_from(">i", data, at)[0])
                at += 4
            elif tag == "d":
                args.append(struct.unpack_from(">d", data, at)[0])
                at += 8
            elif tag == "h":
                args.append(struct.unpack_from(">q", data, at)[0])
                at += 8
            elif tag == "s":
                text, at = _string(data, at)
                args.append(text)
            elif tag in "TF":
                args.append(tag == "T")
            elif tag in "NI":
                args.append(None)
            else:
                return []
        return [(address, args)]
    except (ValueError, struct.error, IndexError):
        return []


def message(address: str, values) -> bytes:
    """An OSC message of floats (what the phone app sends)."""
    def pad(raw: bytes) -> bytes:
        return raw + b"\x00" * (_padded(len(raw)) - len(raw))
    values = list(values)
    return (pad(address.encode()) + pad(("," + "f" * len(values)).encode())
            + b"".join(struct.pack(">f", float(v)) for v in values))


EEG_ADDRESSES = ("/muse/eeg", "/eeg")


def is_eeg(address: str) -> bool:
    """Raw EEG, not the band-power or contact messages that share the port."""
    return address in EEG_ADDRESSES or (address.endswith("/eeg") and "/elements" not in address
                                        and not address.startswith("/person"))


class OscReceiver:
    """Listens on a UDP port and hands every raw EEG sample to `on_sample`
    (arrival time, list of microvolts)."""

    def __init__(self, port: int, on_sample, clock):
        self.port = port
        self.on_sample = on_sample
        self.clock = clock
        self.packets = 0
        self.other = 0
        self.senders = set()
        self._stop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._sock.bind(("0.0.0.0", port))
        except OSError as exc:
            self._sock.close()
            raise OSError("UDP port %d is in use (another EEG program running?): %s" % (port, exc)) from exc
        self._sock.settimeout(0.25)
        self._thread = threading.Thread(target=self._loop, name="questpath-osc", daemon=True)

    def start(self):
        self._thread.start()
        return self

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, sender = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            now = self.clock.now()
            self.senders.add(sender[0])
            for address, args in parse(data):
                values = [a for a in args if isinstance(a, (int, float)) and not isinstance(a, bool)]
                if is_eeg(address) and len(values) >= 4:
                    self.packets += 1
                    self.on_sample(now, values[:8])
                else:
                    self.other += 1

    def close(self) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def local_addresses():
    """This computer's IPv4 addresses, for the phone app's "target IP": the
    one the network traffic leaves by first, then any others."""
    main = None
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("10.255.255.255", 1))        # nothing is sent
        main = probe.getsockname()[0]
        probe.close()
    except OSError:
        pass
    found = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            found.add(info[4][0])
    except socket.gaierror:
        pass
    others = sorted(ip for ip in found if not ip.startswith("127.") and ip != main)
    return ([main] if main and not main.startswith("127.") else []) + others
