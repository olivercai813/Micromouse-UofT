"""Where the headband data comes from.

  MuseBle          the headband itself, over this computer's Bluetooth
  OscHeadband      a phone app (Mind Monitor, MuseLog) streaming OSC
  SyntheticWearer  made-up raw EEG and head tilt that follow spoken cues,
                   for rehearsals and for the bench controls

Each one delivers raw samples into a Senses object; none of them computes
anything about the signal.
"""

from __future__ import annotations

import asyncio
import math
import struct
import sys

import numpy as np

# ---------------------------------------------------------------- Bluetooth

MUSE_UUID = "273e{:04x}-4c4d-454d-96be-f03bac821358".format
CONTROL = MUSE_UUID(0x0001)
EEG = tuple(MUSE_UUID(n) for n in (0x0003, 0x0004, 0x0005, 0x0006))  # TP9 AF7 AF8 TP10
MOTION = MUSE_UUID(0x000A)
UV_PER_COUNT = 1682.0 / 4096.0      # 12-bit samples on libmuse's 0-1682 uV scale
G_PER_COUNT = 1.0 / 16384.0         # 16-bit accelerometer, 2 g full scale
EEG_RATE = 256.0
PRESETS = ("p21", "p20", "p50", "p51", "p1034", "p1035")


def eeg_packet(data):
    """(sequence number, 12 samples in microvolts) of one EEG notification:
    a 16-bit counter followed by twelve 12-bit samples, packed."""
    data = bytes(data)
    bits = np.unpackbits(np.frombuffer(data[2:20], dtype=np.uint8)).reshape(12, 12)
    counts = bits @ (1 << np.arange(11, -1, -1))
    return int.from_bytes(data[:2], "big"), counts * UV_PER_COUNT


def motion_packet(data):
    """Gravity in g from one accelerometer notification: a 16-bit counter,
    then three samples of x, y, z; their mean is returned."""
    samples = np.frombuffer(bytes(data)[2:20], dtype=">i2").reshape(3, 3)
    return samples.mean(axis=0) * G_PER_COUNT


def control_frame(text):
    """A command for the headband's control characteristic."""
    body = text.encode("ascii") + b"\n"
    return bytes([len(body)]) + body


class RowBuilder:
    """Lines up the four electrodes, which arrive as separate packets
    carrying the same sequence number, into rows of four."""

    def __init__(self, keep=8):
        self.keep = keep
        self._open = {}         # sequence number -> {electrode: samples}
        self.dropped = 0

    def add(self, electrode, sequence, samples):
        """A [12, 4] block once all four electrodes of a packet are in."""
        packet = self._open.setdefault(sequence, {})
        packet[electrode] = samples
        if len(packet) < 4:
            return None
        del self._open[sequence]
        for old in [s for s in self._open if (sequence - s) % 65536 > self.keep]:
            del self._open[old]         # a packet that will never complete
            self.dropped += 1
        return np.stack([packet[e] for e in range(4)], axis=1)


class MuseBle:
    """The headband over Bluetooth Low Energy, as an asyncio task. It must
    be switched on and not connected to a phone."""

    def __init__(self, senses, note, clock, name="", preset=None):
        self.senses, self.note, self.clock = senses, note, clock
        self.name = name.lower()
        self.presets = (preset,) if preset else PRESETS
        self.address = None
        self.motion_packets = 0
        self._unpaired = False

    async def run(self):
        from bleak import BleakClient, BleakScanner
        self.note("Looking for the headband over Bluetooth.")
        while True:
            found = await BleakScanner.discover(timeout=5.0, return_adv=True)
            muses = [(advert.rssi, device) for device, advert in found.values()
                     if "muse" in (device.name or "").lower()
                     and self.name in (device.name or "").lower()]
            if not muses:
                await asyncio.sleep(1.0)
                continue
            device = max(muses, key=lambda pair: pair[0])[1]     # the nearest
            lost = asyncio.Event()
            began = self.clock()
            try:
                async with BleakClient(device, disconnected_callback=lambda _: lost.set()) as link:
                    await self._serve(link, lost, device.name)
                self.note(f"{device.name}: connection lost; looking again.")
            except Exception as error:      # bleak raises many kinds
                self.note(f"{device.name}: {type(error).__name__}: {error}; trying again.")
            self.address = None
            if (not self._unpaired and sys.platform == "win32"
                    and self.clock() - began < 8.0):
                # A headband that Windows has paired drops every connection
                # within a second; removing the pairing once cures it.
                self._unpaired = True
                try:
                    await BleakClient(device).unpair()
                    self.note("Removed the headband's old Windows pairing.")
                except Exception:
                    pass
            await asyncio.sleep(1.0)

    async def _serve(self, link, lost, name):
        rows = RowBuilder()
        flowing = asyncio.Event()

        def on_eeg(electrode):
            def take(_sender, data):
                if len(data) < 20:
                    return
                sequence, samples = eeg_packet(data)
                block = rows.add(electrode, sequence, samples)
                if block is not None:
                    self.senses.on_eeg(self.clock(), block, EEG_RATE)
                    flowing.set()
            return take

        def on_motion(_sender, data):
            if len(data) >= 20:
                self.senses.on_gravity(self.clock(), motion_packet(data))
                self.motion_packets += 1

        for electrode, uuid in enumerate(EEG):
            await link.start_notify(uuid, on_eeg(electrode))
        try:
            await link.start_notify(MOTION, on_motion)
        except Exception as error:
            self.note(f"No accelerometer on this headband ({error}): no steering.")

        async def tell(text):
            await link.write_gatt_char(CONTROL, control_frame(text), response=False)

        # Headbands differ in which preset delivers what: keep the first
        # one that gives EEG and the accelerometer, else the first with EEG.
        chosen = eeg_only = None
        for preset in self.presets:
            flowing.clear()
            before = self.motion_packets
            for text in ("h", preset, "d"):
                await tell(text)
            try:
                await asyncio.wait_for(flowing.wait(), timeout=5.0)
            except asyncio.TimeoutError:
                continue
            await asyncio.sleep(2.0)
            if self.motion_packets > before:
                chosen = preset
                break
            eeg_only = eeg_only or preset
        if chosen is None:
            if eeg_only is None:
                self.note(f"{name} connected but sent no EEG with any preset.")
                return
            chosen = eeg_only
            for text in ("h", chosen, "d"):
                await tell(text)
            self.note("The headband sends no accelerometer data: no steering.")
        self.address = link.address
        self.note(f"{name} connected (preset {chosen}).")
        while not lost.is_set():
            await tell("k")             # the headband stops without these
            try:
                await asyncio.wait_for(lost.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                pass


# ---------------------------------------------------------------------- OSC

def _osc_text(data, at):
    end = data.index(b"\0", at)
    return data[at:end].decode("ascii", "replace"), (end + 4) & ~3


def osc_messages(data):
    """(address, numbers) of every message in an OSC packet. Bundles are
    opened; arguments that are not numbers are skipped."""
    if data.startswith(b"#bundle\0"):
        at = 16
        while at + 4 <= len(data):
            size = int.from_bytes(data[at:at + 4], "big")
            yield from osc_messages(data[at + 4:at + 4 + size])
            at += 4 + size
        return
    try:
        address, at = _osc_text(data, 0)
        tags, at = _osc_text(data, at)
        if not address.startswith("/") or not tags.startswith(","):
            return
        numbers = []
        for tag in tags[1:]:
            if tag in "fi":
                numbers.append(struct.unpack_from(">" + tag, data, at)[0])
                at += 4
            elif tag in "dh":
                numbers.append(struct.unpack_from(">" + ("d" if tag == "d" else "q"), data, at)[0])
                at += 8
            elif tag in "sS":
                _, at = _osc_text(data, at)
            elif tag not in "TFNI":
                break
    except (ValueError, struct.error):
        return
    yield address, numbers


class OscHeadband(asyncio.DatagramProtocol):
    """Raw EEG and the accelerometer from a phone app's OSC stream:
    /muse/eeg (or /eeg) with one value per electrode, /muse/acc in g."""

    RATES = (256.0, 220.0)      # what the headbands produce

    def __init__(self, senses, note, clock, port=5000):
        self.senses, self.note, self.clock, self.port = senses, note, clock, port
        self.rate = None
        self._pending = []
        self._since = None
        self._complained = False

    async def run(self):
        loop = asyncio.get_running_loop()
        await loop.create_datagram_endpoint(lambda: self, local_addr=("0.0.0.0", self.port))
        self.note(f"Listening for the phone app on UDP port {self.port}.")
        await asyncio.Event().wait()

    def error_received(self, error):    # Windows reports stray ICMP errors here
        pass

    def datagram_received(self, data, _address):
        now = self.clock()
        rows = []
        for address, numbers in osc_messages(data):
            if address in ("/muse/eeg", "/eeg") and len(numbers) >= 4:
                rows.append(numbers[:4])
            elif address == "/muse/acc" and len(numbers) >= 3:
                self.senses.on_gravity(now, numbers[:3])
        if rows:
            self._take(now, np.array(rows, float))

    def _take(self, now, block):
        if self.rate is not None:
            self.senses.on_eeg(now, block, self.rate)
            return
        # The stream does not say how fast it is: count for three seconds.
        if self._since is None:
            self._since = now
        self._pending.append(block)
        if now - self._since < 3.0:
            return
        measured = sum(len(b) for b in self._pending) / (now - self._since)
        nearest = min(self.RATES, key=lambda rate: abs(rate - measured))
        if abs(nearest - measured) <= 0.12 * nearest:
            self.rate = nearest
            self.senses.trouble = ""
            self.senses.on_eeg(now, np.concatenate(self._pending)[-int(nearest):], nearest)
        else:
            self.senses.trouble = (f"raw EEG is arriving at {measured:.0f} Hz; the "
                                   "app must stream at the headband's full rate")
            if not self._complained:
                self._complained = True
                self.note("The phone app sends " + self.senses.trouble + ".")
        self._pending, self._since = [], None


# ---------------------------------------------------------------- synthetic

class SyntheticWearer:
    """Raw EEG and head tilt of a wearer who does not exist.

    Each electrode gets a 1/f background, mains hum, and narrow-band
    rhythms (theta, alpha, beta, gamma) built from sums of sinusoids. The
    rhythms' strengths depend on the state of mind: relaxed with eyes
    closed has strong alpha, focused has less alpha and more beta, and
    blinks on the forehead electrodes. `contrast` scales how different the
    two states are (0 = identical). The signal passes through the same
    filters and decoder as a real one.

    behaviour   follows   does what each cue says, after a reaction time
                inverts   does the opposite
                ignores   pays no attention and changes state on its own
    Calibration cues are always followed, whatever the behaviour.
    """

    RATE = 256.0
    TILT_DEGREES = 22.0

    def __init__(self, seed=0, behaviour="follows", contrast=1.0):
        if behaviour not in ("follows", "inverts", "ignores"):
            raise ValueError(behaviour)
        self.behaviour = behaviour
        rng = self.rng = np.random.default_rng(seed)
        self._parts = []                # (hz [k], phase [k, 4], relaxed [4], focused [4])

        def rhythm(low, high, count, relaxed, focused):
            relaxed, focused = np.array(relaxed, float), np.array(focused, float)
            focused = relaxed + contrast * (focused - relaxed)
            scale = math.sqrt(2.0 / count)      # the numbers given are RMS microvolts
            self._parts.append((rng.uniform(low, high, count),
                                rng.uniform(0.0, 2.0 * math.pi, (count, 4)),
                                relaxed * scale, focused * scale))

        rhythm(4.5, 7.5, 10, [4.0, 5.0, 5.0, 4.0], [4.5, 6.0, 6.0, 4.5])
        rhythm(8.5, 12.0, 14, [14.0, 7.0, 7.0, 14.0], [5.0, 3.5, 3.5, 5.0])
        rhythm(14.0, 28.0, 24, [3.0, 3.0, 3.0, 3.0], [4.5, 5.0, 5.0, 4.5])
        rhythm(31.0, 44.0, 24, [1.5, 1.5, 1.5, 1.5], [2.2, 2.6, 2.6, 2.2])
        hz = np.geomspace(0.6, 70.0, 80)
        weight = hz ** -0.5
        weight *= 9.0 * math.sqrt(2.0) / math.sqrt((weight ** 2).sum())
        self._background = (hz, rng.uniform(0.0, 2.0 * math.pi, (80, 4)), weight)
        self._hum = rng.uniform(2.0, 8.0, 4)
        self._blink_size = contrast * np.array([25.0, 110.0, 110.0, 25.0])
        self._blinks = []
        self.dropouts = []              # (electrode, from, until) on the caller's
                                        # clock: held at the rail meanwhile
        # The headband sits on the head at an arbitrary angle.
        self._frame = np.linalg.qr(rng.normal(size=(3, 3)))[0]
        self._zero = None               # the caller's clock at the first sample
        self._count = 0                 # EEG samples made so far
        self._mind = self._mind_goal = 0.0      # 0 relaxed .. 1 focused
        self._lean = self._lean_goal = 0.0      # +1 left, -1 right
        self._due = []                  # (time, what, value)
        self._wander_mind = self._wander_lean = None

    def instruct(self, cue, now, calibrating=False):
        """A cue was spoken: relax, focus, left, right, centre or upright."""
        if not calibrating and self.behaviour == "ignores":
            if self._wander_mind is None:
                self._wander_mind = now + self.rng.uniform(1.0, 8.0)
                self._wander_lean = now + self.rng.uniform(1.0, 6.0)
            return
        opposite = not calibrating and self.behaviour == "inverts"
        when = now + self.rng.uniform(0.6, 1.4)
        if cue in ("relax", "focus"):
            self._due.append((when, "mind", float((cue == "focus") != opposite)))
        else:
            lean = {"left": 1.0, "right": -1.0}.get(cue, 0.0)
            self._due.append((when, "lean", -lean if opposite else lean))

    def advance(self, now, senses):
        """Deliver everything the headband would have sent up to `now`."""
        for item in [d for d in self._due if d[0] <= now]:
            self._due.remove(item)
            if item[1] == "mind":
                self._mind_goal = item[2]
            else:
                self._lean_goal = item[2]
        if self._wander_mind is not None:
            if now >= self._wander_mind:
                self._mind_goal = 1.0 - self._mind_goal
                self._wander_mind = now + self.rng.uniform(6.0, 14.0)
            if now >= self._wander_lean:
                self._lean_goal = float(self.rng.choice([-1.0, 0.0, 1.0]))
                self._wander_lean = now + self.rng.uniform(4.0, 9.0)
        if self._zero is None:
            self._zero = now            # the headband is switched on now
        want = int((now - self._zero) * self.RATE) - self._count
        if want <= 0:
            return
        t = (self._count + np.arange(want)) / self.RATE     # since switch-on
        span = want / self.RATE
        self._mind += (1.0 - math.exp(-span / 0.4)) * (self._mind_goal - self._mind)
        self._lean += (1.0 - math.exp(-span / 0.25)) * (self._lean_goal - self._lean)

        hz, phase, weight = self._background
        signal = (np.cos(2.0 * math.pi * t[:, None, None] * hz[None, :, None] + phase)
                  * weight[None, :, None]).sum(axis=1)
        for hz, phase, relaxed, focused in self._parts:
            size = relaxed + self._mind * (focused - relaxed)
            signal += (np.cos(2.0 * math.pi * t[:, None, None] * hz[None, :, None] + phase)
                       ).sum(axis=1) * size
        signal += np.cos(2.0 * math.pi * 60.0 * t)[:, None] * self._hum
        if self._mind > 0.5 and self.rng.random() < 0.25 * span:    # eyes open: blinks
            self._blinks.append(t[0])
        self._blinks = [start for start in self._blinks if t[0] - start < 0.3]
        for start in self._blinks:
            into = (t - start) / 0.25
            bump = np.where((into >= 0) & (into <= 1), 0.5 - 0.5 * np.cos(2.0 * math.pi * into), 0.0)
            signal += bump[:, None] * self._blink_size
        signal += 840.0
        for electrode, start, until in self.dropouts:
            clock = self._zero + t
            signal[(clock >= start) & (clock < until), electrode] = 0.0
        self._count += want
        senses.on_eeg(now, signal, self.RATE)

        angle = math.radians(self.TILT_DEGREES) * self._lean
        gravity = np.array([0.03, math.sin(angle), math.cos(angle)])
        gravity = self._frame @ (gravity + self.rng.normal(0.0, 0.008, 3))
        senses.on_gravity(now, gravity)
