"""Command line.

    python -m questpath demo                 everything simulated, in the terminal
    python -m questpath demo --page          simulated wearer and mouse, real page
    python -m questpath run --mouse COM7     the real thing
    python -m questpath check                is the EEG stream arriving?
    python -m questpath mouse COM7 square    drive the mouse without the headband
    python -m questpath ports                serial ports on this computer
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

from .clock import RealClock
from .detect import DetectConfig
from .grid import Forward, Lane, Turn, plan_step
from .mouse import Mouse, MouseConfig, MouseError, SerialTransport, SimMouse
from .session import Session, SessionConfig


def _lane(a) -> Lane:
    return Lane(rows=a.rows, cols=3, cell_mm=a.cell_mm, start_col={"L": 0, "M": 1, "R": 2}[a.start])


def _config(a) -> SessionConfig:
    detect = DetectConfig(threshold=a.threshold, margin=a.margin, max_rounds=a.max_rounds)
    return SessionConfig(lane=_lane(a), detect=detect, flash_s=a.flash, gap_s=a.gap, style=a.style,
                         drive=a.drive, calib_blocks=0 if a.no_check else a.check_blocks,
                         max_idle=a.max_idle, wait_start=not a.auto, seed=a.seed)


def _session_options(p) -> None:
    p.add_argument("--rows", type=int, default=6, help="rows in the lane, counting the start row (default 6)")
    p.add_argument("--cell-mm", type=float, default=180.0, help="size of a square on the floor (default 180)")
    p.add_argument("--start", choices="LMR", default="M", help="square of the first row the mouse starts on")
    p.add_argument("--flash", type=float, default=3.0, help="seconds each square flickers (default 3)")
    p.add_argument("--gap", type=float, default=0.5, help="steady seconds between flashes")
    p.add_argument("--style", choices=("square", "diagonal"), default="square",
                   help="square: straight runs and quarter turns (default); diagonal: straight to the square")
    p.add_argument("--drive", choices=("each", "end"), default="each",
                   help="each: the mouse moves after every row (default); end: trace the whole path first")
    p.add_argument("--threshold", type=float, default=3.0, help="how far a square must stand out from noise")
    p.add_argument("--margin", type=float, default=1.5, help="...and from the runner-up")
    p.add_argument("--max-rounds", type=int, default=4, help="rounds of flashes before a row counts as 'nothing'")
    p.add_argument("--max-idle", type=int, default=5,
                   help="give up after nothing was picked this many times running (default 5)")
    p.add_argument("--check-blocks", type=int, default=4, help="flicker / no-flicker pairs in the signal check")
    p.add_argument("--no-check", action="store_true", help="skip the signal check")
    p.add_argument("--auto", action="store_true", help="do not wait for the Start button")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--speed", type=int, default=150, help="mouse speed on straight runs, mm/s")
    p.add_argument("--no-trim", action="store_true", help="do not correct runs from the encoder distance")


def _report(summary: dict, session: Session) -> None:
    print()
    print(session.board())
    print()
    print("outcome: %s" % summary["outcome"])
    names = "LMR"
    print("path:    start %s" % names[summary["path"][0][1]]
          + "".join(" -> %s" % names[c[1]] for c in summary["path"][1:]))


def _log_path(a) -> str:
    folder = a.log or os.path.join("runs", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(folder, exist_ok=True)
    return os.path.join(folder, "session.jsonl")


# ------------------------------------------------------------------------ demo

def cmd_demo(a) -> int:
    from .rehearsal import Rehearsal, parse_path
    gaze = parse_path(a.path)
    if not a.page:
        rows = max(a.rows, max(gaze) + 1)
        a.rows = rows
        r = Rehearsal(gaze, _config(a), seed=a.seed or 0, say=print,
                      wearer={"look_uv": a.response}, mouse_config=MouseConfig(speed_mms=a.speed, trim=not a.no_trim))
        print("Simulated session: the wearer looks at %s, row by row.\n" % a.path)
        summary = r.run()
        _report(summary, r.session)
        print("mouse:   %.0f mm forward, %.0f mm left of the start (its square's centre is at %.0f, %.0f)"
              % (summary["mouse_mm"][0], summary["mouse_mm"][1], summary["target_mm"][0], summary["target_mm"][1]))
        print("time:    %.0f s of session, simulated" % summary["seconds"])
        return 0 if summary["outcome"] == "finished" else 1

    # the real page, a simulated wearer and mouse, in real time
    from .display import WebDisplay
    from .eeg import EEGBuffer
    from .server import PageServer
    from .wearer import SimWearer
    from .osc import local_addresses
    clock = RealClock()
    a.rows = max(a.rows, max(gaze) + 1)
    server = PageServer(clock, a.port).start()
    display = WebDisplay(server, clock)
    eeg = EEGBuffer()
    wearer = SimWearer(display, eeg, gaze, seed=a.seed or 0, look_uv=a.response)
    mouse = Mouse(SimMouse(clock), clock, MouseConfig(speed_mms=a.speed, trim=not a.no_trim))
    print("Open this page in a browser (the Quest's, or this computer's):")
    for ip in ["localhost"] + local_addresses():
        print("    http://%s:%d/" % (ip, server.port))
    session = Session(clock, display, eeg, mouse, _config(a), pump=wearer.advance, say=print)
    try:
        summary = session.run()
    except KeyboardInterrupt:
        print("\nstopped")
        return 1
    finally:
        server.close()
    _report(summary, session)
    return 0 if summary["outcome"] == "finished" else 1


# ------------------------------------------------------------------------- run

def cmd_run(a) -> int:
    from .display import WebDisplay
    from .eeg import EEGBuffer
    from .osc import OscReceiver, local_addresses
    from .server import PageServer
    clock = RealClock()
    eeg = EEGBuffer()
    try:
        receiver = OscReceiver(a.eeg_port, eeg.push, clock).start()
    except OSError as exc:
        print(exc)
        return 2
    server = PageServer(clock, a.port).start()
    display = WebDisplay(server, clock)
    ips = local_addresses()
    print("1. Phone: in MuseLog's OSC settings set the target IP to %s, port %d, full rate on, and start streaming."
          % (" or ".join(ips) or "this computer's address", a.eeg_port))
    print("2. Headset: open  http://%s:%d/  in the Quest browser." % (ips[0] if ips else "<this computer>", server.port))
    transport = None
    mouse = None
    try:
        if a.mouse.lower() == "sim":
            print("3. Mouse: SIMULATED (no board will move).")
            mouse = Mouse(SimMouse(clock), clock, MouseConfig(speed_mms=a.speed, trim=not a.no_trim))
        elif a.mouse.lower() != "none":
            transport = SerialTransport(a.mouse, a.baud)
            mouse = Mouse(transport, clock, MouseConfig(speed_mms=a.speed, trim=not a.no_trim),
                          log=(print if a.verbose else None))
            mouse.connect()
            print("3. Mouse: answering on %s. Put it on the start square, facing up the lane, wheels on the floor."
                  % a.mouse)
        else:
            print("3. Mouse: none (the path is only traced).")

        print("\nwaiting for the EEG stream...")
        deadline = clock.now() + a.wait
        while True:
            rate, rms = eeg.quality()
            if rate > 30 and eeg.rate > 0:
                break
            if clock.now() > deadline:
                print("No EEG arrived on UDP port %d in %d s. Same Wi-Fi? Target IP right? Streaming started?"
                      % (a.eeg_port, a.wait))
                return 2
            clock.sleep(0.5)
        print("EEG: %.0f samples/s from %s; electrodes %s; rms %s uV"
              % (rate, ", ".join(sorted(receiver.senders)) or "?", ", ".join(eeg.names()),
                 " ".join("%.0f" % v for v in rms)))
        if eeg.rate < 200:
            print("  (the app is sending a reduced rate; turn 'full rate' on in MuseLog for a cleaner signal)")
        if len(eeg.names()) < 5:
            print("  (no AUX electrode in the stream: the flicker shows best at the back of the head)")

        log = _log_path(a)
        print("log: %s\n" % log)
        session = Session(clock, display, eeg, mouse, _config(a), say=print, log_path=log)
        summary = session.run()
        with open(os.path.join(os.path.dirname(log), "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=1)
        _report(summary, session)
        return 0 if summary["outcome"] == "finished" else 1
    except MouseError as exc:
        print("mouse: %s" % exc)
        return 2
    except KeyboardInterrupt:
        print("\nstopped from the keyboard")
        return 1
    finally:
        if mouse is not None:
            mouse.stop()
        if transport is not None:
            transport.close()
        receiver.close()
        server.close()


# ----------------------------------------------------------------------- check

def cmd_check(a) -> int:
    from .eeg import EEGBuffer
    from .osc import OscReceiver, local_addresses
    clock = RealClock()
    eeg = EEGBuffer()
    try:
        receiver = OscReceiver(a.eeg_port, eeg.push, clock).start()
    except OSError as exc:
        print(exc)
        return 2
    print("listening on UDP %d for %d s; this computer is %s" % (a.eeg_port, a.seconds,
                                                                  " or ".join(local_addresses()) or "?"))
    try:
        end = clock.now() + a.seconds
        while clock.now() < end:
            clock.sleep(1.0)
            rate, rms = eeg.quality()
            print("  %5.0f samples/s   rms uV: %s   (other messages: %d)"
                  % (rate, " ".join("%6.1f" % v for v in rms) or "-", receiver.other))
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
    if eeg.total == 0:
        print("nothing arrived.")
        return 1
    print("electrodes: %s; sample rate taken as %.0f Hz" % (", ".join(eeg.names()), eeg.rate))
    return 0


# ----------------------------------------------------------------------- mouse

def cmd_mouse(a) -> int:
    clock = RealClock()
    lane = Lane(cell_mm=a.cell_mm)
    transport = SimMouse(clock) if a.port.lower() == "sim" else None
    try:
        if transport is None:
            transport = SerialTransport(a.port, a.baud)
        mouse = Mouse(transport, clock, MouseConfig(speed_mms=a.speed, trim=not a.no_trim), log=print)
        mouse.connect()
        if a.what == "pose":
            print(mouse.pose())
            return 0
        if a.what == "stop":
            mouse.stop()
            return 0
        mouse.zero()
        if a.what == "forward":
            moves = [Forward(lane.cell_mm * a.amount)]
        elif a.what == "turn":
            moves = [Turn(a.amount)]
        elif a.what == "left":
            moves = plan_step(lane, (0, 1), (1, 0), a.style)
        elif a.what == "right":
            moves = plan_step(lane, (0, 1), (1, 2), a.style)
        else:                                   # square: out, across, back, and home
            side = lane.cell_mm * a.amount
            moves = [Forward(side), Turn(90), Forward(side), Turn(180), Forward(side), Turn(270),
                     Forward(side), Turn(0)]
        print("moves: %s" % moves)
        mouse.execute(moves)
        print("pose now: %s" % mouse.pose())
        return 0
    except MouseError as exc:
        print("mouse: %s" % exc)
        return 2
    except KeyboardInterrupt:
        print("\nstopped")
        return 1
    finally:
        if transport is not None:
            try:
                transport.write("STOP")
            except Exception:
                pass
            transport.close()


def cmd_ports(a) -> int:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial is not installed: pip install pyserial")
        return 2
    ports = list(list_ports.comports())
    if not ports:
        print("no serial ports")
    for port in ports:
        print("%-8s %s" % (port.device, port.description))
    print("\nThe mouse is the HC-05's OUTGOING Bluetooth port (Windows: Bluetooth settings > More > COM Ports).")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="questpath", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("demo", help="a simulated session (no hardware)")
    _session_options(p)
    p.add_argument("--path", default="M,L,L,R,M", help="where the simulated wearer looks, row by row: "
                   "L, M, R or - for looking away (default M,L,L,R,M)")
    p.add_argument("--response", type=float, default=3.0, help="simulated response, microvolts (default 3)")
    p.add_argument("--page", action="store_true", help="serve the real page and run in real time")
    p.add_argument("--port", type=int, default=8090)
    p.set_defaults(func=cmd_demo)

    p = sub.add_parser("run", help="headset page + MuseLog EEG + the mouse")
    _session_options(p)
    p.add_argument("--mouse", required=True, help="the mouse's COM port (e.g. COM7), 'sim' or 'none'")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--eeg-port", type=int, default=5000, help="UDP port MuseLog streams to (default 5000)")
    p.add_argument("--port", type=int, default=8090, help="port of the headset page (default 8090)")
    p.add_argument("--wait", type=int, default=120, help="seconds to wait for the EEG stream")
    p.add_argument("--log", default="", help="folder for the session log (default runs/<time>)")
    p.add_argument("--verbose", action="store_true", help="print every line to and from the mouse")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("check", help="show whether the EEG stream arrives")
    p.add_argument("--eeg-port", type=int, default=5000)
    p.add_argument("--seconds", type=int, default=15)
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("mouse", help="drive the mouse by hand through the same code")
    p.add_argument("port", help="COM port, or 'sim'")
    p.add_argument("what", choices=("forward", "turn", "left", "right", "square", "pose", "stop"))
    p.add_argument("amount", type=float, nargs="?", default=1.0,
                   help="squares for forward/square, degrees for turn (default 1)")
    p.add_argument("--cell-mm", type=float, default=180.0)
    p.add_argument("--speed", type=int, default=150)
    p.add_argument("--style", choices=("square", "diagonal"), default="square")
    p.add_argument("--no-trim", action="store_true")
    p.add_argument("--baud", type=int, default=9600)
    p.set_defaults(func=cmd_mouse)

    p = sub.add_parser("ports", help="list serial ports")
    p.set_defaults(func=cmd_ports)

    a = parser.parse_args(argv)
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
