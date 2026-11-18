"""Command line: python -m mind2motor <command>

  drive    a session with a headband and the car
  bench    the same session against a made-up wearer and a model board,
           faster than real time
  score    the report of a recorded cued test
  ports    the serial ports of this computer
  wheels   turn the wheels directly, to check the car without a headband
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import time

from . import __version__, score
from .bench import TICK_S, rehearse
from .board import ST_VENDOR, STOP, Wheels, open_car
from .console import Console
from .runlog import RunLog
from .senses import Senses
from .session import Config, Session, Stage
from .sources import MuseBle, OscHeadband, SyntheticWearer


def _session_options(parser):
    parser.add_argument("--trials", type=int, default=0, metavar="N",
                        help="cued test: N focus and N relax cues, then a report")
    parser.add_argument("--steer-trials", type=int, default=0, metavar="N",
                        help="add N cues each of tilt left, tilt right and head straight")
    parser.add_argument("--rounds", type=int, default=2,
                        help="calibration rounds, each one relaxed and one focused "
                             "recording (default 2; with 1 the calibration cannot "
                             "tell a drifting signal from a difference between states)")
    parser.add_argument("--calibrate", type=float, default=15.0, metavar="SECONDS",
                        help="length of each calibration recording (default 15)")
    parser.add_argument("--go", type=float, default=0.65,
                        help="level at which the car starts; 0 = relaxed, 1 = focused")
    parser.add_argument("--stop", type=float, default=0.35,
                        help="level at which the car stops")
    parser.add_argument("--speed", type=int, default=75, metavar="PERCENT",
                        help="wheel duty when going straight (default 75)")
    parser.add_argument("--no-steer", action="store_true",
                        help="no head-tilt steering and no steering calibration")
    parser.add_argument("--seed", type=int, default=None,
                        help="makes the order and length of the cues repeatable")
    parser.add_argument("--wearer", choices=("follows", "inverts", "ignores"),
                        default="follows", help="how the made-up wearer treats the cues")
    parser.add_argument("--contrast", type=float, default=1.0,
                        help="how different the made-up wearer's two states are (0 = identical)")
    parser.add_argument("--out", metavar="FOLDER", help="where the run is written")


def _config(args, **forced):
    if not args.stop < args.go:
        sys.exit("--stop must be below --go")
    if not 0 < args.speed <= 100:
        sys.exit("--speed is a percentage, 1 to 100")
    if args.rounds < 1:
        sys.exit("--rounds must be at least 1")
    config = Config(rounds=args.rounds, calibrate_s=args.calibrate, go=args.go,
                    stop=args.stop, speed=args.speed, steer=not args.no_steer,
                    trials=args.trials, steer_trials=args.steer_trials)
    if config.calibrate_s < config.settle_s + 4:
        sys.exit(f"--calibrate must be at least {config.settle_s + 4:.0f} s")
    for name, value in forced.items():
        setattr(config, name, value)
    return config


def _stay_awake():
    """Nobody touches the keyboard during a run; an idle computer that goes
    to sleep stops the frames and with them the car."""
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)  # continuous | system required
    elif sys.platform == "darwin":
        subprocess.Popen(["caffeinate", "-i", "-w", str(os.getpid())])


async def _live(args):
    config = _config(args, armed=args.armed)
    clock = time.monotonic
    folder = Path(args.out) if args.out else RunLog.new_folder()
    log = RunLog(folder, record=args.record, keep=True)
    senses = Senses(mains=args.mains, recorder=log.raw if args.record else None)
    ui = Console(speak=not args.quiet)
    car = open_car(args.car, args.baud)
    wearer, jobs = None, []
    if args.headband == "synthetic":
        wearer = SyntheticWearer(seed=args.seed or 0, behaviour=args.wearer,
                                 contrast=args.contrast)
        ui.cue = wearer.instruct
        config.armed = True
        config.trials = config.trials or 3
        ui.note("Made-up wearer: calibrates, then follows a cued test. Q quits.")
    elif args.headband == "ble":
        source = MuseBle(senses, ui.note, clock, args.name, args.preset)
        jobs.append(asyncio.create_task(source.run()))
    else:
        source = OscHeadband(senses, ui.note, clock, args.osc_port)
        jobs.append(asyncio.create_task(source.run()))
    session = Session(senses, car, ui, log, config, random.Random(args.seed))
    log.meta(version=__version__, started=time.strftime("%Y-%m-%d %H:%M:%S"),
             headband=args.headband, car=args.car, config=vars(config))
    ui.note(f"Writing this run to {folder}")
    _stay_awake()
    due = clock()
    try:
        while session.stage is not Stage.DONE:
            now = clock()
            if wearer is not None:
                wearer.advance(now, senses)
            session.tick(now, ui.key())
            for job in jobs:
                if job.done() and job.exception():
                    raise job.exception()
            due += TICK_S
            if clock() - due > 1.0:         # fell behind: do not rush to catch up
                due = clock()
            await asyncio.sleep(max(0.0, due - clock()))
    except KeyboardInterrupt:
        pass
    finally:
        car.close()
        ui.close()
        log.close()
        for job in jobs:
            job.cancel()
    print(f"Stopped ({session.ended or 'interrupted'}).")
    if session.ended == "test":
        result = score.report(folder)
        print(score.describe(result))
        print(f"Report: {folder / 'report.json'}")


def _drive(args):
    if args.headband == "ble":
        try:
            import bleak  # noqa: F401
        except ImportError:
            sys.exit("Bluetooth needs the bleak package: pip install bleak")
    if args.car not in ("none", "model"):
        try:
            import serial  # noqa: F401
        except ImportError:
            sys.exit("The car needs the pyserial package: pip install pyserial")
    asyncio.run(_live(args))


def _bench(args):
    results = []
    for repeat in range(args.repeats):
        seed = (args.seed or 0) + repeat
        config = _config(args, armed=True)
        config.trials = config.trials or 10
        folder = Path(args.out) / f"seed{seed}" if args.out else None
        wearer = SyntheticWearer(seed=seed, behaviour=args.wearer, contrast=args.contrast)
        started = time.perf_counter()
        session, log, console = rehearse(config, wearer, seed, RunLog(folder, keep=True))
        log.close()
        if session.ended != "test":
            print(f"seed {seed}: ended with '{session.ended}'")
            for line in console.notes[-4:]:
                print("   ", line)
            continue
        result = score.analyse(log.rows)
        if folder:
            (folder / "report.json").write_text(json.dumps(result, indent=2))
        results.append(result)
        drive = result["drive"]
        print(f"seed {seed}: steady accuracy {100 * drive['steady_accuracy']:.0f}%, "
              f"reached {drive['reached']}/{drive['cues']}, "
              f"p {drive['chance']['p_value']}, "
              f"calibration held-out accuracy "
              f"{100 * session.calibration.held_out_accuracy:.0f}%   "
              f"[{session.now:.0f} s of session in {time.perf_counter() - started:.1f} s]")
    if len(results) == 1:
        print(score.describe(results[0]))
    elif results:
        accuracy = [r["drive"]["steady_accuracy"] for r in results]
        small = sum(1 for r in results
                    if r["drive"]["chance"]["p_value"] is not None
                    and r["drive"]["chance"]["p_value"] < 0.05)
        print(f"{len(results)} runs, wearer {args.wearer}: steady accuracy mean "
              f"{100 * sum(accuracy) / len(accuracy):.0f}%, range "
              f"{100 * min(accuracy):.0f}-{100 * max(accuracy):.0f}%; "
              f"p < 0.05 in {small} of {len(results)}.")
    return 0 if results else 1


def _score(args):
    result = score.report(args.path)
    if not result["cues_scored"]:
        print("No cued test in this log (run drive with --trials N).")
        return 1
    print(score.describe(result))
    return 0


def _ports(_args):
    from serial.tools import list_ports
    found = list(list_ports.comports())
    for port in found:
        mark = "   <- the board's ST-LINK" if port.vid == ST_VENDOR else ""
        print(f"{port.device:12s} {port.description}{mark}")
    if not found:
        print("No serial ports.")
    return 0


def _wheels(args):
    if not (-100 <= args.left <= 100 and -100 <= args.right <= 100):
        sys.exit("duties are signed percentages, -100 to 100")
    car = open_car(args.car, args.baud)
    wanted = Wheels(args.left, args.right)
    began = time.monotonic()
    driving_at = None
    stopped_at = None
    shown = ""
    try:
        while True:
            now = time.monotonic()
            if driving_at is None:
                command = STOP
            elif now - driving_at < args.seconds:
                command = wanted
            elif stopped_at is None:
                stopped_at = now
                command = STOP
            elif now - stopped_at < 0.5:
                command = STOP
            else:
                break

            if not car.send(command, now) and now - began > 3.0:
                sys.exit("The board cannot be reached: " + car.describe(now))
            if driving_at is None and (car.dialect is not None or args.car == "none"):
                # The first requested wheel frame is sent on the next 100 ms tick.
                # Start timing there, not while Bluetooth is still connecting.
                driving_at = time.monotonic() + TICK_S
            elif driving_at is None and now - began > 10.0:
                sys.exit("The board did not identify itself: " + car.describe(now))
            text = car.describe(now)
            if text != shown:
                shown = text
                print(f"{now - began:5.1f} s  sent {car.sent.left}/{car.sent.right}  {text}")
            time.sleep(TICK_S)
    finally:
        car.close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="mind2motor", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)

    drive = commands.add_parser("drive", help="a session with a headband and the car")
    drive.add_argument("--headband", choices=("ble", "osc", "synthetic"), default="ble",
                       help="ble: the headband over this computer's Bluetooth (default); "
                            "osc: a phone app streaming OSC; synthetic: a made-up wearer")
    drive.add_argument("--name", default="",
                       help="part of the headband's Bluetooth name, e.g. 7AC1")
    drive.add_argument("--preset", help="headband preset, instead of trying them in turn")
    drive.add_argument("--osc-port", type=int, default=5000)
    drive.add_argument("--car", default="auto",
                       help="auto (the board's ST-LINK), a serial port, model (a "
                            "simulated board) or none")
    drive.add_argument("--baud", type=int, default=9600,
                       help="HC-05 serial baud rate (default 9600)")
    drive.add_argument("--mains", type=float, default=60.0,
                       help="mains frequency to filter out: 60 or 50")
    drive.add_argument("--armed", action="store_true",
                       help="never wait for SPACE: calibrate at once, then let the car move")
    drive.add_argument("--quiet", action="store_true", help="do not speak the prompts")
    drive.add_argument("--record", action="store_true",
                       help="also keep the raw EEG (eeg.f32 in the run folder)")
    _session_options(drive)
    drive.set_defaults(run=_drive)

    bench = commands.add_parser("bench", help="a made-up wearer and a model board, "
                                              "faster than real time")
    bench.add_argument("--repeats", type=int, default=1,
                       help="runs, each with the next seed")
    _session_options(bench)
    bench.set_defaults(run=_bench)

    scoring = commands.add_parser("score", help="the report of a recorded cued test")
    scoring.add_argument("path", help="a run folder or its ticks.csv")
    scoring.set_defaults(run=_score)

    ports = commands.add_parser("ports", help="the serial ports of this computer")
    ports.set_defaults(run=_ports)

    wheels = commands.add_parser("wheels", help="turn the wheels directly")
    wheels.add_argument("left", type=int, help="left wheel duty, signed percent")
    wheels.add_argument("right", type=int, help="right wheel duty, signed percent")
    wheels.add_argument("--seconds", type=float, default=3.0)
    wheels.add_argument("--car", default="auto")
    wheels.add_argument("--baud", type=int, default=9600,
                        help="HC-05 serial baud rate (default 9600)")
    wheels.set_defaults(run=_wheels)

    args = parser.parse_args(argv)
    return args.run(args) or 0


if __name__ == "__main__":
    sys.exit(main())
