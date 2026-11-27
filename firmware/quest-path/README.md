# quest-path: trace a path with your eyes, the Micromouse drives it

The wearer of a Meta Quest 3S sees a lane of rows, three squares to a row,
with the mouse on its start square. The three squares of the row ahead of the
mouse flicker **one at a time**. The wearer looks at the square the mouse
should go to. The laptop remembers which square was flickering when, looks for
that flicker in the EEG a Muse headband streams through MuseLog (a steady-state
visual evoked potential, SSVEP), and sends the mouse to the square that
answered:

| square that answers | the path goes | the mouse (default style) |
|---|---|---|
| left | to the left square of the next row | turns left, one square, turns back, one square forward |
| middle | straight on | one square forward |
| right | to the right square of the next row | turns right, one square, turns back, one square forward |
| none | nowhere | **stays still**; the same row is flashed again |

Then the next row flickers, and so on to the end of the lane.

```text
Quest browser  <--- Wi-Fi --->  laptop: python -m questpath run  --- Bluetooth (HC-05) --->  micromouseFirmware
(the squares)                        ^                                (TURN / V / POSE commands)
                                     |
Muse --BLE--> phone, MuseLog --OSC over Wi-Fi (UDP 5000)
```

Nothing in `micromouseFirmware` had to change: this drives the mouse with its
closed-loop `TURN`, `V` and `POSE` commands.

## What has and has not been tried

- **Tried, in simulation only:** every part of the program, 85 tests, with a
  made-up wearer (EEG noise plus a small response to the flicker), a made-up
  mouse that speaks the firmware's protocol, the real page running in a
  headless desktop browser, and EEG arriving over a real network socket.
- **Not tried:** a real Quest, a real Muse, a real Micromouse. Nobody has yet
  worn this. Expect to adjust things on the first real session, above all the
  flash length and how the mouse's distances and turns come out on the floor.
- The life-size lane on the floor (AR mode) has never run on a headset.

## Try it without any hardware

```cmd
cd firmware\quest-path
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[test]"
.\.venv\Scripts\python.exe -m questpath demo
```

That runs a whole session in a few seconds of simulated time: a wearer who
looks at middle, left, left, right, middle, and a mouse that follows. Choose
the path with `--path M,L,-,R` (`-` = looking away, so nothing is picked).

To see the page itself, with the simulated wearer and mouse behind it:

```cmd
.\.venv\Scripts\python.exe -m questpath demo --page
```

and open the address it prints in a browser on the laptop or in the headset.
Press **Start**.

## The real thing

1. **Mouse.** Flash `micromouseFirmware`, switch the mouse on and keep it
   still for a second while the gyro calibrates. Pair the HC-05 and find its
   *outgoing* COM port (`python -m questpath ports`). With the wheels **on the
   floor**, check that the mouse drives one square and turns a quarter turn:

   ```cmd
   python -m questpath mouse COM7 forward
   python -m questpath mouse COM7 turn 90
   python -m questpath mouse COM7 square
   ```

   If a square comes out short or long, the wheel constants in the firmware's
   `motor.h` are the place to fix it (its README says so too). Runs are
   corrected once from the encoder distance; `--no-trim` turns that off.

2. **Headband.** Wear the Muse, open MuseLog on the phone, and in its OSC
   settings set the target IP to the laptop's address, port 5000, **full rate
   on**, then start streaming. Check that it arrives:

   ```cmd
   python -m questpath check
   ```

   The flicker shows up at the back of the head, over the visual cortex. A Muse
   has no electrode there. The lab's eye-as-a-camera rig adds one cup electrode
   on the Muse's AUX input; with it this has a far better chance. With the
   plain headband only the electrodes behind the ears see a little of it.

3. **Lane.** Mark a lane on the floor, squares of 180 mm (or pass
   `--cell-mm`). Put the mouse on the middle square of the first row, facing up
   the lane.

4. **Go.**

   ```cmd
   python -m questpath run --mouse COM7 --rows 6
   ```

   It prints the address to open in the Quest browser (same Wi-Fi). Make the
   browser window large, press **Full screen**, then **Start**.

   First comes a signal check (the whole row flickers, then stays steady, four
   times; just watch it). The program says whether the flicker shows in the
   EEG and which electrodes it will use. Then the tracing starts. **Stop** on
   the page, or Ctrl+C on the laptop, stops the mouse at once.

   Use `--mouse sim` to do all of this with the real headset and headband but
   no mouse, and `--mouse none` to only trace.

Every flash, score, decision and move is written to `runs/<time>/session.jsonl`.

## Options that matter

| option | default | what it does |
|---|---|---|
| `--flash` | 3 | seconds each square flickers. Longer = surer and slower |
| `--max-rounds` | 4 | rounds of three flashes before a row counts as "nothing" |
| `--threshold`, `--margin` | 3.0, 1.5 | how far the winning square must stand above noise, and above the runner-up |
| `--style` | `square` | `square`: straight runs and quarter turns only. `diagonal`: turn towards the square and drive straight to it |
| `--drive` | `each` | `each`: the mouse moves after every row. `end`: trace the whole path first, then drive it |
| `--rows`, `--cell-mm`, `--start` | 6, 180, M | the lane |
| `--speed` | 150 | mm/s on straight runs |
| `--no-check` | | skip the signal check |

One round of a row takes 3 x (3 + 0.5) = 10.5 s; a clear response is decided
in one or two rounds.

## How it decides

- The page flickers a square by showing it white for a whole number of display
  frames and black for the same number, so the rate is exact: 12 Hz on a 72 or
  120 Hz display, 15 Hz on 90 or 60 Hz. It reports the start and end of every
  flash on its own clock, which the laptop matches to its own.
- EEG samples arrive in bursts, so the laptop rebuilds their timing from the
  headband's steady sample rate instead of trusting arrival times.
- For each flash, canonical correlation measures how well the EEG follows a
  sine and cosine at the flicker rate and twice that rate, and compares it with
  nearby rates where nothing flickers (staying out of the 8 to 12 Hz alpha
  band, the brain's own idle rhythm). That gives one score per flash, close to
  a z-score.
- A row is decided when one square's combined score is at least 3 **and** 1.5
  above the runner-up; the neighbours of the square being looked at respond a
  little too, so both conditions are needed. If that has not happened after
  four rounds, nothing is picked.

## How well it works, in simulation

100 rows per line, 3 s flashes, a made-up wearer with 8 uV of noise per
electrode plus alpha, drift and mains hum. "Response" is the size of the
flicker response at the best electrode. These numbers say the method is sound;
they say nothing about any real head.

| simulated wearer | right square | wrong square | nothing picked | rounds |
|---|---|---|---|---|
| looking, 3 uV response | 100 % | 0 % | 0 % | 1.4 |
| looking, 1.5 uV response | 76 % | 0 % | 24 % | 3.1 |
| looking, 1 uV response | 8 % | 0 % | 92 % | 3.9 |
| looking, 0.6 uV response | 0 % | 1 % | 99 % | 4.0 |
| looking away | 0 % | 0 % | 100 % | 4.0 |
| no response at all | 0 % | 0 % | 100 % | 4.0 |
| 3 uV at the cortex but no AUX electrode (about 1 uV behind the ears) | 17 % | 0 % | 83 % | 3.8 |
| 3 uV, MuseLog at its reduced rate (64 Hz) | 67 % | 0 % | 33 % | 3.4 |
| 3 uV, 90 Hz headset (15 Hz flicker) | 98 % | 1 % | 1 % | 1.4 |
| 3 uV, samples arriving in 0.5 s bursts | 100 % | 0 % | 0 % | 1.4 |

So in simulation a weak response costs time or gives "nothing picked", not a
wrong square; a response under about 1.5 uV needs longer flashes
(`--flash 5`) or the extra electrode.

`python tests/bench_detector.py 100` repeats this.

## The lane on the floor (AR, experimental)

The page can also draw the lane life-size on the real floor in passthrough,
so that the squares lie where the mouse drives. WebXR needs a secure page, so
connect the headset by USB (developer mode on) and run

```cmd
adb reverse tcp:8090 tcp:8090
```

then open `http://localhost:8090/` in the Quest browser and press **Lane on
the floor (AR)**. The start square appears 0.6 m in front of you; pinch or pull
the trigger to lay the lane down again. This mode has not been run on a
headset yet.

## Files

```text
questpath/
  session.py    one tracing session: flash, score, decide, drive
  detect.py     the SSVEP score and the row decision
  eeg.py        recent EEG with a rebuilt sample clock
  osc.py        MuseLog's OSC stream (no dependency)
  mouse.py      micromouseFirmware protocol, and a simulated board
  grid.py       the lane and the moves between squares
  display.py    the headset page as the session sees it (real or simulated)
  server.py     the web server the page talks to
  wearer.py     a made-up wearer for rehearsals
  web/index.html  the page
tests/          pytest (the page test needs Playwright and is skipped without it)
```

## Safety

Every command that moves the mouse carries its own time limit, enforced by the
firmware, so the mouse stops by itself if the laptop or the Bluetooth link
dies. The program sends `STOP` whenever a session ends, however it ends. Keep
the lane clear, and keep a hand near the mouse on the first runs.
