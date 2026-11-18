"""The operator's side: keys, one status line, spoken prompts.

Prompts are spoken because the wearer has their eyes closed for half of the
calibration. On Windows one speech process is started and fed lines, so a
prompt starts at once; macOS uses `say`, Linux `spd-say` or `espeak` when
one is installed. Without any of those the prompts are only printed.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import time

try:
    import msvcrt
except ImportError:
    msvcrt = None

WINDOWS_VOICE = ("Add-Type -AssemblyName System.Speech; "
                 "$voice = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                 "while ($null -ne ($line = [Console]::In.ReadLine())) "
                 "{ [void]$voice.SpeakAsync($line) }; "
                 # the program has ended: finish what is being said, then go
                 "while ($voice.State -ne 'Ready') { Start-Sleep -Milliseconds 100 }")


class Console:
    def __init__(self, speak=True):
        self.speak = speak
        self._voice = None
        self._line_open = False
        self._printed = 0.0
        self._key_at = 0.0
        self._restore = None
        if msvcrt is None and sys.stdin.isatty():
            import termios
            import tty
            self._restore = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)

    def key(self):
        """One waiting key press, lower case, or "". A held key repeats,
        so whatever else is waiting is dropped and a press within 0.4 s of
        the last accepted one is ignored."""
        pressed = ""
        if msvcrt is not None:
            while msvcrt.kbhit():
                char = msvcrt.getwch()
                if char in "\x00\xe0":          # arrow or function key
                    msvcrt.getwch()
                else:
                    pressed = pressed or char
        elif self._restore is not None:
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                pressed = os.read(sys.stdin.fileno(), 64).decode(errors="ignore")[:1]
        if not pressed:
            return ""
        now = time.monotonic()
        if now - self._key_at < 0.4:
            return ""
        self._key_at = now
        return pressed.lower()

    def status(self, text):
        """Overwrites one line of the terminal; when the output goes to a
        file instead, one line a second."""
        if sys.stdout.isatty():
            width = shutil.get_terminal_size().columns - 1
            print("\r" + text[:width].ljust(width), end="", flush=True)
            self._line_open = True
        elif time.monotonic() - self._printed >= 1.0:
            self._printed = time.monotonic()
            print(text, flush=True)

    def note(self, text):
        """A message on a line of its own."""
        if self._line_open:
            print()
            self._line_open = False
        print(text, flush=True)

    def say(self, text):
        self.note(text)
        if not self.speak:
            return
        try:
            if sys.platform == "win32":
                if self._voice is None:
                    self._voice = subprocess.Popen(
                        ["powershell", "-NoProfile", "-Command", WINDOWS_VOICE],
                        stdin=subprocess.PIPE, text=True,
                        creationflags=subprocess.CREATE_NO_WINDOW)
                self._voice.stdin.write(text.replace("\n", " ") + "\n")
                self._voice.stdin.flush()
            else:
                for program in ("say", "spd-say", "espeak"):
                    if shutil.which(program):
                        subprocess.Popen([program, text], stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
                        break
        except OSError:
            self.speak = False          # no voice on this machine: print only

    def beep(self):
        try:
            import winsound
            winsound.MessageBeep()      # returns at once
        except ImportError:
            print("\a", end="", flush=True)

    def cue(self, kind, now, calibrating=False):
        """Called with every cue given to the wearer; a made-up wearer
        listens here."""

    def close(self):
        if self._line_open:
            print()
            self._line_open = False
        if self._voice is not None:
            try:
                self._voice.stdin.close()
            except OSError:
                pass
        if self._restore is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._restore)


class Silent:
    """A console for unattended runs: keeps the notes, says nothing."""

    def __init__(self, listener=None):
        self.notes = []
        self.listener = listener        # called as listener(kind, now, calibrating)

    def key(self):
        return ""

    def status(self, text):
        pass

    def note(self, text):
        self.notes.append(text)

    say = note

    def beep(self):
        pass

    def cue(self, kind, now, calibrating=False):
        if self.listener is not None:
            self.listener(kind, now, calibrating)

    def close(self):
        pass
