"""The real page in a real (headless) browser, against the real server.

A simulated wearer and mouse stand in for the hardware; everything between
them is the code that runs in a session: the web server, the page's frame-
locked flicker, its reports, the clock matching, the detector, the mouse
driver. Takes about a minute of real time. Skipped without Playwright
(pip install playwright; playwright install chromium).
"""

import os
import re
import subprocess
import sys
import time

import pytest

sync_api = pytest.importorskip("playwright.sync_api")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PORT = 8097


def browser(play):
    for options in ({}, {"channel": "chrome"}, {"channel": "msedge"}):
        try:
            return play.chromium.launch(headless=True, **options)
        except Exception:
            continue
    pytest.skip("no browser for Playwright")


def test_a_session_through_the_real_page(tmp_path):
    command = [sys.executable, "-m", "questpath", "demo", "--page", "--port", str(PORT), "--auto",
               "--path", "L,R", "--rows", "3", "--flash", "1.5", "--gap", "0.3", "--check-blocks", "2",
               "--response", "8", "--seed", "5"]
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    shot = str(tmp_path / "page.png")
    errors = []
    try:
        with sync_api.sync_playwright() as play:
            chrome = browser(play)
            page = chrome.new_page(viewport={"width": 1280, "height": 720})
            page.on("pageerror", lambda exc: errors.append(str(exc)))
            for _ in range(50):                      # the server needs a moment
                try:
                    page.goto("http://127.0.0.1:%d/" % PORT)
                    break
                except Exception:
                    time.sleep(0.2)
            page.wait_for_function("refreshHz > 0 && connected", timeout=15000)
            page.wait_for_function("flash !== null", timeout=30000)        # a square is flickering
            page.screenshot(path=shot)
            seen = page.evaluate("({ hz: flash ? flash.hz : 0, refresh: refreshHz, rows: state.rows })")
            output, _ = process.communicate(timeout=180)
            final = page.evaluate("state")
            chrome.close()
    finally:
        if process.poll() is None:
            process.kill()
    assert not errors, errors
    assert "outcome: finished" in output, output
    assert re.search(r"path:\s+start M -> L -> R", output), output
    assert seen["rows"] == 3 and seen["refresh"] >= 50
    assert abs(seen["hz"] * round(seen["refresh"] / (2 * seen["hz"])) * 2 - seen["refresh"]) < 1e-6   # whole frames
    assert final["phase"] == "done" and final["mouse"] == [2, 2]
    assert os.path.getsize(shot) > 2000
    keep = os.environ.get("QUESTPATH_SCREENSHOT")
    if keep:
        import shutil
        shutil.copy(shot, keep)


def test_the_real_command_with_an_eeg_stream_that_holds_no_response(tmp_path):
    """`questpath run` end to end: EEG over a real UDP socket (noise only), the
    page in a browser, a simulated mouse. With nothing in the EEG the signal
    check must warn, no square may be picked, and the mouse must not move."""
    import socket
    import threading

    import numpy as np

    from questpath import osc

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    eeg_port = probe.getsockname()[1]
    probe.close()
    command = [sys.executable, "-m", "questpath", "run", "--mouse", "sim", "--auto", "--port", str(PORT + 1),
               "--eeg-port", str(eeg_port), "--rows", "3", "--flash", "1.5", "--gap", "0.3", "--check-blocks", "1",
               "--max-rounds", "1", "--max-idle", "1", "--wait", "30", "--log", str(tmp_path / "run")]
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    stop = threading.Event()

    def headband():
        rng = np.random.default_rng(0)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        start, sent = time.perf_counter(), 0
        while not stop.is_set():
            due = int((time.perf_counter() - start) * 256)
            for _ in range(due - sent):
                sender.sendto(osc.message("/muse/eeg", rng.normal(0, 8, 5)), ("127.0.0.1", eeg_port))
            sent = due
            time.sleep(0.01)

    thread = threading.Thread(target=headband, daemon=True)
    thread.start()
    try:
        with sync_api.sync_playwright() as play:
            chrome = browser(play)
            page = chrome.new_page(viewport={"width": 1280, "height": 720})
            for _ in range(50):
                try:
                    page.goto("http://127.0.0.1:%d/" % (PORT + 1))
                    break
                except Exception:
                    time.sleep(0.2)
            output, _ = process.communicate(timeout=180)
            chrome.close()
    finally:
        stop.set()
        if process.poll() is None:
            process.kill()
    assert "EEG: 25" in output and "TP9, AF7, AF8, TP10, AUX" in output, output      # about 256 samples/s, 5 electrodes
    assert "WARNING: the flicker barely shows" in output, output
    assert "nothing picked" in output and "gave up" in output, output
    assert output.rstrip().endswith("path:    start M"), output          # no square was added to the path
    records = [line for line in open(tmp_path / "run" / "session.jsonl", encoding="utf-8")]
    assert any('"kind": "flash"' in r for r in records) and not any('"kind": "moved"' in r for r in records)
