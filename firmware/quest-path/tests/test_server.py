"""The headset's web server, talked to the way the page talks to it."""

import json
import threading
import time
import urllib.request

import pytest

from questpath.clock import RealClock
from questpath.display import WebDisplay
from questpath.server import PageServer


@pytest.fixture
def server():
    s = PageServer(RealClock(), port=0).start()
    yield s
    s.close()


def url(server, path):
    return "http://127.0.0.1:%d%s" % (server.port, path)


def post(server, body):
    request = urllib.request.Request(url(server, "/event"), data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=5) as reply:
        return json.loads(reply.read())


def listen(server, lines, ready):
    """Read server-sent events the way the page's EventSource does."""
    with urllib.request.urlopen(url(server, "/events"), timeout=10) as stream:
        ready.set()
        for raw in stream:
            text = raw.decode().strip()
            if text.startswith("data: "):
                lines.append(json.loads(text[6:]))
            if len(lines) >= 2:
                return


def test_the_page_is_served_and_never_cached(server):
    with urllib.request.urlopen(url(server, "/"), timeout=5) as reply:
        body = reply.read().decode()
        assert "no-store" in reply.headers["Cache-Control"]
    assert "<canvas" in body and "/events" in body and "flash_done" in body
    with pytest.raises(urllib.error.HTTPError):
        urllib.request.urlopen(url(server, "/nothing-here"), timeout=5)


def test_commands_reach_the_page_and_a_reloaded_page_gets_the_board_again(server):
    server.push({"cmd": "state", "message": "hello headset", "phase": "wait"})
    lines, ready = [], threading.Event()
    thread = threading.Thread(target=listen, args=(server, lines, ready), daemon=True)
    thread.start()
    assert ready.wait(5)
    deadline = time.time() + 5
    while server.clients() == 0 and time.time() < deadline:
        time.sleep(0.02)
    server.push({"cmd": "flash", "id": 7, "row": 1, "col": 2, "seconds": 2.0})
    thread.join(5)
    assert lines[0]["message"] == "hello headset"        # the last board, sent on connecting
    assert lines[1]["cmd"] == "flash" and lines[1]["col"] == 2


def test_page_times_are_put_on_the_laptop_clock(server):
    clock = server.clock
    # a page whose clock started 1000 s before the laptop's
    for k in range(5):
        post(server, {"type": "ping", "page": "abc", "t_send": clock.now() - 1000.0})
        time.sleep(0.01)
    now = clock.now()
    assert abs(server.to_laptop(now - 1000.0) - now) < 0.05
    assert server.events() == []                         # pings are not passed on
    # the page reloads: its clock starts again, the old offset must go
    post(server, {"type": "ping", "page": "new", "t_send": 3.0})
    assert abs(server.to_laptop(3.0) - clock.now()) < 0.05


def test_bad_posts_are_refused(server):
    request = urllib.request.Request(url(server, "/event"), data=b"not json")
    with pytest.raises(urllib.error.HTTPError):
        urllib.request.urlopen(request, timeout=5)


def test_web_display_turns_page_reports_into_epochs(server):
    clock = server.clock
    display = WebDisplay(server, clock)
    assert not display.ready()
    lines, ready = [], threading.Event()
    threading.Thread(target=listen, args=(server, lines, ready), daemon=True).start()
    assert ready.wait(5)
    post(server, {"type": "hello", "page": "p", "t_send": 10.0, "refresh": 72, "hz": 12.0, "agent": "test"})
    deadline = time.time() + 5
    while not display.ready() and time.time() < deadline:
        time.sleep(0.02)
    assert display.ready() and display.refresh == 72

    flash_id = display.flash(2, 0, 2.0)
    asked = clock.now()
    assert display.known_until() <= asked                # what is on the screen is not known yet
    # the page says it flashed from 10.5 to 12.5 on its clock (which is `asked - 10` behind)
    post(server, {"type": "flash_done", "page": "p", "t_send": 12.6, "id": flash_id, "t0": 10.5, "t1": 12.5,
                  "hz": 12.0, "refresh": 72, "dropped": 1})
    post(server, {"type": "button", "page": "p", "t_send": 12.7, "name": "stop"})
    events = display.poll()
    done = [e for e in events if e["type"] == "flash_done"][0]["epoch"]
    assert (done.row, done.col, done.hz, done.dropped) == (2, 0, 12.0, 1)
    assert abs((done.t1 - done.t0) - 2.0) < 1e-6
    assert {"type": "button", "name": "stop"} in events
    assert display.epochs == [done]
    assert display.known_until() >= asked
    assert display.poll() == []
