"""The small web server the headset's browser talks to.

    GET  /          the page (never cached: a stale copy in the headset's
                    browser would silently run old stimulus code)
    GET  /events    server-sent events: commands for the page
    POST /event     what the page reports (hello, flash_done, button, ping)

The page stamps what it sends with its own clock (seconds since it loaded).
The laptop turns page times into its own by adding the smallest
(arrival - page time) seen lately: the message that crossed the network
fastest is the best estimate of the difference between the two clocks.
"""

from __future__ import annotations

import json
import os
import queue
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


class PageServer:
    def __init__(self, clock, port: int = 8090, web_dir: str = WEB_DIR):
        self.clock = clock
        self.port = port
        self.web_dir = web_dir
        self._events = queue.Queue()
        self._clients = []
        self._lock = threading.Lock()
        self._offsets = deque()          # (arrival, arrival - page time)
        self._page_id = None
        self.last_state = None
        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), self._handler())
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        self._thread = threading.Thread(target=self.httpd.serve_forever, name="questpath-http", daemon=True)

    def start(self):
        self._thread.start()
        return self

    def close(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()

    # ---- towards the page
    def push(self, obj: dict) -> None:
        text = json.dumps(obj)
        with self._lock:
            if obj.get("cmd") == "state":
                self.last_state = text
            for q in self._clients:
                q.put(text)

    def clients(self) -> int:
        with self._lock:
            return len(self._clients)

    # ---- from the page
    def events(self):
        out = []
        while True:
            try:
                out.append(self._events.get_nowait())
            except queue.Empty:
                return out

    def to_laptop(self, page_time: float) -> float:
        with self._lock:
            if not self._offsets:
                return self.clock.now()
            return page_time + min(d for _, d in self._offsets)

    def _note_clock(self, body: dict, arrival: float) -> None:
        sent = body.get("t_send")
        if not isinstance(sent, (int, float)):
            return
        with self._lock:
            if body.get("page") != self._page_id:        # the page was reloaded: new clock
                self._page_id = body.get("page")
                self._offsets.clear()
            self._offsets.append((arrival, arrival - float(sent)))
            while self._offsets and self._offsets[0][0] < arrival - 30.0:
                self._offsets.popleft()

    # ---- http
    def _handler(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):          # keep the console for the session
                pass

            def _send(self, code, body: bytes, kind: str):
                self.send_response(code)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store, max-age=0")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?")[0]
                if path in ("/", "/index.html"):
                    try:
                        with open(os.path.join(server.web_dir, "index.html"), "rb") as f:
                            self._send(200, f.read(), "text/html; charset=utf-8")
                    except OSError:
                        self._send(500, b"page missing", "text/plain")
                elif path == "/events":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "keep-alive")
                    self.end_headers()
                    q = queue.Queue()
                    with server._lock:
                        server._clients.append(q)
                        if server.last_state:
                            q.put(server.last_state)
                    try:
                        while True:
                            try:
                                text = q.get(timeout=5.0)
                                self.wfile.write(("data: " + text + "\n\n").encode())
                            except queue.Empty:
                                self.wfile.write(b": still here\n\n")
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionError, OSError):
                        pass
                    finally:
                        with server._lock:
                            if q in server._clients:
                                server._clients.remove(q)
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                arrival = server.clock.now()
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(size) or b"{}")
                    if not isinstance(body, dict):
                        raise ValueError
                except (ValueError, json.JSONDecodeError):
                    self._send(400, b"{}", "application/json")
                    return
                if self.path.split("?")[0] != "/event":
                    self._send(404, b"{}", "application/json")
                    return
                server._note_clock(body, arrival)
                if body.get("type") != "ping":
                    server._events.put(body)
                self._send(200, json.dumps({"t": arrival}).encode(), "application/json")

        return Handler
