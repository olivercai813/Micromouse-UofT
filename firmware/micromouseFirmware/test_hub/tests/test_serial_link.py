import queue
import threading
import unittest
from unittest.mock import patch

from micromouse_hub.serial_link import SerialLink


class FakeSerial:
    def __init__(self, port, **_kwargs):
        self.port = port
        self.is_open = True
        self.closed = threading.Event()
        self.writes = []

    def read(self, _size):
        self.closed.wait(0.05)
        return b""

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)

    def flush(self):
        return None

    def close(self):
        self.is_open = False
        self.closed.set()


class SerialLinkTests(unittest.TestCase):
    def test_reconnect_stops_old_reader_before_using_new_port(self):
        created = []

        def factory(port, **kwargs):
            instance = FakeSerial(port, **kwargs)
            created.append(instance)
            return instance

        link = SerialLink(queue.Queue())
        with patch("micromouse_hub.serial_link.serial.Serial", side_effect=factory):
            link.connect("COM7")
            old_thread = link._thread
            link.connect("COM8")
            self.assertFalse(created[0].is_open)
            self.assertIsNot(link._thread, old_thread)
            self.assertFalse(old_thread.is_alive())
            self.assertEqual(link._serial.port, "COM8")
            link.disconnect(send_stop=False)


if __name__ == "__main__":
    unittest.main()
