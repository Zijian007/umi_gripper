"""Explicit no-hardware ServoBus substitute, with instantaneous position feedback.

This models the controller protocol, not force, inertia or physical gripping.
"""

import threading


class FakeServoBus:
    def __init__(self, port, baudrate, *, initial_position=2048):
        self.port_name, self.baudrate = port, baudrate
        self.initial_position = initial_position
        self.opened = False
        self.positions = {}
        self.commands = []
        self.stalled = False
        self.temperature_c = 25
        self.telemetry_error = None
        self._lock = threading.Lock()

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def _require_open(self):
        if not self.opened:
            raise RuntimeError("fake bus is closed")

    def verify(self, sid):
        self._require_open()
        return 3215

    def check_position_mode(self, sid):
        self._require_open()

    def prepare(self, sid, strength):
        self.set_grip_strength(sid, strength)

    def set_grip_strength(self, sid, strength):
        with self._lock:
            self._require_open()
            self.commands.append(("strength", sid, strength))

    def set_position(self, sid, position, speed, accel=50):
        with self._lock:
            self._require_open()
            self.commands.append(("position", sid, position, speed, accel))
            if not self.stalled:
                self.positions[sid] = position

    def telemetry(self, sid):
        with self._lock:
            self._require_open()
            if self.telemetry_error is not None:
                raise self.telemetry_error
            return {"raw_position": self.positions.get(sid, self.initial_position),
                    "load": 0, "current_ma": 0, "temperature_c": self.temperature_c}
