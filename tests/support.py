"""Simulated serial transport and calibration fixtures for offline tests."""

import json
import threading
from functools import partial
from pathlib import Path


class FakeServoBus:
    """Simulate servo commands and immediate position feedback."""

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

    def configure_position_mode(self, sid):
        self._require_open()
        self.commands.append(("position_mode", sid))

    def set_current_position_as_middle(self, sid):
        self._require_open()
        self.commands.append(("set_middle", sid))

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


fake_bus_at_midpoint = partial(FakeServoBus, initial_position=1500)


def write_calibration(path: Path, *, channels=(1,), port="/dev/ttyUMI_TEST", baudrate=1_000_000, **overrides) -> Path:
    servos = [None, None]
    for channel in channels:
        servos[channel] = {
            "serial_port": str(port),
            "servo_id": channel + 1,
            "baudrate": baudrate,
            "closed_position_steps": 2000,
            "neutral_position_steps": 1500,
            "open_position_steps": 1000,
            "speed": 1200,
            "open_grip_strength": 40,
            "close_grip_strength": 40,
            "status_frequency_hz": 50,
            "calibration_step_steps": 128,
            "motion_timeout_s": 30,
            **overrides,
        }
    path.write_text(json.dumps({"servos": servos}), encoding="utf-8")
    return path
