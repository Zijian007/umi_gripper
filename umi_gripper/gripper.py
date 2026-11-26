"""Public control of one physical UMI gripper."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from .bus import UmiError
from .config import (
    _CALIBRATION_KEYS,
    _DRIVER_SETTING_KEYS,
    _servo_config,
    config_file,
    default_servo_config,
    load_servo_config,
)
from .session import UmiReading, _open_servo, _OpeningCleanupError, _UmiSession


def _device_info(channel: int, config: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "channel": channel,
        "position_unit": "steps",
        "calibration": {key: config[key] for key in _CALIBRATION_KEYS},
        "runtime": {key: config[key] for key in _DRIVER_SETTING_KEYS},
    }


class Gripper:
    """Exclusive control of one servo on a shared serial session."""

    def __init__(self, session: _UmiSession, channel: int, servo_id: int):
        self._session = session
        self._channel = channel
        self._servo_id = servo_id
        self._closed = False
        self._closing = False
        self._close_lock = threading.Lock()

    @classmethod
    def open(
        cls, *, channel: int, config_path: str | Path | None = None, settings: Mapping[str, Any] | None = None,
    ) -> Gripper:
        config = load_servo_config(config_file(config_path), channel, settings)
        return cls._open_config(channel, config)

    @staticmethod
    def validate_config(
        *, channel: int, config_path: str | Path | None = None, settings: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Validate one channel offline and return the effective device information."""
        config = load_servo_config(config_file(config_path), channel, settings)
        return _device_info(channel, config)

    @classmethod
    def _open_config(
        cls, channel: int, config: dict[str, Any], *, calibrating: bool = False,
    ) -> Gripper:
        servo_id = config["servo_id"]
        try:
            session = _open_servo(channel, config, calibrating=calibrating)
        except _OpeningCleanupError as exc:
            exc.resource = cls(exc.session, channel, servo_id)
            raise
        return cls(session, channel, servo_id)

    @classmethod
    def _open_for_calibration(cls, channel: int, selected: dict[str, Any]) -> Gripper:
        config = _servo_config(default_servo_config(selected), channel, calibrated=False)
        return cls._open_config(channel, config, calibrating=True)

    def __enter__(self) -> Gripper:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _config_snapshot(self) -> dict[str, Any]:
        with self._close_lock:
            self._require_open()
            return self._session.config_snapshot(self._servo_id)

    def _set_position(self, position: int, *, speed: int | None = None) -> None:
        with self._close_lock:
            self._require_open()
            self._session.set_position_steps(self._servo_id, position, speed=speed)

    def _set_calibration(self, points: Mapping[str, Any]) -> None:
        with self._close_lock:
            self._require_open()
            self._session.update_calibration(self._servo_id, points)

    def _set_grip_strength(self, key: str, strength: int) -> None:
        with self._close_lock:
            self._require_open()
            self._session.set_grip_strength(self._servo_id, key, strength)

    def wait(self, timeout: float | None = None) -> UmiReading:
        """Wait for the current motion to settle and return its last reading."""
        if timeout is None:
            timeout = self._config_snapshot().get("motion_timeout_s", 30)
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout < 0:
            raise ValueError("timeout must be a finite non-negative number of seconds")
        deadline = time.monotonic() + timeout
        while (reading := self.read()).state == "moving":
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise UmiError(f"UMI servo ID {self._servo_id} motion timed out")
            time.sleep(min(0.02, remaining))
        return reading

    def device_info(self) -> dict[str, Any]:
        return _device_info(self._channel, self._config_snapshot())

    def set_target(self, close_01: float, *, retry: bool = False) -> None:
        """Submit a normalized target, optionally rearming an identical settled target."""
        with self._close_lock:
            self._require_open()
            self._session.set_close_01(self._servo_id, close_01, retry=retry)

    def read(self) -> UmiReading:
        with self._close_lock:
            self._require_open()
            return self._session.reading(self._servo_id)

    def stop(self) -> None:
        self._stop()

    def _stop(self, *, timeout: float = 0.5, allow_invalid: bool = False) -> None:
        with self._close_lock:
            self._require_open()
            if not self._session.stop_channel(self._servo_id, timeout=timeout, allow_invalid=allow_invalid):
                raise UmiError(f"UMI servo ID {self._servo_id} stop timed out")

    def _require_open(self) -> None:
        if self._closing:
            raise UmiError("UMI gripper is closing or closed")

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closing = True
            self._session.release_channel(self._servo_id)
            self._closed = True
