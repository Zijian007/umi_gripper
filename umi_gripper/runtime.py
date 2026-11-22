"""UMI gripper: one ``UmiGripper`` per side.

Sides that share a calibration file share one ``_UmiSession`` (buses and the worker).
Feetech register I/O lives in ``bus.py``. Calibration and step conversion live
in ``config.py``.
"""

from __future__ import annotations

import hashlib
import json
import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Literal, Mapping

from .bus import STEPS_PER_TURN, ServoBus, ServoBusPort, UmiError
from .config import (
    _CALIBRATION_KEYS,
    _MOTION_OPTION_KEYS,
    _RUNTIME_OPTION_KEYS,
    _calibrated_channel,
    _calibration_points,
    _checked_int,
    _close_01,
    _invalid_position_message,
    _move_direction,
    _position_in_span,
    _read_servos,
    _require_close_01,
    _servo_config,
    _side_channel,
    _steps_for_close_01,
    _width,
)

_LOGGER = logging.getLogger(__name__)

INITIAL_POSITION_TIMEOUT_S = 2.0
_CALIBRATION_GRIP_STRENGTH = 40
StateName = Literal["ready", "moving", "arrived", "grasped", "blocked", "error"]
CommandPhase = Literal["stopping", "idle", "follow"]

_sessions: dict[str, "_UmiSession"] = {}
_sessions_lock = threading.Lock()


@dataclass(frozen=True)
class _Sample:
    """One worker tick. Send and stall re-check ``version`` under the lock."""

    position: int
    temperature_c: int
    target: int | None
    version: int
    stop_version: int
    phase: CommandPhase


@dataclass
class _Channel:
    index: int
    config: dict[str, Any]
    bus: ServoBusPort
    calibrated: bool = True
    position: int | None = None
    position_valid: bool = False
    target: int | None = None
    target_speed: int | None = None
    version: int = 0
    blocked_direction: str | None = None
    motion_reference_position: int | None = None
    motion_reference_at: float | None = None
    sent_version: int | None = None
    stop_pending: bool = False
    stop_version: int = 0
    stop_result: tuple[StateName, str, str | None] = ("ready", "", None)
    stopped: threading.Event = field(default_factory=threading.Event)
    state: StateName = "ready"
    message: str = ""
    telemetry: dict | None = None
    sampled_at_mono_s: float = 0.0
    next_status_poll_at: float = 0.0
    initial_ready: threading.Event = field(default_factory=threading.Event)
    initial_error: UmiError | None = None
    applied_strength: int | None = None
    active_strength_key: str = "close_grip_strength"
    pending_strength: tuple[str, int] | None = None
    strength_error: UmiError | None = None
    strength_applied: threading.Event = field(default_factory=threading.Event)
    ref_count: int = 0
    overtemp_since: float | None = None
    overtemp_latched: bool = False
    bus_fault: bool = False
    error_code: int = 0

    def command_phase(self) -> CommandPhase:
        """stopping | idle | follow. Send versus stall is ``sent_version``."""
        if self.stop_pending:
            return "stopping"
        if self.target is None:
            return "idle"
        return "follow"


@dataclass(frozen=True)
class UmiReading:
    """Cached channel feedback in normalized closure, steps, amperes and Celsius."""

    close_01: float | None
    sampled_at_mono_s: float
    state: StateName
    message: str
    current: float
    temperature: float
    effort: float
    raw_position: float | None


class _OpeningCleanupError(UmiError):
    """Opening failed and the caller still owns a resource that requires close."""


class _UmiSession:
    """Shared buses and worker for one calibration file.

    Calls from ``UmiGripper`` come first. The worker tick follows.
    """

    def __init__(
        self, config_path: Path, *, worker_target: Callable | None = None,
        join_worker: Callable | None = None,
        bus_factory: Callable[[str, int], ServoBusPort] | None = None,
    ):
        self.config_path = config_path
        self._worker_target = worker_target
        self._join_worker = join_worker or self._join_standalone_worker
        self._bus_factory = bus_factory or ServoBus
        self._lifecycle_lock = threading.Lock()
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.wakeup = threading.Event()
        self.buses: dict[tuple[str, int], ServoBusPort] = {}
        self.channels: dict[int, _Channel] = {}
        self.worker: threading.Thread | None = None
        self._closed = False
        self._closing_channels: dict[int, tuple[str, int]] = {}

    def open_channel(
        self, channel: int, config: dict[str, Any], *, calibrated: bool = True,
        initialize_calibration: bool = False,
    ) -> None:
        with self._lifecycle_lock:
            if self._closed:
                raise UmiError("umi session is closed")
            if self.stop_event.is_set() or self._closing_channels:
                raise UmiError("umi session cleanup is pending")
            self._open_channel(channel, config, calibrated, initialize_calibration)

    def _open_channel(
        self, channel: int, config: dict[str, Any], calibrated: bool, initialize_calibration: bool,
    ) -> None:
        with self.lock:
            if channel in self.channels:
                self.channels[channel].ref_count += 1
                return
            key = (config["serial_port"], config["baudrate"])
            for existing in self.channels.values():
                if existing.config["serial_port"] == config["serial_port"] and (
                    existing.config["servo_id"] == config["servo_id"] or existing.config["baudrate"] != config["baudrate"]
                ):
                    raise UmiError("channels sharing a port require distinct servo IDs and one baudrate")
            bus = self.buses.get(key)
            strength_key = "close_grip_strength" if calibrated else "calibration_grip_strength"
            strength = config[strength_key] if calibrated else _CALIBRATION_GRIP_STRENGTH
            sid = config["servo_id"]
            try:
                if bus is None:
                    bus = self._bus_factory(*key)
                    self.buses[key] = bus
                    bus.open()
                bus.verify(sid)
                if initialize_calibration and not calibrated:
                    bus.configure_position_mode(sid)
                    bus.set_current_position_as_middle(sid)
                bus.check_position_mode(sid)
                bus.prepare(sid, strength)
            except Exception:
                if bus is not None and not any(other.bus is bus for other in self.channels.values()):
                    self._closing_channels[channel] = key
                    self._rollback_open(channel)
                raise
            servo = _Channel(
                index=channel,
                config=config,
                bus=bus,
                calibrated=calibrated,
                applied_strength=strength,
                active_strength_key=strength_key,
                state="moving",
                message="reading initial position",
                ref_count=1,
            )
            servo.stopped.set()
            self.channels[channel] = servo
        try:
            if self.worker is None or not self.worker.is_alive():
                target = self._worker_target(self._run) if self._worker_target is not None else self._run
                self.worker = threading.Thread(target=target, name="umi-gripper", daemon=True)
                self.worker.start()
            self._wait_initial(servo)
        except Exception:
            self._rollback_open(channel)
            raise

    def _rollback_open(self, channel: int) -> None:
        try:
            self._release_channel(channel)
        except Exception as exc:
            raise _OpeningCleanupError(f"UMI open cleanup incomplete: {exc}") from exc

    def _wait_initial(self, servo: _Channel) -> None:
        if not servo.initial_ready.wait(INITIAL_POSITION_TIMEOUT_S):
            with self.lock:
                error = servo.initial_error
            detail = f": {error}" if error else ""
            raise UmiError(
                f"umi channel {servo.index} did not report a position within "
                f"{INITIAL_POSITION_TIMEOUT_S:g}s{detail}"
            )
        with self.lock:
            if servo.initial_error is not None:
                raise servo.initial_error

    def set_close_01(self, channel: int, close_01: float) -> None:
        close_01 = _require_close_01(close_01)
        with self.lock:
            servo = self._require_channel(channel)
            if not servo.calibrated:
                raise UmiError(f"umi channel {channel} is not calibrated")
            if servo.position is None or not servo.position_valid:
                raise UmiError(_invalid_position_message(
                    servo.index, servo.position, servo.config, detail=servo.message
                ))
            self._queue_move(servo, _steps_for_close_01(servo.config, close_01))
        self.wakeup.set()

    def set_position_steps(self, channel: int, position: int, *, speed: int | None = None) -> None:
        """Queue an explicit position for calibration or neutral positioning."""
        _checked_int(position, "position", 0, STEPS_PER_TURN - 1)
        if speed is not None:
            _checked_int(speed, "speed", 1, 3400)
        with self.lock:
            servo = self._require_channel(channel)
            if servo.position is None or not servo.position_valid:
                raise UmiError(f"umi channel {channel} has no valid position")
            if servo.calibrated and not _position_in_span(servo.config, position):
                raise UmiError(f"umi channel {channel} target is outside calibration")
            self._queue_move(servo, position, speed=speed)
        self.wakeup.set()

    def update_calibration(self, channel: int, config: Mapping[str, Any]) -> None:
        points = _calibration_points(config, channel)
        with self.lock:
            servo = self._require_channel(channel)
            servo.config.update(points)
            servo.calibrated = True

    def set_grip_strength(self, channel: int, key: str, strength: int) -> None:
        with self._lifecycle_lock:
            with self.lock:
                servo = self._require_channel(channel)
                servo.pending_strength = (key, strength)
                servo.strength_error = None
                servo.strength_applied.clear()
            self.wakeup.set()
            if not servo.strength_applied.wait(2):
                raise UmiError("updating UMI grip strength timed out")
            if servo.strength_error is not None:
                raise servo.strength_error

    def _queue_move(self, servo: _Channel, target: int, *, speed: int | None = None) -> None:
        """Arm a move. Leave an in-flight stop pending so the worker can finish it."""
        servo.blocked_direction = None
        servo.version += 1
        servo.target = target
        servo.target_speed = speed
        servo.motion_reference_position = None
        servo.motion_reference_at = None
        servo.sent_version = None
        servo.state = "moving"
        servo.message = "moving"

    def stop_channel(self, channel: int, *, timeout: float = 0.5) -> bool:
        with self.lock:
            servo = self._require_channel(channel)
            self._request_stop(servo, "ready", "stopped at current position")
        self.wakeup.set()
        return servo.stopped.wait(timeout)

    def release_channel(self, channel: int) -> None:
        with self._lifecycle_lock:
            self._release_channel(channel)

    def _release_channel(self, channel: int) -> None:
        if self.stop_event.is_set() or channel in self._closing_channels:
            self._finish_close(channel)
            return
        with self.lock:
            servo = self.channels.get(channel)
            if servo is None:
                return
            servo.ref_count -= 1
            if servo.ref_count > 0:
                return
            self._request_stop(servo, "ready", "channel closed")
        self.wakeup.set()
        servo.stopped.wait(0.5)
        with self.lock:
            self.channels.pop(channel, None)
            if not self.channels:
                self.stop_event.set()
                self.wakeup.set()
            elif not any(other.bus is servo.bus for other in self.channels.values()):
                self._closing_channels[channel] = (servo.config["serial_port"], servo.config["baudrate"])
        self._finish_close(channel)

    def _finish_close(self, channel: int) -> None:
        """Release only resources that have finished closing, so failed closes are retryable."""
        if self._closed:
            return
        if self.stop_event.is_set():
            if self.worker is not None and not self._join_worker(self.worker):
                raise UmiError("UMI worker did not stop; retry close after it exits")
            self.worker = None
            for key, bus in tuple(self.buses.items()):
                bus.close()
                del self.buses[key]
            self._closing_channels.clear()
            self._closed = True
        elif channel in self._closing_channels:
            key = self._closing_channels[channel]
            self.buses[key].close()
            del self.buses[key]
            del self._closing_channels[channel]

    @staticmethod
    def _join_standalone_worker(worker: threading.Thread | None) -> bool:
        if worker is None or worker.ident is None:
            return True
        if worker is threading.current_thread():
            return False
        worker.join(timeout=2.0)
        if not worker.is_alive():
            return True
        _LOGGER.error("UMI worker did not stop within 2 s; keeping its serial ports open")
        return False

    def _require_channel(self, channel: int) -> _Channel:
        servo = self.channels.get(channel)
        if servo is None:
            raise UmiError(f"umi channel {channel} is not open")
        return servo

    def reading(self, channel: int) -> UmiReading:
        with self.lock:
            servo = self._require_channel(channel)
            close_01 = None
            if servo.calibrated and servo.position is not None and servo.position_valid:
                close_01 = _close_01(servo.config, servo.position)
            current_a = 0.0
            temperature = 0.0
            effort = 0.0
            if servo.telemetry is not None:
                current_a = float(servo.telemetry["current_ma"]) / 1000.0
                temperature = float(servo.telemetry["temperature_c"])
                effort = float(servo.telemetry["load"])
            return UmiReading(
                close_01=close_01,
                sampled_at_mono_s=servo.sampled_at_mono_s,
                state=servo.state,
                message=servo.message,
                current=current_a,
                temperature=temperature,
                effort=effort,
                raw_position=float(servo.position) if servo.position is not None else None,
            )

    def _run(self) -> None:
        while not self.stop_event.is_set():
            now = time.monotonic()
            next_due = now + 0.05
            with self.lock:
                servos = list(self.channels.values())
            for servo in servos:
                self._apply_pending_strength(servo)
                if now >= servo.next_status_poll_at:
                    self._update(servo)
                    servo.next_status_poll_at = now + 1.0 / servo.config["status_frequency_hz"]
                next_due = min(next_due, servo.next_status_poll_at)
            self.wakeup.wait(max(0.001, next_due - time.monotonic()))
            self.wakeup.clear()

    def _update(self, servo: _Channel) -> None:
        try:
            sample = self._ingest(servo)
            if sample is None:
                return
            if sample.phase == "stopping":
                self._complete_stop(servo, sample)
                return
            if self._temperature_fault(servo, sample):
                self.wakeup.set()
                return
            if sample.phase == "idle":
                return
            self._follow_target(servo, sample)
        except UmiError as exc:
            self._record_bus_error(servo, exc)

    def _ingest(self, servo: _Channel) -> _Sample | None:
        telemetry = servo.bus.telemetry(servo.config["servo_id"])
        now = time.monotonic()
        with self.lock:
            servo.position = telemetry["raw_position"]
            servo.position_valid = not servo.calibrated or _position_in_span(servo.config, servo.position)
            servo.telemetry = telemetry
            servo.sampled_at_mono_s = now
            self._clear_bus_fault(servo)
            self._note_initial(servo)
            sample = _Sample(
                position=servo.position,
                temperature_c=telemetry["temperature_c"],
                target=servo.target,
                version=servo.version,
                stop_version=servo.stop_version,
                phase=servo.command_phase(),
            )
            if servo.position_valid:
                return sample
            servo.target = None
            servo.state = "error"
            servo.message = _invalid_position_message(
                servo.index, servo.position, servo.config, detail=servo.message
            )
            return None

    def _note_initial(self, servo: _Channel) -> None:
        if servo.initial_ready.is_set():
            return
        if servo.position_valid:
            servo.initial_error = None
            servo.state = "ready"
            servo.message = "ready"
        else:
            servo.initial_error = UmiError(_invalid_position_message(
                servo.index, servo.position, servo.config, detail=servo.message
            ))
            servo.state = "error"
            servo.message = str(servo.initial_error)
        servo.initial_ready.set()

    def _temperature_fault(self, servo: _Channel, sample: _Sample) -> bool:
        """Stop only after the servo stays hot. Clear that error once it cools.

        Thresholds come from the channel config (``overtemp_c``,
        ``overtemp_hold_s``, ``overtemp_clear_c``). One sample at the trip
        temperature does not stop unless the hold time is 0. The error stays
        until the temperature is back at or below the clear threshold.
        """
        now = time.monotonic()
        with self.lock:
            trip_c = servo.config["overtemp_c"]
            clear_c = servo.config["overtemp_clear_c"]
            hold_s = servo.config["overtemp_hold_s"]
            if sample.temperature_c >= trip_c:
                if servo.overtemp_since is None:
                    servo.overtemp_since = now
                if now - servo.overtemp_since < hold_s:
                    return False
                servo.overtemp_latched = True
            elif not servo.overtemp_latched:
                servo.overtemp_since = None
                return False
            elif sample.temperature_c <= clear_c:
                servo.overtemp_since = None
                servo.overtemp_latched = False
                if (
                    servo.message == "servo overtemperature"
                    and servo.target is None
                    and not servo.stop_pending
                ):
                    servo.state = "ready"
                    servo.message = "ready"
                return False
            if servo.stop_pending or (
                servo.target is None and servo.message == "servo overtemperature"
            ):
                return False
            self._request_stop(servo, "error", "servo overtemperature")
            return True

    def _follow_target(self, servo: _Channel, sample: _Sample) -> None:
        assert sample.target is not None
        direction = _move_direction(servo.config, sample.position, sample.target) if servo.calibrated else "manual"
        remaining = abs(sample.position - sample.target)
        with self.lock:
            if servo.blocked_direction == direction:
                return
        if remaining <= servo.config["position_tolerance_steps"]:
            with self.lock:
                self._request_stop(servo, "arrived", "reached target")
            self.wakeup.set()
            return
        if servo.sent_version != sample.version:
            self._send_target(servo, sample, direction)
            return
        self._watch_stall(servo, sample, direction)

    def _send_target(self, servo: _Channel, sample: _Sample, direction: str) -> None:
        assert sample.target is not None
        strength_key = {
            "manual": "calibration_grip_strength", "open": "open_grip_strength", "close": "close_grip_strength",
        }[direction]
        self._apply_strength(servo, strength_key)
        with self.lock:
            if servo.version != sample.version:
                return
            speed = servo.target_speed if servo.target_speed is not None else servo.config["speed"]
        servo.bus.set_position(
            servo.config["servo_id"],
            sample.target,
            speed,
            servo.config["accel"],
        )
        with self.lock:
            if servo.version != sample.version:
                return
            servo.sent_version = sample.version
            servo.motion_reference_position = sample.position
            servo.motion_reference_at = time.monotonic()
            servo.state = "moving"
            servo.message = "moving"

    def _apply_strength(self, servo: _Channel, key: str, strength: int | None = None) -> None:
        if strength is None:
            strength = _CALIBRATION_GRIP_STRENGTH if key == "calibration_grip_strength" else servo.config[key]
        if strength != servo.applied_strength:
            servo.bus.set_grip_strength(servo.config["servo_id"], strength)
            servo.applied_strength = strength
        servo.active_strength_key = key

    def _apply_pending_strength(self, servo: _Channel) -> None:
        with self.lock:
            pending = servo.pending_strength
        if pending is None:
            return
        key, strength = pending
        try:
            if key == servo.active_strength_key:
                self._apply_strength(servo, key, strength)
            with self.lock:
                servo.config[key] = strength
        except UmiError as exc:
            servo.strength_error = exc
        finally:
            with self.lock:
                servo.pending_strength = None
                servo.strength_applied.set()

    def _watch_stall(self, servo: _Channel, sample: _Sample, direction: str) -> None:
        now = time.monotonic()
        with self.lock:
            reference_position = servo.motion_reference_position
            reference_at = servo.motion_reference_at
            movement = servo.config["stall_movement_steps"]
            if (
                reference_position is None
                or reference_at is None
                or abs(sample.position - reference_position) >= movement
            ):
                servo.motion_reference_position = sample.position
                servo.motion_reference_at = now
                return
            window_key = "closing_stall_window_s" if direction == "close" else "opening_stall_window_s"
            window = servo.config[window_key]
            stalled = now - reference_at >= window
        if not stalled:
            return
        if direction == "close":
            self._hold_closed(servo)
            return
        with self.lock:
            self._request_stop(
                servo,
                "blocked",
                "opening blocked by object or hard stop",
                blocked_direction=direction,
            )
        self.wakeup.set()

    def _hold_closed(self, servo: _Channel) -> None:
        with self.lock:
            servo.blocked_direction = "close"
            servo.motion_reference_position = None
            servo.motion_reference_at = None
            servo.state = "grasped"
            servo.message = "holding closed grip"
            servo.stopped.set()

    def _request_stop(
        self,
        servo: _Channel,
        state: StateName,
        message: str,
        *,
        blocked_direction: str | None = None,
    ) -> None:
        servo.version += 1
        servo.target = None
        servo.target_speed = None
        if blocked_direction is not None:
            servo.blocked_direction = blocked_direction
        servo.motion_reference_position = None
        servo.motion_reference_at = None
        servo.sent_version = None
        servo.stop_pending = True
        servo.stop_version = servo.version
        servo.stop_result = (state, message, blocked_direction)
        servo.stopped.clear()
        servo.state = "moving"
        servo.message = "stopping"

    def _finish_stop(self, servo: _Channel, version: int) -> None:
        with self.lock:
            if not servo.stop_pending or servo.stop_version != version:
                return
            servo.stop_pending = False
            if servo.version == version:
                state, message, blocked = servo.stop_result
                servo.state = state
                servo.message = message
                servo.blocked_direction = blocked
            servo.stopped.set()

    def _complete_stop(self, servo: _Channel, sample: _Sample) -> None:
        servo.bus.set_position(
            servo.config["servo_id"],
            sample.position,
            servo.config["speed"],
            servo.config["accel"],
        )
        self._finish_stop(servo, sample.stop_version)

    def _clear_bus_fault(self, servo: _Channel) -> None:
        """A later good sample drops the timeout text and does not resume the old target."""
        servo.error_code = 0
        if not servo.bus_fault or servo.overtemp_latched or not servo.position_valid:
            return
        servo.bus_fault = False
        if servo.state == "error" and servo.target is None and not servo.stop_pending:
            servo.state = "ready"
            servo.message = "ready"

    def _record_bus_error(self, servo: _Channel, exc: UmiError) -> None:
        with self.lock:
            servo.error_code = exc.code or 0
            servo.bus_fault = True
            servo.target = None
            servo.stop_pending = False
            servo.stopped.set()
            servo.state = "error"
            servo.message = str(exc)
            if not servo.initial_ready.is_set():
                servo.initial_error = exc
                servo.initial_ready.set()


def _session_for(
    config_path: Path, *, worker_target: Callable | None = None, join_worker: Callable | None = None,
) -> _UmiSession:
    key = str(config_path)
    with _sessions_lock:
        session = _sessions.get(key)
        if session is None or session._closed:
            session = _UmiSession(config_path, worker_target=worker_target, join_worker=join_worker)
            _sessions[key] = session
        return session


class UmiGripper:
    """One UMI gripper channel borrowed from a shared serial session."""

    def __init__(self, session: _UmiSession, channel: int):
        self._session = session
        self._channel = channel
        self._closed = False
        self._closing = False
        self._close_lock = threading.Lock()

    width = staticmethod(_width)
    close_01 = staticmethod(_close_01)

    @classmethod
    def open(
        cls, *, side: str, options: dict[str, Any] | None = None,
        worker_target: Callable | None = None, join_worker: Callable | None = None,
    ) -> UmiGripper:
        options = dict(options or {})
        if "channel" not in options:
            raise UmiError(f"umi options for {side} require channel")
        channel = _side_channel(options["channel"])
        raw_path = options.get("config_path")
        if not isinstance(raw_path, str) or not raw_path.strip():
            raise UmiError(f"umi options for {side} require config_path")
        config_path = Path(raw_path).expanduser().resolve()
        # A side without its own calibration uses the populated slot.
        servos = _read_servos(config_path)
        calibrated_channel, raw = _calibrated_channel(
            servos, channel, side=side, config_path=config_path
        )
        config = _servo_config(raw, calibrated_channel, options=options)
        session = _session_for(config_path, worker_target=worker_target, join_worker=join_worker)
        device = cls(session, calibrated_channel)
        try:
            session.open_channel(calibrated_channel, config)
        except _OpeningCleanupError as exc:
            exc.resource = device
            raise
        return device

    def device_info(self) -> dict[str, Any]:
        with self._session.lock:
            config = dict(self._session.channels[self._channel].config)
        calibration = {key: config[key] for key in _CALIBRATION_KEYS}
        return {
            "channel": self._channel,
            "position_unit": "steps",
            "config_path": str(self._session.config_path),
            "calibration": calibration,
            "calibration_sha256": hashlib.sha256(
                json.dumps(calibration, sort_keys=True).encode()
            ).hexdigest(),
            "runtime": {key: config[key] for key in (*_RUNTIME_OPTION_KEYS, *_MOTION_OPTION_KEYS)},
        }

    def set_target(self, close_01: float) -> None:
        with self._close_lock:
            if self._closing:
                raise UmiError("UMI gripper is closing or closed")
            self._session.set_close_01(self._channel, close_01)

    request_target = set_target

    def read(self) -> UmiReading:
        return self._session.reading(self._channel)

    def stop(self) -> None:
        with self._close_lock:
            if not self._closing:
                self._session.stop_channel(self._channel)

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closing = True
            self._session.release_channel(self._channel)
            self._closed = True
            key = str(self._session.config_path)
            with _sessions_lock:
                if _sessions.get(key) is self._session and self._session._closed:
                    _sessions.pop(key)
