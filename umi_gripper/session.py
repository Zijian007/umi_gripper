"""Own the serial port, worker and motion state shared by its servos."""

from __future__ import annotations

import logging
import os
import threading
import time
from collections.abc import Mapping
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Literal

from .bus import STEPS_PER_TURN, ServoBus, ServoBusPort, UmiError
from .config import (
    _calibration_points,
    _checked_int,
    _close_01,
    _invalid_position_message,
    _move_direction,
    _position_in_span,
    _require_close_01,
    _steps_for_close_01,
)

_LOGGER = logging.getLogger(__name__)

INITIAL_POSITION_TIMEOUT_S = 2.0
_CALIBRATION_GRIP_STRENGTH = 40
_STRENGTH_UPDATE_TIMEOUT_S = 2.0
StateName = Literal["ready", "moving", "arrived", "grasped", "blocked", "error"]
FaultName = Literal["communication", "position", "overtemperature"]

_sessions: dict[str, "_UmiSession"] = {}
_maintenance_ports: set[str] = set()
_sessions_lock = threading.Lock()


@dataclass(frozen=True)
class _PendingStop:
    version: int
    state: StateName
    message: str
    blocked_direction: str | None


@dataclass(frozen=True)
class _Sample:
    """One worker tick. Motion decisions re-check ``version`` under the lock."""

    position: int
    temperature_c: int
    target: int | None
    version: int
    pending_stop: _PendingStop | None


@dataclass
class _StrengthRequest:
    key: str
    strength: int
    done: threading.Event = field(default_factory=threading.Event)
    error: Exception | None = None


@dataclass
class _Channel:
    index: int
    servo_id: int
    config: dict[str, Any]
    calibrated: bool = True
    position: int | None = None
    position_valid: bool = False
    requested_target: int | None = None
    target: int | None = None
    target_speed: int | None = None
    version: int = 0
    blocked_direction: str | None = None
    motion_reference_position: int | None = None
    motion_reference_at: float | None = None
    sent_version: int | None = None
    pending_stop: _PendingStop | None = None
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
    pending_strength: _StrengthRequest | None = None
    closing: bool = False
    overtemp_since: float | None = None
    overtemp_latched: bool = False
    bus_fault: bool = False
    error_code: int | None = None


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
    feedback_valid: bool = True
    fault: FaultName | None = None
    fault_code: int | None = None


class _OpeningCleanupError(UmiError):
    """Opening failed and the caller still owns a resource that requires close."""

    def __init__(self, message: str, session: _UmiSession):
        super().__init__(message)
        self.session = session


class _SessionClosed(UmiError):
    """A new open raced with retirement of the previous port session."""


class _PortBusy(UmiError):
    """The port already has a controller or maintenance owner."""


class _UmiSession:
    """Serialize commands for the servos attached to one port."""

    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self._lifecycle_lock = threading.Lock()
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.wakeup = threading.Event()
        self.bus: ServoBusPort | None = None
        self.channels: dict[int, _Channel] = {}
        self.worker: threading.Thread | None = None
        self._closed = False

    def open_channel(
        self, channel: int, config: dict[str, Any], *, calibrating: bool = False,
    ) -> None:
        with self._lifecycle_lock:
            if self._closed:
                raise _SessionClosed("umi session is closed")
            if self.stop_event.is_set():
                raise UmiError("umi session cleanup is pending")
            if config["baudrate"] != self.baudrate:
                raise UmiError(f"UMI port {self.port} is already open at {self.baudrate} baud")
            self._open_channel(channel, config, calibrating)

    def _open_channel(
        self, channel: int, config: dict[str, Any], calibrating: bool,
    ) -> None:
        sid = config["servo_id"]
        if sid in self.channels:
            raise UmiError(f"UMI servo ID {sid} on {self.port} is already open")
        strength_key = "calibration_grip_strength" if calibrating else "close_grip_strength"
        strength = _CALIBRATION_GRIP_STRENGTH if calibrating else config[strength_key]
        try:
            if self.bus is None:
                self.bus = ServoBus(self.port, self.baudrate)
                self.bus.open()
            self.bus.verify(sid)
            if calibrating:
                self.bus.configure_position_mode(sid)
                self.bus.set_current_position_as_middle(sid)
            self.bus.check_position_mode(sid)
            self.bus.prepare(sid, strength)
            servo = _Channel(
                index=channel,
                servo_id=sid,
                config=config,
                calibrated=not calibrating,
                applied_strength=strength,
                active_strength_key=strength_key,
                state="moving",
                message="reading initial position",
            )
            servo.stopped.set()
            with self.lock:
                self.channels[sid] = servo
            if self.worker is None or not self.worker.is_alive():
                self.worker = threading.Thread(target=self._run, name="umi-gripper", daemon=True)
                self.worker.start()
            self.wakeup.set()
            self._wait_initial(servo)
        except Exception:
            self._rollback_open(sid)
            raise

    def _rollback_open(self, servo_id: int) -> None:
        try:
            self._release_channel(servo_id)
        except Exception as exc:
            raise _OpeningCleanupError(f"UMI open cleanup incomplete: {exc}", self) from exc

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

    def set_close_01(self, servo_id: int, close_01: float, *, retry: bool = False) -> None:
        close_01 = _require_close_01(close_01)
        with self.lock:
            servo = self._require_channel(servo_id)
            if not servo.calibrated:
                raise UmiError(f"umi channel {servo.index} is not calibrated")
            if servo.bus_fault:
                raise UmiError(servo.message, code=servo.error_code)
            if servo.overtemp_latched:
                raise UmiError("servo overtemperature")
            if servo.position is None or not servo.position_valid:
                raise UmiError(_invalid_position_message(
                    servo.index, servo.position, servo.config, detail=servo.message
                ))
            target = _steps_for_close_01(servo.config, close_01)
            if servo.requested_target == target and (servo.state == "moving" or not retry):
                return
            self._queue_move(servo, target)
            servo.requested_target = target
        self.wakeup.set()

    def set_position_steps(self, servo_id: int, position: int, *, speed: int | None = None) -> None:
        """Queue an explicit position for calibration or neutral positioning."""
        _checked_int(position, "position", 0, STEPS_PER_TURN - 1)
        if speed is not None:
            _checked_int(speed, "speed", 1, 3400)
        with self.lock:
            servo = self._require_channel(servo_id)
            if servo.position is None or not servo.position_valid:
                raise UmiError(f"umi channel {servo.index} has no valid position")
            if servo.calibrated and not _position_in_span(servo.config, position):
                raise UmiError(f"umi channel {servo.index} target is outside calibration")
            self._queue_move(servo, position, speed=speed)
        self.wakeup.set()

    def update_calibration(self, servo_id: int, config: Mapping[str, Any]) -> None:
        with self.lock:
            servo = self._require_channel(servo_id)
            points = _calibration_points(config, servo.index)
            servo.config.update(points)
            servo.calibrated = True
            servo.requested_target = None

    def set_grip_strength(self, servo_id: int, key: str, strength: int) -> None:
        request = _StrengthRequest(key, strength)
        with self._lifecycle_lock:
            with self.lock:
                servo = self._require_channel(servo_id)
                servo.pending_strength = request
            self.wakeup.set()
            if not request.done.wait(_STRENGTH_UPDATE_TIMEOUT_S):
                with self.lock:
                    if servo.pending_strength is request:
                        servo.pending_strength = None
                        raise UmiError("updating UMI grip strength timed out before the write started")
                if not request.done.is_set():
                    raise UmiError("updating UMI grip strength timed out; the write may still complete")
            if request.error is not None:
                raise request.error

    @staticmethod
    def _set_motion_target(servo: _Channel, target: int | None, *, speed: int | None = None) -> None:
        servo.target = target
        servo.target_speed = speed
        servo.motion_reference_position = None
        servo.motion_reference_at = None
        servo.sent_version = None

    def _queue_move(self, servo: _Channel, target: int, *, speed: int | None = None) -> None:
        """Arm a move. Leave an in-flight stop pending so the worker can finish it."""
        servo.requested_target = None
        servo.blocked_direction = None
        servo.version += 1
        self._set_motion_target(servo, target, speed=speed)
        servo.state = "moving"
        servo.message = "moving"

    def stop_channel(self, servo_id: int, *, timeout: float = 0.5, allow_invalid: bool = False) -> bool:
        with self.lock:
            servo = self._require_channel(servo_id)
            if not servo.position_valid:
                if allow_invalid:
                    return True
                raise UmiError(servo.message, code=servo.error_code)
            servo.requested_target = None
            self._request_stop(servo, "ready", "stopped at current position")
        self.wakeup.set()
        if not servo.stopped.wait(timeout):
            return False
        with self.lock:
            if servo.bus_fault or not servo.position_valid:
                raise UmiError(servo.message, code=servo.error_code)
        return True

    def release_channel(self, servo_id: int) -> None:
        with self._lifecycle_lock:
            self._release_channel(servo_id)

    def _release_channel(self, servo_id: int) -> None:
        if self.stop_event.is_set():
            self._finish_close()
            return
        with self.lock:
            servo = self.channels.get(servo_id)
            if servo is not None and not servo.closing:
                servo.closing = True
                self._request_stop(servo, "ready", "channel closed")
        if servo is not None:
            self.wakeup.set()
            if not servo.stopped.wait(0.5) and len(self.channels) > 1:
                raise UmiError(f"UMI servo ID {servo_id} did not stop; retry close")
        with self.lock:
            self.channels.pop(servo_id, None)
            if self.channels:
                return
        self.stop_event.set()
        self.wakeup.set()
        self._finish_close()

    def _finish_close(self) -> None:
        """Release only resources that have finished closing, so failed closes are retryable."""
        if self._closed:
            return
        if self.worker is not None and not self._join_worker(self.worker):
            raise UmiError("UMI worker did not stop; retry close after it exits")
        self.worker = None
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        with _sessions_lock:
            self._closed = True
            if _sessions.get(self.port) is self:
                del _sessions[self.port]

    @staticmethod
    def _join_worker(worker: threading.Thread) -> bool:
        if worker.ident is None:
            return True
        if worker is threading.current_thread():
            return False
        worker.join(timeout=2.0)
        if not worker.is_alive():
            return True
        _LOGGER.error("UMI worker did not stop within 2 s; keeping its serial ports open")
        return False

    def _require_channel(self, servo_id: int) -> _Channel:
        servo = self.channels.get(servo_id)
        if servo is None:
            raise UmiError(f"UMI servo ID {servo_id} is not open")
        return servo

    def _require_bus(self) -> ServoBusPort:
        bus = self.bus
        if bus is None:
            raise UmiError(f"UMI port {self.port} is not open")
        return bus

    def config_snapshot(self, servo_id: int) -> dict[str, Any]:
        with self.lock:
            return dict(self._require_channel(servo_id).config)

    def reading(self, servo_id: int) -> UmiReading:
        with self.lock:
            servo = self._require_channel(servo_id)
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
            fault: FaultName | None = None
            if servo.bus_fault:
                fault = "communication"
            elif not servo.position_valid:
                fault = "position"
            elif servo.overtemp_latched:
                fault = "overtemperature"
            return UmiReading(
                close_01=close_01,
                sampled_at_mono_s=servo.sampled_at_mono_s,
                state=servo.state,
                message=servo.message,
                current=current_a,
                temperature=temperature,
                effort=effort,
                raw_position=float(servo.position) if servo.position is not None else None,
                feedback_valid=servo.position_valid,
                fault=fault,
                fault_code=servo.error_code,
            )

    def _run(self) -> None:
        while not self.stop_event.is_set():
            now = time.monotonic()
            next_due = now + 0.05
            with self.lock:
                servos = list(self.channels.values())
            for servo in servos:
                self._apply_pending_strength(servo)
                with self.lock:
                    needs_poll = servo.pending_stop is not None or now >= servo.next_status_poll_at
                if needs_poll:
                    self._update(servo)
                    servo.next_status_poll_at = now + 1.0 / servo.config["status_frequency_hz"]
                next_due = min(next_due, servo.next_status_poll_at)
            self.wakeup.wait(max(0.001, next_due - time.monotonic()))
            self.wakeup.clear()

    def _update(self, servo: _Channel) -> None:
        try:
            with self.lock:
                if servo.closing and servo.pending_stop is None:
                    return
            sample = self._ingest(servo)
            if sample is None:
                return
            if sample.pending_stop is not None:
                self._complete_stop(servo, sample.position, sample.pending_stop)
                return
            if self._temperature_fault(servo, sample):
                self.wakeup.set()
                return
            if sample.target is not None:
                self._follow_target(servo, sample)
        except UmiError as exc:
            self._record_bus_error(servo, exc)

    def _ingest(self, servo: _Channel) -> _Sample | None:
        telemetry = self._require_bus().telemetry(servo.servo_id)
        now = time.monotonic()
        with self.lock:
            was_valid = servo.position_valid
            servo.position = telemetry["raw_position"]
            servo.position_valid = not servo.calibrated or _position_in_span(servo.config, servo.position)
            servo.telemetry = telemetry
            servo.sampled_at_mono_s = now
            servo.bus_fault = False
            servo.error_code = None
            if not servo.position_valid:
                self._abort_motion(servo)
                servo.state = "error"
                servo.message = _invalid_position_message(
                    servo.index, servo.position, servo.config, detail=servo.message
                )
            elif not was_valid and servo.state == "error":
                if servo.overtemp_latched:
                    servo.message = "servo overtemperature"
                else:
                    servo.state = "ready"
                    servo.message = "ready"
            self._note_initial(servo)
            if not servo.position_valid:
                return None
            return _Sample(
                position=servo.position,
                temperature_c=telemetry["temperature_c"],
                target=servo.target,
                version=servo.version,
                pending_stop=servo.pending_stop,
            )

    def _note_initial(self, servo: _Channel) -> None:
        if servo.initial_ready.is_set():
            return
        if servo.position_valid:
            servo.state = "ready"
            servo.message = "ready"
        else:
            servo.initial_error = UmiError(servo.message)
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
            if servo.closing:
                return False
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
                    and servo.pending_stop is None
                ):
                    servo.state = "ready"
                    servo.message = "ready"
                return False
            if servo.pending_stop is not None or (
                servo.target is None and servo.message == "servo overtemperature"
            ):
                return False
            servo.requested_target = None
            self._request_stop(servo, "error", "servo overtemperature")
            return True

    def _follow_target(self, servo: _Channel, sample: _Sample) -> None:
        assert sample.target is not None
        with self.lock:
            if servo.version != sample.version:
                return
            direction = _move_direction(servo.config, sample.position, sample.target) if servo.calibrated else "manual"
            if servo.blocked_direction == direction:
                return
            if abs(sample.position - sample.target) <= servo.config["position_tolerance_steps"]:
                self._request_stop(servo, "arrived", "reached target")
                self.wakeup.set()
                return
            needs_send = servo.sent_version != sample.version
        if needs_send:
            self._send_target(servo, sample, direction)
        else:
            self._watch_stall(servo, sample, direction)

    def _send_target(self, servo: _Channel, sample: _Sample, direction: str) -> None:
        assert sample.target is not None
        with self.lock:
            if servo.version != sample.version:
                return
        strength_key = {
            "manual": "calibration_grip_strength", "open": "open_grip_strength", "close": "close_grip_strength",
        }[direction]
        self._apply_strength(servo, strength_key)
        with self.lock:
            if servo.version != sample.version:
                return
            speed = servo.target_speed if servo.target_speed is not None else servo.config["speed"]
        self._require_bus().set_position(
            servo.servo_id,
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
            self._require_bus().set_grip_strength(servo.servo_id, strength)
            servo.applied_strength = strength
        servo.active_strength_key = key

    def _apply_pending_strength(self, servo: _Channel) -> None:
        with self.lock:
            # Claim the request before any bus I/O so a timeout can only cancel queued work.
            request = servo.pending_strength
            servo.pending_strength = None
        if request is None:
            return
        key, strength = request.key, request.strength
        try:
            if key == servo.active_strength_key:
                self._apply_strength(servo, key, strength)
            with self.lock:
                servo.config[key] = strength
        except Exception as exc:
            request.error = exc
        finally:
            request.done.set()

    def _watch_stall(self, servo: _Channel, sample: _Sample, direction: str) -> None:
        now = time.monotonic()
        with self.lock:
            if servo.version != sample.version:
                return
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
            if now - reference_at < servo.config[window_key]:
                return
            if direction == "close":
                servo.blocked_direction = "close"
                servo.motion_reference_position = None
                servo.motion_reference_at = None
                servo.state = "grasped"
                servo.message = "holding closed grip"
                servo.stopped.set()
                return
            self._request_stop(
                servo,
                "blocked",
                "opening blocked by object or hard stop",
                blocked_direction=direction,
            )
        self.wakeup.set()

    def _abort_motion(self, servo: _Channel) -> None:
        self._set_motion_target(servo, None)
        servo.requested_target = None
        servo.blocked_direction = None
        servo.pending_stop = None
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
        self._set_motion_target(servo, None)
        if blocked_direction is not None:
            servo.blocked_direction = blocked_direction
        servo.pending_stop = _PendingStop(servo.version, state, message, blocked_direction)
        servo.stopped.clear()
        servo.state = "moving"
        servo.message = "stopping"

    def _complete_stop(self, servo: _Channel, position: int, stop: _PendingStop) -> None:
        self._require_bus().set_position(
            servo.servo_id,
            position,
            servo.config["speed"],
            servo.config["accel"],
        )
        with self.lock:
            if servo.pending_stop is not stop:
                return
            servo.pending_stop = None
            # A target queued during the stop write keeps its own state.
            if servo.version == stop.version:
                servo.state = stop.state
                servo.message = stop.message
                servo.blocked_direction = stop.blocked_direction
            servo.stopped.set()

    def _record_bus_error(self, servo: _Channel, exc: UmiError) -> None:
        with self.lock:
            servo.error_code = exc.code
            servo.bus_fault = True
            servo.position_valid = False
            self._abort_motion(servo)
            servo.state = "error"
            servo.message = str(exc)
            if not servo.initial_ready.is_set():
                servo.initial_error = exc
                servo.initial_ready.set()


def _port_key(port: str) -> str:
    if os.name == "nt" and not os.path.isabs(port):
        return os.path.normcase(port)
    return os.path.normcase(os.path.realpath(port))


@contextmanager
def _maintenance_port(port: str):
    key = _port_key(port)
    with _sessions_lock:
        if key in _sessions or key in _maintenance_ports:
            raise _PortBusy(f"UMI port {port} is already in use")
        _maintenance_ports.add(key)
    try:
        yield key
    finally:
        with _sessions_lock:
            _maintenance_ports.remove(key)


def _open_servo(
    channel: int, config: dict[str, Any], *, calibrating: bool = False,
) -> _UmiSession:
    port = config["serial_port"]
    key = _port_key(port)
    while True:
        with _sessions_lock:
            if key in _maintenance_ports:
                raise _PortBusy(f"UMI port {port} is in maintenance use")
            session = _sessions.get(key)
            if session is None:
                session = _UmiSession(key, config["baudrate"])
                _sessions[key] = session
        try:
            session.open_channel(channel, config, calibrating=calibrating)
        except _SessionClosed:
            continue
        return session
