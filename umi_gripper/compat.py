"""Compatibility API for the original width-based gripper controller."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from .bus import STEPS_PER_TURN, UmiError
from .config import (
    DEFAULT_CONFIG_PATH,
    _invalid_position_message,
    _legacy_servos,
    _read_json,
    _width,
)
from .runtime import UmiGripper, _UmiSession

_STATUS_LABELS = {
    "initializing": "正在读取当前位置",
    "ready": "就绪",
    "moving": "运动中",
    "stopping": "停止中",
    "arrived": "已到位",
    "grasped": "已夹住",
    "blocked": "已阻塞",
    "error": "异常",
    "stopped": "已停止",
}


@dataclass(frozen=True)
class GripperReading:
    """Stable snapshot returned by :meth:`Gripper.read`."""

    channel: int
    status: str
    message: str
    width: float | None
    position_steps: int | None
    sampled_at: float
    current_a: float | None
    temperature_c: float | None
    effort: int | None
    error_code: int
    target_width: float | None
    blocked_direction: str | None


class Gripper:
    """Expose the original width API over the shared UMI runtime."""

    def __init__(
        self, config_path=DEFAULT_CONFIG_PATH, *, config=None, allow_uninitialized=False,
        bus_factory=None, initialize_calibration=False,
    ):
        self.config_path = Path(config_path)
        if config is None:
            config = _read_json(self.config_path)
        servos = _legacy_servos(config, calibrated=not allow_uninitialized, validate_existing=True)
        self.config = {"servos": servos}
        self._session = _UmiSession(self.config_path, bus_factory=bus_factory)
        self._devices: dict[int, UmiGripper] = {}
        self._channels = tuple(i for i, item in enumerate(self.config["servos"]) if item is not None)
        try:
            for index in self._channels:
                item = servos[index]
                self._devices[index] = UmiGripper(self._session, index)
                self._session.open_channel(
                    index, item, calibrated=item.get("open_position_steps") is not None,
                    initialize_calibration=initialize_calibration,
                )
        except Exception as opening_error:
            try:
                self.disconnect()
            except Exception as cleanup_error:
                raise cleanup_error from opening_error
            raise
        self.worker = self._session.worker

    @property
    def channels(self):
        return self._session.channels

    @property
    def buses(self):
        return self._session.buses

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disconnect()

    def disconnect(self):
        """Release every channel, retaining any device that needs another close."""
        failures = []
        for index, device in tuple(self._devices.items()):
            try:
                device.close()
            except Exception as exc:
                failures.append(exc)
            else:
                del self._devices[index]
        if failures:
            message = "UMI cleanup incomplete: " + "; ".join(str(exc) for exc in failures)
            raise UmiError(message, resource=self) from failures[0]

    close = disconnect

    def _channel(self, channel):
        if channel is None:
            if len(self._channels) != 1:
                raise UmiError("已配置多个舵机，请明确指定通道号 0 或 1。")
            channel = self._channels[0]
        if isinstance(channel, bool) or channel not in self._devices:
            raise UmiError(f"舵机通道 {channel!r} 未配置或已关闭。")
        return self.channels[channel]

    def get_state(self, channel=None):
        with self._session.lock:
            item = self._channel(channel)
            telemetry = dict(item.telemetry) if item.telemetry is not None else None
            target_width = _width(item.config, item.target) if item.calibrated and item.target is not None else None
            status = "stopping" if item.stop_pending else item.state
            if not item.initial_ready.is_set():
                status = "initializing"
            return {
                "state": _STATUS_LABELS[status],
                "status": status,
                "blocked_direction": {"open": "张开", "close": "闭合"}.get(item.blocked_direction, item.blocked_direction),
                "target_width": target_width,
                "width": _width(item.config, item.position) if item.calibrated and item.position_valid else None,
                "position_raw": item.position,
                "position_unit": "steps",
                "sampled_at": item.sampled_at_mono_s,
                "error_code": item.error_code,
                "telemetry": telemetry,
                "message": item.message,
            }

    def read(self, channel=None) -> GripperReading:
        item = self._channel(channel)
        state = self.get_state(item.index)
        telemetry = state["telemetry"] or {}
        current = telemetry.get("current_ma")
        return GripperReading(
            channel=item.index,
            status=state["status"],
            message=state["message"],
            width=state["width"],
            position_steps=state["position_raw"],
            sampled_at=state["sampled_at"],
            current_a=current / 1000.0 if current is not None else None,
            temperature_c=float(telemetry["temperature_c"]) if "temperature_c" in telemetry else None,
            effort=telemetry.get("load"),
            error_code=state["error_code"],
            target_width=state["target_width"],
            blocked_direction=state["blocked_direction"],
        )

    def get_width(self, channel=None):
        with self._session.lock:
            item = self._channel(channel)
            if not item.calibrated:
                raise UmiError(f"{item.index} 号舵机尚未完成标定。")
            if not item.position_valid:
                raise UmiError(_invalid_position_message(item.index, item.position, item.config, detail=item.message))
            return _width(item.config, item.position)

    def goto(self, width, channel=None):
        if isinstance(width, bool) or not isinstance(width, (int, float)) or not 0 <= width <= 1:
            raise ValueError("goto() 的取值必须在 0 到 1 之间。")
        self._devices[self._channel(channel).index].set_target(1.0 - width)

    def open_gripper(self, channel=None):
        self.goto(1.0, channel)

    def close_gripper(self, channel=None):
        self.goto(0.0, channel)

    def stop(self, channel=None):
        item = self._channel(channel)
        with self._session.lock:
            if not item.position_valid:
                item.target = item.target_speed = None
                item.stop_pending = False
                item.stopped.set()
                return self.read(item.index)
        if not self._session.stop_channel(item.index, timeout=1.0):
            raise UmiError(f"{item.index} 号舵机停止超时。")
        return self.read(item.index)

    def wait(self, timeout=None, channel=None):
        item = self._channel(channel)
        deadline = time.monotonic() + (timeout if timeout is not None else item.config["motion_timeout_s"])
        while True:
            state = self.get_state(item.index)
            if state["status"] not in {"initializing", "moving", "stopping"}:
                return state
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise UmiError(f"{item.index} 号舵机等待动作完成超时。")
            time.sleep(min(0.02, remaining))

    def reset(self, channel=None):
        item = self._channel(channel)
        if not item.calibrated:
            raise UmiError(f"{item.index} 号舵机尚未完成标定。")
        self._session.set_position_steps(item.index, item.config["neutral_position_steps"])
        state = self.wait(channel=item.index)
        if state["status"] != "arrived":
            raise UmiError("复位失败：" + state["message"])

    def manual_step(self, direction, channel=None):
        if direction not in {-1, 1}:
            raise ValueError("手动方向只能为 -1 或 1。")
        item = self._channel(channel)
        if item.position is None:
            raise UmiError(f"{item.index} 号舵机尚未读到当前位置。")
        target = max(0, min(STEPS_PER_TURN - 1, item.position + direction * item.config["calibration_step_steps"]))
        self._session.set_position_steps(item.index, target, speed=min(item.config["speed"], 600))

    def set_calibration(self, channel, config):
        self._session.update_calibration(self._channel(channel).index, config)

    def set_grip_strength(self, strength, channel=None, *, direction="闭合"):
        directions = {"张开": "open", "open": "open", "闭合": "close", "close": "close"}
        if direction not in directions:
            raise ValueError("力度方向只能为 open/close 或 张开/闭合。")
        if isinstance(strength, bool) or not isinstance(strength, int) or not 1 <= strength <= 100:
            raise ValueError("夹紧力度必须是 1 到 100 的整数。")
        channel = self._channel(channel).index
        key = f"{directions[direction]}_grip_strength"
        self._session.set_grip_strength(channel, key, strength)

    def calibrate_position(self, name, channel=None):
        from .cli.interaction import calibrate_position

        return calibrate_position(self, name, self._channel(channel).index)

    def guided_grip_test(self, channel=None):
        from .cli.interaction import guided_grip_test

        return guided_grip_test(self, self._channel(channel).index)
