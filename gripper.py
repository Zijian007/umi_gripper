"""支持一至两个独立通道的总线舵机夹爪控制。"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Protocol


STEPS_PER_TURN = 4096
INITIAL_POSITION_TIMEOUT_S = 2.0
DEFAULT_BAUDRATES = (1_000_000, 500_000, 250_000, 115_200, 57_600, 38_400)
DEFAULT_CONFIG_PATH = Path(__file__).with_name("gripper_config.json")

DEFAULT_MOTION_OPTIONS = {
    "accel": 50,
    "position_tolerance_steps": 12,
    "stall_movement_steps": 3,
    "opening_stall_window_s": 0.5,
    "closing_stall_window_s": 0.2,
    "overtemp_c": 70,
    "overtemp_clear_c": 65,
    "overtemp_hold_s": 0.5,
}

_FEEDBACK_START = 56
_FEEDBACK_LENGTH = 15


class GripperError(RuntimeError):
    def __init__(self, message: str, *, code: int | None = None):
        super().__init__(message)
        self.code = code


class ConfigError(GripperError):
    pass


def _word(data: list[int], offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def _signed(value: int, bit: int) -> int:
    return -(value & ~(1 << bit)) if value & (1 << bit) else value


def decode_status_block(data: list[int]) -> dict:
    """Decode one contiguous STS status packet from address 56 to 70."""
    if len(data) < _FEEDBACK_LENGTH:
        raise GripperError(
            f"读取反馈失败：状态包长度不足（{len(data)} 字节）"
        )
    position = _signed(_word(data, 0), 15) % STEPS_PER_TURN
    load = _signed(_word(data, 4), 10)
    current = _signed(_word(data, 13), 15)
    return {
        "raw_position": position,
        "load": load,
        "current_ma": round(current * 6.5),
        "temperature_c": data[7],
    }


def _read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConfigError(f"找不到配置文件：{path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(f"配置文件不是有效的 JSON：{exc}") from exc


def save_config(config: dict, path: str | Path = DEFAULT_CONFIG_PATH) -> None:
    saved = dict(config) if "servos" in config else {}
    saved["servos"] = [
        {key: value for key, value in item.items() if key != "calibration_grip_strength"}
        if item is not None
        else None
        for item in _servos(config)
    ]
    Path(path).write_text(
        json.dumps(saved, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def default_servo_config(selected: dict | None = None) -> dict:
    selected = selected or {}
    return {
        "serial_port": selected.get("port"),
        "servo_id": selected.get("servo_id"),
        "baudrate": selected.get("baudrate"),
        "servo_model_number": selected.get("model"),
        "closed_position_steps": None,
        "neutral_position_steps": None,
        "open_position_steps": None,
        "speed": 1200,
        "calibration_grip_strength": 40,
        "open_grip_strength": 40,
        "close_grip_strength": 40,
        "status_frequency_hz": 50,
        "calibration_step_steps": 128,
        "motion_timeout_s": 30,
        **DEFAULT_MOTION_OPTIONS,
    }


def _integer(config: dict, key: str, low: int, high: int) -> int:
    value = config.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not low <= value <= high
    ):
        raise ConfigError(f"配置项“{key}”必须是 {low} 到 {high} 的整数。")
    return value


def _servos(config: dict) -> list[dict | None]:
    if "servos" not in config:
        value = [config, None]
    else:
        value = config["servos"]
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(x is not None and not isinstance(x, dict) for x in value)
    ):
        raise ConfigError("配置项“servos”必须是包含两个配置对象或 null 的列表。")
    servos = []
    for item in value:
        if item is None:
            servos.append(None)
            continue
        item = dict(item)
        old_strength = item.pop("grip_strength", None)
        if old_strength is not None:
            item.setdefault("open_grip_strength", old_strength)
            item.setdefault("close_grip_strength", old_strength)
        for key, value in DEFAULT_MOTION_OPTIONS.items():
            item.setdefault(key, value)
        # 标定力度只取代码中的默认值，不接受 JSON 中的同名设置。
        item["calibration_grip_strength"] = default_servo_config()[
            "calibration_grip_strength"
        ]
        servos.append(item)
    return servos


def _validate_servo(config: dict, channel: int, calibrated: bool) -> None:
    required = (
        "serial_port",
        "servo_id",
        "baudrate",
        "speed",
        "open_grip_strength",
        "close_grip_strength",
        "status_frequency_hz",
        "calibration_step_steps",
        "motion_timeout_s",
    )
    missing = [key for key in required if config.get(key) in (None, "")]
    if missing:
        raise ConfigError(f"{channel} 号舵机配置尚未完成：" + "、".join(missing))
    if not isinstance(config["serial_port"], str):
        raise ConfigError(f"{channel} 号舵机串口无效。")
    _integer(config, "servo_id", 0, 253)
    _integer(config, "baudrate", 9600, 1_000_000)
    _integer(config, "speed", 1, 3400)
    _integer(config, "calibration_grip_strength", 1, 100)
    _integer(config, "open_grip_strength", 1, 100)
    _integer(config, "close_grip_strength", 1, 100)
    _integer(config, "status_frequency_hz", 1, 100)
    _integer(config, "calibration_step_steps", 10, 1024)
    _integer(config, "motion_timeout_s", 2, 120)
    _integer(config, "accel", 0, 254)
    _integer(config, "position_tolerance_steps", 0, 128)
    _integer(config, "stall_movement_steps", 1, 128)
    _integer(config, "overtemp_c", 1, 100)
    _integer(config, "overtemp_clear_c", 0, 99)
    if config["overtemp_clear_c"] >= config["overtemp_c"]:
        raise ConfigError("过温解除温度必须低于过温触发温度。")
    for key in ("opening_stall_window_s", "closing_stall_window_s"):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"配置项“{key}”必须是 0.05 到 5 秒的数字。")
        if not 0.05 <= float(value) <= 5.0:
            raise ConfigError(f"配置项“{key}”必须是 0.05 到 5 秒的数字。")
    hold = config["overtemp_hold_s"]
    if isinstance(hold, bool) or not isinstance(hold, (int, float)):
        raise ConfigError("配置项“overtemp_hold_s”必须是 0 到 5 秒的数字。")
    if not 0.0 <= float(hold) <= 5.0:
        raise ConfigError("配置项“overtemp_hold_s”必须是 0 到 5 秒的数字。")
    if not calibrated:
        return
    for key in (
        "closed_position_steps",
        "neutral_position_steps",
        "open_position_steps",
    ):
        _integer(config, key, 0, STEPS_PER_TURN - 1)
    opened, neutral = config["open_position_steps"], config["neutral_position_steps"]
    span = opened - config["closed_position_steps"]
    neutral_delta = neutral - config["closed_position_steps"]
    if (
        not 10 <= abs(span) < STEPS_PER_TURN // 2
        or neutral_delta * span < 0
        or abs(neutral_delta) > abs(span)
    ):
        raise ConfigError(f"{channel} 号舵机标定位置不合理。")


def validate_config(config: dict, *, calibrated: bool = True) -> None:
    servos = _servos(config)
    active = [(i, x) for i, x in enumerate(servos) if x is not None]
    if not active:
        raise ConfigError("至少需要配置一个舵机。")
    seen, ports = set(), {}
    for index, item in active:
        _validate_servo(item, index, calibrated)
        identity = (item["serial_port"], item["servo_id"])
        if identity in seen:
            raise ConfigError("同一串口上的两个舵机 ID 不能相同。")
        seen.add(identity)
        if (
            item["serial_port"] in ports
            and ports[item["serial_port"]] != item["baudrate"]
        ):
            raise ConfigError("同一串口上的两个舵机必须使用相同波特率。")
        ports[item["serial_port"]] = item["baudrate"]


class ServoBusPort(Protocol):
    """Controller-facing bus contract; successful reads provide explicit position/status feedback."""

    def open(self) -> None: ...
    def close(self) -> None: ...
    def verify(self, sid: int) -> int: ...
    def check_position_mode(self, sid: int) -> None: ...
    def prepare(self, sid: int, strength: int) -> None: ...
    def set_grip_strength(self, sid: int, strength: int) -> None: ...
    def set_position(self, sid: int, position: int, speed: int, accel: int = 50) -> None: ...
    def telemetry(self, sid: int) -> dict: ...


class ServoBus:
    """一个物理串口，可供多个不同 ID 的舵机使用。"""

    def __init__(self, port: str, baudrate: int):
        import scservo_sdk

        self.sdk = scservo_sdk
        self.port_name, self.baudrate = port, baudrate
        self.port, self.packet, self.opened = self.sdk.PortHandler(port), None, False
        self._io_lock = threading.RLock()

    @staticmethod
    def _check(result: int, error: int, action: str) -> None:
        from scservo_sdk import COMM_SUCCESS

        if result != COMM_SUCCESS:
            raise GripperError(
                f"{action}失败：串口通信异常（代码 {result}）。", code=result
            )
        if error:
            raise GripperError(
                f"{action}失败：舵机报告异常（代码 {error}）。", code=error
            )

    def open(self) -> None:
        with self._io_lock:
            try:
                self.opened = self.port.openPort()
            except Exception as exc:
                raise GripperError(f"无法打开串口 {self.port_name}：{exc}") from exc
            if not self.opened:
                raise GripperError(f"无法打开串口 {self.port_name}。")
            if not self.port.setBaudRate(self.baudrate):
                self.close()
                raise GripperError(f"无法设置串口速度 {self.baudrate}。")
            self.packet = self.sdk.sms_sts(self.port)

    def close(self) -> None:
        with self._io_lock:
            if self.opened:
                self.opened = False
                self.port.closePort()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def _read1(self, sid, address, action):
        with self._io_lock:
            value, result, error = self.packet.read1ByteTxRx(sid, address)
        self._check(result, error, action)
        return value

    def _read2(self, sid, address, action):
        with self._io_lock:
            value, result, error = self.packet.read2ByteTxRx(sid, address)
        self._check(result, error, action)
        return value

    def _write1(self, sid, address, value, action):
        with self._io_lock:
            result, error = self.packet.write1ByteTxRx(sid, address, value)
        self._check(result, error, action)

    def _write2(self, sid, address, value, action):
        with self._io_lock:
            result, error = self.packet.write2ByteTxRx(sid, address, value)
        self._check(result, error, action)

    def verify(self, sid: int) -> int:
        with self._io_lock:
            model, result, error = self.packet.ping(sid)
        self._check(result, error, f"确认 ID {sid} 舵机")
        return model

    def configure_position_mode(self, sid: int) -> None:
        self._write1(sid, self.sdk.SMS_STS_TORQUE_ENABLE, 1, "开启扭矩")
        self._write1(sid, 55, 0, "解锁舵机设置")
        self._write2(sid, 9, 0, "设置最小角度")
        self._write2(sid, 11, STEPS_PER_TURN - 1, "设置最大角度")
        self._write1(sid, 33, 0, "切换绝对位置模式")
        self._write1(sid, 55, 1, "锁定舵机设置")

    def set_current_position_as_middle(self, sid: int) -> None:
        """使用舵机的置中指令，将当前安装角度设为 2048。"""
        self._write1(sid, self.sdk.SMS_STS_TORQUE_ENABLE, 128, "将当前位置设为中位")
        time.sleep(0.05)
        with self._io_lock:
            position, result, error = self.packet.ReadPos(sid)
        self._check(result, error, "确认中位设置")
        if not 2044 <= position <= 2052:
            raise GripperError("舵机未能将当前位置设为中位，请检查舵机型号和通信。")

    def check_position_mode(self, sid: int) -> None:
        if self._read1(sid, 33, "读取舵机模式") != 0:
            raise GripperError(
                f"ID {sid} 舵机未处于绝对位置模式，请先运行 gripper_cal.py。"
            )

    def prepare(self, sid: int, strength: int) -> None:
        print(f"prepare: sid={sid}, strength={strength}")
        # self.configure_overload_protection(sid)
        self.set_grip_strength(sid, strength)
        self._write1(sid, self.sdk.SMS_STS_TORQUE_ENABLE, 1, "开启扭矩")

    def configure_overload_protection(self, sid: int) -> None:
        """写入负载过载参数，同时保留其余卸载保护开关。"""
        desired = {
            34: 20,   # 过载后保持 80% 的最大扭矩。
            35: 200,  # 保护时间 2 秒，单位为 10ms。
            36: 80,  # 过载阈值为最大扭矩的 100%。
        }
        unload_condition = self._read1(sid, 19, "读取卸载条件")
        protection_current_needs_update = (
            self._read2(sid, 28, "读取保护电流") != 400
        )

        # 关闭负载过载自动卸力
        # desired_unload_condition = unload_condition & ~0x20
        # 开启负载过载自动卸力
        desired_unload_condition = unload_condition | 0x20

        updates = [
            (address, value, action)
            for address, value, action in (
                (34, desired[34], "设置过载后保持力度"),
                (35, desired[35], "设置过载触发持续时间"),
                (36, desired[36], "设置过载阈值"),
                (19, desired_unload_condition, "设置负载过载自动卸力"),
            )
            if self._read1(sid, address, f"读取{action}") != value
        ]
        if not protection_current_needs_update and not updates:
            return
        self._write1(sid, 55, 0, "解锁舵机设置")
        try:
            if protection_current_needs_update:
                self._write2(sid, 28, 500, "设置保护电流")
            for address, value, action in updates:
                self._write1(sid, address, value, action)
        finally:
            self._write1(sid, 55, 1, "锁定舵机设置")

    def set_grip_strength(self, sid: int, strength: int) -> None:
        self._write2(sid, 48, strength * 10, "设置夹紧力度")

    def change_id(self, old_id: int, new_id: int) -> None:
        self._write1(old_id, 55, 0, "解锁舵机设置")
        self._write1(old_id, 5, new_id, "写入新 ID")
        self._write1(new_id, 55, 1, "锁定舵机设置")
        self.verify(new_id)

    def set_position(self, sid: int, position: int, speed: int, accel: int = 50) -> None:
        with self._io_lock:
            result, error = self.packet.WritePosEx(sid, position, speed, accel)
        self._check(result, error, "下发目标位置")

    def telemetry(self, sid: int) -> dict:
        with self._io_lock:
            data, result, _error = self.packet.readTxRx(
                sid, _FEEDBACK_START, _FEEDBACK_LENGTH
            )
        # The SDK packet error byte overlaps the status block's address 65.
        # The block remains decodable when that byte is nonzero.
        self._check(result, 0, "读取反馈")
        return decode_status_block(list(data))


def _scan(baudrates: tuple[int, ...]) -> list[dict]:
    from serial.tools import list_ports
    from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

    found = []
    for info in list_ports.comports():
        handler, opened = PortHandler(info.device), False
        try:
            try:
                opened = handler.openPort()
            except Exception:
                continue
            if not opened:
                continue
            packet = sms_sts(handler)
            for baudrate in baudrates:
                if not handler.setBaudRate(baudrate):
                    continue
                for sid in range(254):
                    model, result, error = packet.ping(sid)
                    if result == COMM_SUCCESS and not error:
                        found.append(
                            {
                                "port": info.device,
                                "servo_id": sid,
                                "baudrate": baudrate,
                                "model": model,
                            }
                        )
        finally:
            if opened:
                handler.closePort()
    return found


def discover_servos(baudrates: tuple[int, ...] = DEFAULT_BAUDRATES) -> list[dict]:
    if not baudrates:
        return []
    found = _scan((baudrates[0],))
    return found if found else _scan(baudrates[1:])


class _KeyReader:
    def __enter__(self):
        if os.name == "nt":
            import msvcrt

            self.read = msvcrt.getwch
        else:
            import termios, tty

            if not sys.stdin.isatty():
                raise GripperError("当前输入不是交互终端，无法使用 W/S。")
            self.termios, self.fd = termios, sys.stdin.fileno()
            self.settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.read = lambda: sys.stdin.read(1)
        return self

    def __exit__(self, *_):
        if os.name != "nt":
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.settings)


@dataclass
class _Channel:
    index: int
    config: dict
    bus: ServoBusPort
    calibrated: bool
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
    stop_result: tuple = ("就绪", "", None)
    stopped: threading.Event = field(default_factory=threading.Event)
    state: dict = field(default_factory=dict)
    next_status: float = 0
    initial_position_ready: threading.Event = field(default_factory=threading.Event)
    initial_position_error: GripperError | None = None
    sampled_at: float = 0.0
    error_code: int = 0
    bus_fault: bool = False
    overtemp_since: float | None = None
    overtemp_latched: bool = False
    applied_strength: int | None = None
    active_strength_key: str | None = None
    pending_strength: tuple[str, int] | None = None
    strength_error: GripperError | None = None
    strength_applied: threading.Event = field(default_factory=threading.Event)


class Gripper:
    def __init__(
        self,
        config_path: str | Path = DEFAULT_CONFIG_PATH,
        *,
        config: dict | None = None,
        allow_uninitialized=False,
        bus_factory: Callable[[str, int], ServoBusPort] | None = None,
    ):
        self.config_path = Path(config_path)
        self.config = {
            "servos": _servos(
                config if config is not None else _read_json(self.config_path)
            )
        }
        validate_config(self.config, calibrated=not allow_uninitialized)
        self.lock, self.stop_event, self.wakeup, self.buses, self.channels = (
            threading.RLock(),
            threading.Event(),
            threading.Event(),
            {},
            {},
        )
        self.strength_lock = threading.Lock()
        bus_factory = ServoBus if bus_factory is None else bus_factory
        try:
            for index, item in enumerate(self.config["servos"]):
                if item is None:
                    continue
                key = (item["serial_port"], item["baudrate"])
                if key not in self.buses:
                    self.buses[key] = bus_factory(*key)
                bus = self.buses[key]
                self.channels[index] = _Channel(
                    index, item, bus, item.get("open_position_steps") is not None
                )
            for bus in self.buses.values():
                bus.open()
            for item in self.channels.values():
                sid = item.config["servo_id"]
                item.bus.verify(sid)
                item.bus.check_position_mode(sid)
                strength_key = (
                    "close_grip_strength"
                    if item.calibrated
                    else "calibration_grip_strength"
                )
                item.bus.prepare(sid, item.config[strength_key])
                item.applied_strength = item.config[strength_key]
                item.active_strength_key = strength_key
                item.stopped.set()
                item.state = {
                    "state": "正在读取当前位置",
                    "blocked_direction": None,
                    "target_width": None,
                    "width": None,
                    "position_raw": None,
                    "position_unit": "steps",
                    "sampled_at": 0.0,
                    "error_code": 0,
                    "telemetry": None,
                    "message": "正在读取单圈绝对位置。",
                }
        except Exception:
            for bus in self.buses.values():
                bus.close()
            raise
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()
        try:
            self._wait_for_initial_positions()
        except Exception:
            self.disconnect()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disconnect()

    def disconnect(self):
        """停止后台更新并关闭串口；可重复调用。"""
        if not self.stop_event.is_set():
            for item in self.channels.values():
                self._request_stop(item, "已停止", "夹爪已停止。")
            for item in self.channels.values():
                item.stopped.wait(0.5)
        self.stop_event.set()
        self.wakeup.set()
        self.worker.join(timeout=1)
        if self.worker.is_alive():
            raise GripperError("后台轮询线程未退出，串口保持打开。")
        for bus in self.buses.values():
            bus.close()

    def close(self):
        """兼容旧名称；请优先使用 disconnect()。"""
        self.disconnect()

    def _channel(self, channel):
        if channel is None:
            if len(self.channels) == 1:
                return next(iter(self.channels.values()))
            raise GripperError("已配置多个舵机；请明确指定通道号 0 或 1。")
        if isinstance(channel, bool) or channel not in (0, 1):
            raise GripperError("通道号必须是 0 或 1。")
        if channel not in self.channels:
            raise GripperError(f"{channel} 号舵机未配置或不存在。")
        return self.channels[channel]

    def _wait_for_initial_positions(self):
        deadline = time.monotonic() + INITIAL_POSITION_TIMEOUT_S
        for item in self.channels.values():
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not item.initial_position_ready.wait(remaining):
                with self.lock:
                    error = item.initial_position_error
                detail = f"：{error}" if error else "。"
                raise GripperError(
                    f"{item.index} 号舵机未能在 {INITIAL_POSITION_TIMEOUT_S:g} 秒内读取当前位置"
                    + detail
                )
            with self.lock:
                if item.initial_position_error is not None:
                    raise item.initial_position_error

    @staticmethod
    def _width(item, position):
        closed = item.config["closed_position_steps"]
        span = item.config["open_position_steps"] - closed
        return max(0.0, min(1.0, (position - closed) / span))

    @staticmethod
    def _position_is_valid(item, position):
        closed = item.config["closed_position_steps"]
        opened = item.config["open_position_steps"]
        tolerance = item.config["position_tolerance_steps"]
        return (
            min(closed, opened) - tolerance
            <= position
            <= max(closed, opened) + tolerance
        )

    @staticmethod
    def _invalid_position_message(item):
        if item.position is None:
            status = item.state.get("message", "尚未成功读取舵机位置")
            return f"{item.index} 号舵机尚未读到当前位置。当前状态：{status}。"
        if not item.calibrated:
            return f"{item.index} 号舵机当前位置 {item.position} 无法用于未完成标定的操作。"
        closed = item.config["closed_position_steps"]
        opened = item.config["open_position_steps"]
        return (
            f"{item.index} 号舵机当前读数为 {item.position}，超出标定范围 "
            f"{min(closed, opened)} 到 {max(closed, opened)}"
            f"（闭合位置 {closed}，张开位置 {opened}）。"
            f"端点允许 {item.config['position_tolerance_steps']} 步的停靠误差。"
            "通常是确认标定点后舵机仍有少量移动，或开合边界标得过紧；"
            "请重新标定并在两个端点留出少量余量。"
        )

    @staticmethod
    def _direction(item, start, end):
        delta = end - start
        if not item.calibrated:
            return "正向" if delta > 0 else "反向"
        span = (
            item.config["open_position_steps"]
            - item.config["closed_position_steps"]
        )
        return "张开" if delta * span > 0 else "闭合"

    @staticmethod
    def _stall_window(item, direction):
        key = "closing_stall_window_s" if direction == "闭合" else "opening_stall_window_s"
        return float(item.config[key])

    def get_state(self, channel=None):
        item = self._channel(channel)
        with self.lock:
            return dict(item.state)

    def get_width(self, channel=None):
        item = self._channel(channel)
        if not item.calibrated:
            raise GripperError(f"{item.index} 号舵机尚未完成标定。")
        if item.position is None or not item.position_valid:
            raise GripperError(self._invalid_position_message(item))
        return self._width(item, item.position)

    def _request_stop(self, item, state, message, *, blocked_direction=None):
        with self.lock:
            item.version += 1
            item.target = None
            item.target_speed = None
            if blocked_direction is not None:
                item.blocked_direction = blocked_direction
            item.motion_reference_position = None
            item.motion_reference_at = None
            item.sent_version = None
            item.stop_pending, item.stop_version, item.stop_result = (
                True,
                item.version,
                (state, message, blocked_direction),
            )
            item.stopped.clear()
            item.state.update(
                state="停止中",
                blocked_direction=blocked_direction,
                target_width=None,
                message="正在停止舵机。",
            )
        self.wakeup.set()

    def _finish_stop(self, item, version):
        with self.lock:
            if not item.stop_pending or item.stop_version != version:
                return
            item.stop_pending = False
            if item.version == version:
                state, message, blocked = item.stop_result
                item.state.update(
                    state=state,
                    blocked_direction=blocked,
                    target_width=None,
                    message=message,
                )
            item.stopped.set()

    def _set_target(self, item, target, label=None, speed=None):
        with self.lock:
            if item.position is None or not item.position_valid:
                raise GripperError(self._invalid_position_message(item))
            if item.blocked_direction is not None:
                item.blocked_direction = None
            item.version += 1
            item.target = target
            item.target_speed = speed or item.config["speed"]
            item.motion_reference_position = None
            item.motion_reference_at = None
            item.sent_version = None
            item.state.update(
                state="运动中",
                blocked_direction=None,
                target_width=label,
                message="正在移动。",
            )
        self.wakeup.set()
        return True

    def _hold_closed(self, item):
        with self.lock:
            item.blocked_direction = "闭合"
            item.motion_reference_position = None
            item.motion_reference_at = None
            item.state.update(
                state="已夹住",
                blocked_direction="闭合",
                message="闭合方向检测到物体，正在按当前夹紧力度保持。",
            )
            item.stopped.set()

    def set_grip_strength(self, strength, channel=None, *, direction="闭合"):
        """更新指定方向的力度；当前正在使用时由后台立即写入。"""
        if (
            isinstance(strength, bool)
            or not isinstance(strength, int)
            or not 1 <= strength <= 100
        ):
            raise ValueError("夹紧力度必须是 1 到 100 的整数。")
        if direction not in {"张开", "闭合"}:
            raise ValueError("力度方向只能为“张开”或“闭合”。")
        key = "open_grip_strength" if direction == "张开" else "close_grip_strength"
        item = self._channel(channel)
        with self.strength_lock:
            with self.lock:
                if self.stop_event.is_set():
                    raise GripperError("夹爪已断开，无法更新夹紧力度。")
                item.pending_strength = (key, strength)
                item.strength_error = None
                item.strength_applied.clear()
            self.wakeup.set()
            if not item.strength_applied.wait(2):
                raise GripperError("更新夹紧力度超时。")
            with self.lock:
                if item.strength_error is not None:
                    raise item.strength_error

    def _apply_strength(self, item, key, strength=None):
        """仅在力度数值变化时写入；只能由后台循环调用。"""
        with self.lock:
            if strength is None:
                strength = item.config[key]
            changed = strength != item.applied_strength
        if changed:
            item.bus.set_grip_strength(item.config["servo_id"], strength)
        with self.lock:
            item.applied_strength = strength
            item.active_strength_key = key

    def _apply_pending_strength(self, item):
        with self.lock:
            pending = item.pending_strength
            active_key = item.active_strength_key
        if pending is None:
            return
        key, strength = pending
        try:
            if key == active_key:
                self._apply_strength(item, key, strength)
        except GripperError as exc:
            with self.lock:
                item.pending_strength = None
                item.strength_error = exc
                item.strength_applied.set()
            return
        with self.lock:
            item.config[key] = strength
            item.pending_strength = None
            item.strength_error = None
            item.strength_applied.set()

    def _temperature_fault(self, item, temperature_c: int) -> bool:
        """Stop after sustained over-temperature and clear only after cooling."""
        now = time.monotonic()
        with self.lock:
            trip_c = int(item.config["overtemp_c"])
            clear_c = int(item.config["overtemp_clear_c"])
            hold_s = float(item.config["overtemp_hold_s"])
            if temperature_c >= trip_c:
                if item.overtemp_since is None:
                    item.overtemp_since = now
                if now - item.overtemp_since < hold_s:
                    return False
                item.overtemp_latched = True
            elif not item.overtemp_latched:
                item.overtemp_since = None
                return False
            elif temperature_c <= clear_c:
                item.overtemp_since = None
                item.overtemp_latched = False
                if item.state["state"] == "异常" and item.target is None:
                    item.state.update(state="就绪", message="已恢复。")
                return False
            if item.stop_pending or item.state["message"] == "舵机温度过高，已停止运动。":
                return True
        self._request_stop(item, "异常", "舵机温度过高，已停止运动。")
        return True

    def _update(self, item):
        try:
            t = item.bus.telemetry(item.config["servo_id"])
            sampled_at = time.monotonic()
            with self.lock:
                item.position = t["raw_position"]
                item.sampled_at = sampled_at
                item.error_code = 0
                recovered_bus = item.bus_fault
                item.bus_fault = False
                item.position_valid = (
                    not item.calibrated
                    or self._position_is_valid(item, item.position)
                )
                position, target, target_speed, version, pending, stop_version = (
                    item.position,
                    item.target,
                    item.target_speed,
                    item.version,
                    item.stop_pending,
                    item.stop_version,
                )
                item.state.update(
                    width=(
                        self._width(item, position)
                        if item.calibrated and item.position_valid
                        else None
                    ),
                    position_raw=position,
                    sampled_at=sampled_at,
                    error_code=0,
                    telemetry=t,
                )
                if (
                    recovered_bus
                    and item.state["state"] == "异常"
                    and item.target is None
                    and not item.overtemp_latched
                ):
                    item.state.update(state="就绪", message="已恢复。")
                if not item.initial_position_ready.is_set():
                    if item.position_valid:
                        item.initial_position_error = None
                    else:
                        item.initial_position_error = GripperError(
                            self._invalid_position_message(item)
                        )
                    item.initial_position_ready.set()
            if not item.position_valid:
                with self.lock:
                    item.target = item.target_speed = None
                    item.state.update(
                        state="异常",
                        message=self._invalid_position_message(item),
                    )
                return
            if pending:
                item.bus.set_position(
                    item.config["servo_id"], position, item.config["speed"],
                    item.config["accel"],
                )
                self._finish_stop(item, stop_version)
                return
            if self._temperature_fault(item, t["temperature_c"]):
                return
            if target is None:
                return
            direction = self._direction(item, position, target)
            remaining = abs(position - target)
            if item.blocked_direction == direction:
                return
            if remaining <= item.config["position_tolerance_steps"]:
                self._request_stop(item, "已到位", "已到达目标位置。")
            elif item.sent_version != version:
                strength_key = (
                    "calibration_grip_strength"
                    if not item.calibrated
                    else (
                        "open_grip_strength"
                        if direction == "张开"
                        else "close_grip_strength"
                    )
                )
                self._apply_strength(item, strength_key)
                with self.lock:
                    if item.version != version:
                        return
                item.bus.set_position(
                    item.config["servo_id"], target, target_speed, item.config["accel"]
                )
                with self.lock:
                    if item.version != version:
                        return
                    item.sent_version = version
                    item.motion_reference_position = position
                    item.motion_reference_at = time.monotonic()
                    item.state.update(
                        state="运动中", blocked_direction=None, message="正在移动。"
                    )
            else:
                now = time.monotonic()
                with self.lock:
                    reference_position = item.motion_reference_position
                    reference_at = item.motion_reference_at
                    if (
                        reference_position is None
                        or reference_at is None
                        or abs(position - reference_position)
                        >= item.config["stall_movement_steps"]
                    ):
                        item.motion_reference_position = position
                        item.motion_reference_at = now
                        return
                    stalled = now - reference_at >= self._stall_window(item, direction)
                if not stalled:
                    return
                if direction == "闭合":
                    self._hold_closed(item)
                    return
                self._request_stop(
                    item,
                    "已阻塞",
                    f"{direction} 方向检测到物体或物理限位，已停止继续施力。",
                    blocked_direction=direction,
                )
        except GripperError as exc:
            with self.lock:
                item.error_code = exc.code or 0
                item.bus_fault = True
                if not item.initial_position_ready.is_set():
                    item.initial_position_error = exc
                item.target = item.target_speed = None
                item.stop_pending = False
                item.stopped.set()
                item.state.update(
                    state="异常", message=str(exc), error_code=item.error_code
                )

    def _run(self):
        while not self.stop_event.is_set():
            now, next_due = time.monotonic(), time.monotonic() + 0.05
            for item in self.channels.values():
                self._apply_pending_strength(item)
                if now >= item.next_status:
                    self._update(item)
                    item.next_status = now + 1 / item.config["status_frequency_hz"]
                next_due = min(next_due, item.next_status)
            self.wakeup.wait(max(0.001, next_due - time.monotonic()))
            self.wakeup.clear()

    def goto(self, width, channel=None):
        if not isinstance(width, (int, float)) or not 0 <= float(width) <= 1:
            raise ValueError("goto() 的取值必须在 0 到 1 之间。")
        item = self._channel(channel)
        if not item.calibrated:
            raise GripperError(f"{item.index} 号舵机尚未完成标定。")
        closed = item.config["closed_position_steps"]
        span = item.config["open_position_steps"] - closed
        self._set_target(
            item, closed + round(span * float(width)), float(width)
        )

    def wait(self, timeout=None, channel=None):
        item = self._channel(channel)
        deadline = time.monotonic() + (timeout or item.config["motion_timeout_s"])
        while time.monotonic() < deadline:
            result = self.get_state(item.index)
            if result["state"] not in {"运动中", "停止中", "正在读取当前位置"}:
                return result
            time.sleep(0.02)
        raise GripperError(f"{item.index} 号舵机等待动作完成超时。")

    def reset(self, channel=None):
        item = self._channel(channel)
        if not item.calibrated:
            raise GripperError(f"{item.index} 号舵机尚未完成标定。")
        self._set_target(
            item,
            item.config["neutral_position_steps"],
            self._width(item, item.config["neutral_position_steps"]),
        )
        result = self.wait(channel=item.index)
        if result["state"] != "已到位":
            raise GripperError("复位失败：" + result["message"])

    def manual_step(self, direction, channel=None):
        if direction not in {-1, 1}:
            raise ValueError("手动方向只能为 -1 或 1。")
        item = self._channel(channel)
        return self._set_target(
            item,
            min(
                STEPS_PER_TURN - 1,
                max(0, item.position + direction * item.config["calibration_step_steps"]),
            ),
            speed=min(item.config["speed"], 600),
        )

    def _keyboard(self, item, name):
        with _KeyReader() as reader:
            while True:
                key = reader.read().lower()
                if key in {"w", "s"}:
                    try:
                        self.manual_step(1 if key == "w" else -1, item.index)
                    except GripperError as exc:
                        print(str(exc))
                elif key in {"\r", "\n"}:
                    self._request_stop(item, "就绪", "已停止在当前位置。")
                    item.stopped.wait(1)
                    print(f"已确认{name}。\n")
                    return True
                elif key == "q":
                    self._request_stop(item, "就绪", "用户已取消手动移动。")
                    item.stopped.wait(1)
                    print("已取消。\n")
                    return False

    def calibrate_position(self, name, channel=None):
        item = self._channel(channel)
        print(f"\n标定 {item.index} 号舵机{name}：W/S 控制移动。到位后按回车；Q 退出。")
        return item.position if self._keyboard(item, name) else None

    def guided_grip_test(self, channel=None):
        item = self._channel(channel)
        if not item.calibrated:
            raise GripperError("请先完成闭合、中立和张开位置标定。")
        original_strength = item.config["close_grip_strength"]
        strength = original_strength

        def open_to_maximum():
            self.goto(1.0, item.index)
            result = self.wait(channel=item.index)
            if result["state"] == "异常":
                raise GripperError("张开夹爪失败：" + result["message"])

        open_to_maximum()
        print(
            f"\n{item.index} 号舵机夹紧测试，当前闭合力度：{strength}，"
            f"张开力度：{item.config['open_grip_strength']}。"
            "请放入测试物，按回车自动闭合；Q 退出。"
        )
        waiting_to_close = True
        with _KeyReader() as reader:
            while True:
                key = reader.read().lower()
                if key == "q":
                    open_to_maximum()
                    if strength != original_strength:
                        self.set_grip_strength(original_strength, item.index)
                    print("已跳过夹紧测试。\n")
                    return None
                if waiting_to_close and key in {"\r", "\n"}:
                    self.goto(0.0, item.index)
                    result = self.wait(channel=item.index)
                    if result["state"] == "异常":
                        raise GripperError("闭合夹爪失败：" + result["message"])
                    print(
                        f"本轮闭合力度：{strength}。感受夹持效果后："
                        "按 W 将闭合力度增加 5，按 S 减少 5，按回车确认保存。"
                    )
                    waiting_to_close = False
                    continue
                if not waiting_to_close and key in {"w", "s"}:
                    updated = max(1, min(100, strength + (5 if key == "w" else -5)))
                    if updated == strength:
                        print(f"当前力度已是 {strength}，无法继续调整。")
                        continue
                    self.set_grip_strength(updated, item.index)
                    strength = updated
                    print(f"当前闭合力度已更新为：{strength}。正在重新张开夹爪。")
                    open_to_maximum()
                    print("请重新放入测试物，按回车自动闭合；Q 退出。")
                    waiting_to_close = True
                    continue
                if not waiting_to_close and key in {"\r", "\n"}:
                    print(f"已确认闭合力度：{strength}。\n")
                    return strength
