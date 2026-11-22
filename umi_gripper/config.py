"""Load and validate shared calibration and runtime configuration."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

from .bus import STEPS_PER_TURN, UmiError

_SOURCE_CONFIG_PATH = Path(__file__).resolve().parents[1] / "gripper_config.json"
DEFAULT_CONFIG_PATH = _SOURCE_CONFIG_PATH if _SOURCE_CONFIG_PATH.is_file() else Path("gripper_config.json")


class ConfigError(UmiError):
    pass


# Defaults for runtime motion and thermal protection.
_MOTION_DEFAULTS = {
    "accel": 50,
    "position_tolerance_steps": 12,
    "stall_movement_steps": 3,
    "opening_stall_window_s": 0.5,
    "closing_stall_window_s": 0.2,
    "overtemp_c": 70,
    "overtemp_clear_c": 65,
    "overtemp_hold_s": 0.5,
}

_RUNTIME_OPTION_KEYS = (
    "speed",
    "open_grip_strength",
    "close_grip_strength",
    "status_frequency_hz",
)
_BUS_KEYS = ("serial_port", "servo_id", "baudrate")
_POSITION_KEYS = (
    "closed_position_steps",
    "neutral_position_steps",
    "open_position_steps",
)
_CALIBRATION_KEYS = (*_BUS_KEYS, *_POSITION_KEYS)
_MOTION_OPTION_KEYS = tuple(_MOTION_DEFAULTS)


def _span_steps(config: Mapping[str, Any]) -> int:
    return int(config["open_position_steps"]) - int(config["closed_position_steps"])


def _width(config: Mapping[str, Any], position: int) -> float:
    """0 at the closed stop, 1 at the open stop."""
    return max(0.0, min(1.0, (position - int(config["closed_position_steps"])) / _span_steps(config)))


def _close_01(config: Mapping[str, Any], position: int) -> float:
    """0 open, 1 closed."""
    return 1.0 - _width(config, position)


def _steps_for_close_01(config: Mapping[str, Any], close_01: float) -> int:
    closed = int(config["closed_position_steps"])
    return closed + round(_span_steps(config) * (1.0 - close_01))


def _position_in_span(config: Mapping[str, Any], position: int) -> bool:
    closed = int(config["closed_position_steps"])
    opened = int(config["open_position_steps"])
    low, high = min(closed, opened), max(closed, opened)
    tolerance = config["position_tolerance_steps"]
    return low - tolerance <= position <= high + tolerance


def _move_direction(config: Mapping[str, Any], start: int, end: int) -> str:
    """``open`` when width increases. A zero move counts as ``close``."""
    return "open" if (end - start) * _span_steps(config) > 0 else "close"


def _calibration_span_ok(opened: int, closed: int, neutral: int) -> bool:
    span = opened - closed
    neutral_delta = neutral - closed
    return (
        10 <= abs(span) < STEPS_PER_TURN // 2
        and neutral_delta * span >= 0
        and abs(neutral_delta) <= abs(span)
    )


def _invalid_position_message(index: int, position: int | None, config: Mapping[str, Any], *, detail: str) -> str:
    if position is None:
        return f"umi channel {index} has no position yet ({detail})"
    closed = int(config["closed_position_steps"])
    opened = int(config["open_position_steps"])
    return (
        f"umi channel {index} position {position} outside calibration "
        f"[{min(closed, opened)}, {max(closed, opened)}]"
    )


def _require_close_01(close_01: float) -> float:
    if not isinstance(close_01, (int, float)) or isinstance(close_01, bool):
        raise ValueError("close_01 must be a finite number")
    close_01 = float(close_01)
    if not math.isfinite(close_01) or not 0.0 <= close_01 <= 1.0:
        raise ValueError("close_01 must be in [0, 1]")
    return close_01


def _finite(config: Mapping[str, Any], key: str, low: float, high: float) -> float:
    value = config.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"umi config {key!r} must be a number in [{low:g}, {high:g}]")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ConfigError(f"umi config {key!r} must be a number in [{low:g}, {high:g}]")
    return value


def _apply_motion_options(item: dict[str, Any]) -> None:
    _integer(item, "accel", 0, 254)
    _integer(item, "position_tolerance_steps", 0, 128)
    _integer(item, "stall_movement_steps", 1, 128)
    _integer(item, "overtemp_c", 1, 100)
    _integer(item, "overtemp_clear_c", 0, 99)
    if item["overtemp_clear_c"] >= item["overtemp_c"]:
        raise ConfigError("umi config 'overtemp_clear_c' must be below 'overtemp_c'")
    for key in ("opening_stall_window_s", "closing_stall_window_s"):
        item[key] = _finite(item, key, 0.05, 5.0)
    item["overtemp_hold_s"] = _finite(item, "overtemp_hold_s", 0.0, 5.0)


def _integer(config: Mapping[str, Any], key: str, low: int, high: int) -> int:
    return _checked_int(config.get(key), key, low, high)


def _checked_int(value: Any, key: str, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ConfigError(f"umi config {key!r} must be an int in [{low}, {high}]")
    return value


def _servo_slots(servos: Any) -> list[dict | None]:
    if not isinstance(servos, list) or len(servos) != 2 or any(
        item is not None and not isinstance(item, dict) for item in servos
    ):
        raise ConfigError("umi calibration JSON must contain servos: [left|null, right|null]")
    return servos


def _read_servos(config_path: Path) -> list[dict | None]:
    document = _read_json(config_path)
    return _servo_slots(document.get("servos") if isinstance(document, dict) else None)


def _side_channel(channel: object) -> int:
    if isinstance(channel, bool) or not isinstance(channel, int) or channel not in (0, 1):
        raise ConfigError(f"umi options.channel must be 0 or 1, got {channel!r}")
    return channel


def _calibrated_channel(
    servos: list[dict | None], requested_channel: int, *, side: str, config_path: Path,
) -> tuple[int, dict]:
    """YAML channel → calibrated JSON slot. A missing side uses the other slot."""
    requested = servos[requested_channel]
    if requested is not None:
        return requested_channel, requested
    fallback_channel = 1 - requested_channel
    fallback = servos[fallback_channel]
    if fallback is not None:
        return fallback_channel, fallback
    raise ConfigError(
        f"umi channel {requested_channel} for {side} is not calibrated in {config_path} "
        f"(and channel {fallback_channel} is also missing)"
    )


def _calibration_points(config: Mapping[str, Any], channel: int) -> dict[str, int]:
    points = {
        key: _integer(config, key, 0, STEPS_PER_TURN - 1) for key in _POSITION_KEYS
    }
    if not _calibration_span_ok(
        points["open_position_steps"], points["closed_position_steps"], points["neutral_position_steps"],
    ):
        raise ConfigError(f"umi channel {channel} calibration span is invalid")
    return points


def _servo_config(
    raw: Mapping[str, Any], channel: int, *, options: Mapping[str, Any] | None = None, calibrated: bool = True,
) -> dict[str, Any]:
    item = {**_MOTION_DEFAULTS, **raw}
    if options is not None:
        for key in (*_RUNTIME_OPTION_KEYS, *_MOTION_OPTION_KEYS):
            if options.get(key) is not None:
                item[key] = options[key]

    required = (*_BUS_KEYS, *_RUNTIME_OPTION_KEYS, *_POSITION_KEYS) if calibrated else (*_BUS_KEYS, *_RUNTIME_OPTION_KEYS)
    missing = [key for key in required if item.get(key) in (None, "")]
    if missing:
        raise ConfigError(f"umi channel {channel} missing keys: {', '.join(missing)}")
    if not isinstance(item["serial_port"], str):
        raise ConfigError(f"umi channel {channel} serial_port must be a string")

    _integer(item, "servo_id", 0, 253)
    _integer(item, "baudrate", 9600, 1_000_000)
    _integer(item, "speed", 1, 3400)
    _integer(item, "open_grip_strength", 1, 100)
    _integer(item, "close_grip_strength", 1, 100)
    _integer(item, "status_frequency_hz", 1, 100)
    if calibrated:
        _calibration_points(item, channel)
    _apply_motion_options(item)
    return item


def load_servo_config(config_path: Path, channel: int, options: Mapping[str, Any]) -> dict[str, Any]:
    raw = _read_servos(config_path)[_side_channel(channel)]
    if raw is None:
        raise ConfigError(f"umi channel {channel} is not calibrated in {config_path}")
    return _servo_config(raw, channel, options=options)


def _read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConfigError(f"umi calibration file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(f"umi calibration file is not valid JSON: {exc}") from exc


def save_config(config: dict, path: str | Path = DEFAULT_CONFIG_PATH) -> None:
    saved = dict(config) if "servos" in config else {}
    saved["servos"] = [
        {**_MOTION_DEFAULTS, **item}
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
        "open_grip_strength": 40,
        "close_grip_strength": 40,
        "status_frequency_hz": 50,
        "calibration_step_steps": 128,
        "motion_timeout_s": 30,
        **_MOTION_DEFAULTS,
    }


def _servos(config: dict) -> list[dict | None]:
    if not isinstance(config, dict):
        raise ConfigError("夹爪配置必须是 JSON 对象。")
    servos = []
    for item in _servo_slots(config.get("servos", [config, None])):
        if item is None:
            servos.append(None)
            continue
        item = dict(item)
        item.pop("calibration_grip_strength", None)
        old_strength = item.pop("grip_strength", None)
        if old_strength is not None:
            item.setdefault("open_grip_strength", old_strength)
            item.setdefault("close_grip_strength", old_strength)
        servos.append(item)
    return servos


def _legacy_servos(config: dict, *, calibrated: bool, validate_existing: bool = False) -> list[dict | None]:
    servos = _servos(config)
    active = [(i, x) for i, x in enumerate(servos) if x is not None]
    if not active:
        raise ConfigError("至少需要配置一个舵机。")
    seen, ports = set(), {}
    for index, item in active:
        servos[index] = _servo_config(
            item, index,
            calibrated=calibrated or (validate_existing and item.get("open_position_steps") is not None),
        )
        _integer(item, "calibration_step_steps", 10, 1024)
        _integer(item, "motion_timeout_s", 2, 120)
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
    return servos


def validate_config(config: dict, *, calibrated: bool = True) -> None:
    _legacy_servos(config, calibrated=calibrated)
