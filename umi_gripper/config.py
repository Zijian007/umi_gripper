"""Calibration files, position conversion and runtime settings."""

from __future__ import annotations

import json
import math
import os
import stat
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from .bus import STEPS_PER_TURN, UmiError


def config_file(config_path: str | Path | None = None) -> Path:
    if config_path is not None:
        return Path(config_path).expanduser()
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "pyproject.toml").is_file():
        return source_root / "gripper_config.json"
    return Path.home() / "umi_gripper" / "gripper_config.json"


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
_DRIVER_SETTING_KEYS = (*_RUNTIME_OPTION_KEYS, *_MOTION_DEFAULTS)


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
    return _servos(_read_json(config_path))


def configured_channels(config_path: str | Path) -> tuple[int, ...]:
    servos = _read_servos(Path(config_path).expanduser())
    return tuple(channel for channel, item in enumerate(servos) if item is not None)


def _slot_index(channel: object) -> int:
    if isinstance(channel, bool) or not isinstance(channel, int) or channel not in (0, 1):
        raise ConfigError(f"umi channel must be 0 or 1, got {channel!r}")
    return channel


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
        unknown = options.keys() - _DRIVER_SETTING_KEYS
        if unknown:
            fields = ", ".join(sorted(map(repr, unknown)))
            raise ConfigError(f"unknown UMI settings: {fields}")
        item.update({key: value for key, value in options.items() if value is not None})

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


def load_servo_config(
    config_path: Path, channel: int, options: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    raw = _read_servos(config_path)[_slot_index(channel)]
    if raw is None:
        raise ConfigError(f"umi channel {channel} is not calibrated in {config_path}")
    return _servo_config(raw, channel, options=options)


def load_servo_configs(config_path: Path) -> dict[int, dict[str, Any]]:
    """Validate all configured channels from one read of the calibration file."""
    return {
        channel: _servo_config(raw, channel)
        for channel, raw in enumerate(_read_servos(config_path))
        if raw is not None
    }


def channels_by_servo_id(configs: Mapping[int, Mapping[str, Any]]) -> dict[int, list[int]]:
    channels: dict[int, list[int]] = {}
    for channel, config in configs.items():
        channels.setdefault(config["servo_id"], []).append(channel)
    return channels


def _read_json(path: Path, *, missing_ok: bool = False) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        if missing_ok:
            return {"servos": [None, None]}
        raise ConfigError(f"umi calibration file not found: {path}") from exc
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ConfigError(f"umi calibration file is not valid JSON: {exc}") from exc
    except OSError as exc:
        raise ConfigError(f"cannot read umi calibration file {path}: {exc}") from exc


def _read_calibration_for_update(path: Path) -> dict:
    saved = _read_json(path, missing_ok=True)
    if isinstance(saved, dict) and "servos" in saved:
        return {**saved, "servos": _servo_slots(saved["servos"])}
    return {"servos": _servos(saved)}


def _write_json_atomic(path: Path, config: dict) -> None:
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False,
        ) as temp:
            temp_path = Path(temp.name)
            if path.exists():
                temp_path.chmod(stat.S_IMODE(path.stat().st_mode))
            json.dump(config, temp, ensure_ascii=False, indent=2)
            temp.write("\n")
            temp.flush()
            os.fsync(temp.fileno())
        os.replace(temp_path, path)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def save_calibration_channel(path: Path, channel: int, calibration: Mapping[str, Any]) -> None:
    """Merge one completed calibration under a file lock, then publish it atomically."""
    channel = _slot_index(channel)
    calibrated = _servo_config(calibration, channel)
    try:
        path = path.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        # Lock a stable file because atomic replacement changes the JSON file's inode.
        with path.with_name(f".{path.name}.lock").open("a+b") as lock:
            if os.name == "nt":
                import msvcrt

                lock.seek(0)
                msvcrt.locking(lock.fileno(), msvcrt.LK_LOCK, 1)
            else:
                import fcntl

                fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            saved = _read_calibration_for_update(path)
            saved["servos"][channel] = calibrated
            _write_json_atomic(path, saved)
    except OSError as exc:
        raise ConfigError(f"cannot save umi calibration file {path}: {exc}") from exc


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


def _span_steps(config: Mapping[str, Any]) -> int:
    return int(config["open_position_steps"]) - int(config["closed_position_steps"])


def _close_01(config: Mapping[str, Any], position: int) -> float:
    """0 open, 1 closed."""
    width = (position - int(config["closed_position_steps"])) / _span_steps(config)
    return 1.0 - max(0.0, min(1.0, width))


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
