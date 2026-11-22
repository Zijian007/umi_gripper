"""Compatibility imports for existing ``from gripper import …`` callers."""

from umi_gripper.bus import ServoBus, ServoBusPort, decode_status_block
from umi_gripper.bus import UmiError as GripperError
from umi_gripper.compat import Gripper, GripperReading
from umi_gripper.config import (
    DEFAULT_CONFIG_PATH,
    ConfigError,
    default_servo_config,
    save_config,
    validate_config,
)
from umi_gripper.maintenance import DEFAULT_BAUDRATES, discover_servos

__all__ = [
    "DEFAULT_BAUDRATES", "DEFAULT_CONFIG_PATH", "ConfigError", "Gripper", "GripperError",
    "GripperReading", "ServoBus", "ServoBusPort", "decode_status_block", "default_servo_config",
    "discover_servos", "save_config", "validate_config",
]
