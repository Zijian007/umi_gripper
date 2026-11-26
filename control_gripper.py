"""Open or close one calibrated UMI gripper from the terminal.

usage: control_gripper.py [-h] (--channel {0,1} | --id ID) [--config CONFIG]
                          {open,close}

选择一只已标定的 UMI 夹爪并全开或全闭

positional arguments:
  {open,close}     open 全开，close 全闭

options:
  -h, --help       show this help message and exit
  --channel {0,1}  标定通道，0 左侧，1 右侧
  --id ID          标定文件中的总线舵机 ID，范围 0–253
  --config CONFIG  标定文件（默认 /home/zijianwang/umi_gripper/gripper_config.json）
channel 0 -> ID 1
channel 1 -> ID 0
"""

import argparse
import sys
from pathlib import Path
from typing import Any

from umi_gripper import Gripper, UmiError
from umi_gripper.config import (
    channels_by_servo_id,
    config_file,
    load_servo_config,
    load_servo_configs,
)


def calibration_for_id(config_path: Path, servo_id: int) -> tuple[int, dict[str, Any]]:
    configs = load_servo_configs(config_path)
    matches = channels_by_servo_id(configs).get(servo_id, [])
    if not matches:
        raise UmiError(f"{config_path} 里没有舵机 ID {servo_id} 的标定。")
    if len(matches) > 1:
        raise UmiError(f"舵机 ID {servo_id} 对应多个标定通道，请用 --channel 指定。")
    channel = matches[0]
    return channel, configs[channel]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="选择一只已标定的 UMI 夹爪并全开或全闭")
    selector = parser.add_mutually_exclusive_group(required=True)
    selector.add_argument("--channel", type=int, choices=(0, 1), help="标定通道，0 左侧，1 右侧")
    selector.add_argument("--id", type=int, help="标定文件中的总线舵机 ID，范围 0–253")
    parser.add_argument("action", choices=("open", "close"), help="open 全开，close 全闭")
    parser.add_argument("--config", type=Path, help=f"标定文件（默认 {config_file()}）")
    args = parser.parse_args(argv)
    if args.id is not None and not 0 <= args.id <= 253:
        parser.error("--id 必须在 0–253 之间")

    path = config_file(args.config)
    closing = args.action == "close"
    try:
        if args.channel is not None:
            channel = args.channel
            config = load_servo_config(path, channel)
        else:
            channel, config = calibration_for_id(path, args.id)
        with Gripper._open_config(channel, config) as gripper:
            servo_id = config["servo_id"]
            print(f"通道 {channel}，舵机 ID {servo_id}，开始{'闭合' if closing else '张开'}。", flush=True)
            gripper.set_target(float(closing))
            reading = gripper.wait()
            print(f"状态 {reading.state}，位置 {reading.raw_position} 步。{reading.message}")
        return 0 if reading.state == "arrived" or (closing and reading.state == "grasped") else 1
    except (UmiError, OSError) as exc:
        print(f"夹爪控制失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
