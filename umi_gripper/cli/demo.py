"""演示单通道或双通道夹爪控制。"""

import argparse
from contextlib import ExitStack

from ..bus import UmiError
from ..config import channels_by_servo_id, config_file, load_servo_configs
from ..gripper import Gripper


_SIDE_NAMES = ("左夹爪", "右夹爪")
_MOTION_STATUS = {"arrived": "已到位", "grasped": "已夹住", "blocked": "张开受阻", "error": "故障"}


def _select_channel(configs):
    channels_by_id = channels_by_servo_id(configs)
    choices = {}
    print("可选夹爪")
    for channel, config in configs.items():
        servo_id = config["servo_id"]
        choice = str(servo_id) if len(channels_by_id[servo_id]) == 1 else f"{servo_id} {channel}"
        choices[choice] = channel
        print(f"  输入 {choice}  {_SIDE_NAMES[channel]}（舵机 ID {servo_id}，通道 {channel}）")

    while True:
        choice = " ".join(input("选择夹爪（q 退出）> ").lower().split())
        if choice == "q":
            return None
        if choice in choices:
            return choices[choice]
        print("请选择列表里的输入值。")


def _run_selected_gripper(gripper, label):
    print(f"已选 {label}。输入开度，0 全闭，1 全开，0.8 表示 80%。s 换夹爪，q 退出。")
    while True:
        choice = input(f"{label} 开度> ").strip().lower()
        if choice in ("s", "q"):
            return choice
        try:
            width = float(choice)
            if not 0 <= width <= 1:
                raise ValueError
        except ValueError:
            print("请输入 0 到 1 的开度，例如输入 0.8 打开到 80%。")
            continue

        gripper.set_target(1.0 - width, retry=True)
        result = gripper.wait()
        actual = "不可用" if result.close_01 is None else f"{1.0 - result.close_01:.1%}"
        status = _MOTION_STATUS.get(result.state, result.message)
        print(f"目标开度 {width:.1%} | 实测开度 {actual} | {status}")
        if result.state in ("blocked", "error"):
            print(result.message)


def main(config_path=None):
    try:
        config_path = config_file(config_path)
        configs = load_servo_configs(config_path)
        if not configs:
            raise UmiError("标定文件中没有舵机。")
        print(f"UMI 夹爪试跑 | 标定文件 {config_path}")
        with ExitStack() as stack:
            grippers = {}
            while True:
                channel = _select_channel(configs)
                if channel is None:
                    return
                config = configs[channel]
                if channel not in grippers:
                    grippers[channel] = stack.enter_context(Gripper._open_config(channel, config))
                label = f"{_SIDE_NAMES[channel]}（ID {config['servo_id']}）"
                if _run_selected_gripper(grippers[channel], label) == "q":
                    return
    except (EOFError, KeyboardInterrupt):
        print()
    except UmiError as exc:
        print(f"操作失败：{exc}")


def cli():
    parser = argparse.ArgumentParser(description="交互试跑已标定的夹爪")
    parser.add_argument("--config", help=f"标定 JSON 路径（默认 {config_file()}）")
    main(parser.parse_args().config)


if __name__ == "__main__":
    cli()
