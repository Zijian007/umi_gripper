"""Terminal-only maintenance prompts."""

import os
import sys

from ..bus import UmiError


class _KeyReader:
    def __enter__(self):
        if os.name == "nt":
            import msvcrt

            self.read = msvcrt.getwch
        else:
            import termios
            import tty

            if not sys.stdin.isatty():
                raise UmiError("当前输入不是交互终端，无法使用 W/S。")
            self.termios, self.fd = termios, sys.stdin.fileno()
            self.settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.read = lambda: sys.stdin.read(1)
        return self

    def __exit__(self, *_):
        if os.name != "nt":
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.settings)


def calibrate_position(gripper, name, channel):
    print(f"\n标定 {channel} 号舵机{name}。W/S 移动，回车确认，Q 退出。")
    with _KeyReader() as reader:
        while True:
            key = reader.read().lower()
            if key in {"w", "s"}:
                try:
                    gripper.manual_step(1 if key == "w" else -1, channel)
                except UmiError as exc:
                    print(exc)
            elif key in {"\r", "\n", "q"}:
                stopped = gripper.stop(channel)
                return None if key == "q" else stopped.position_steps


def guided_grip_test(gripper, channel):
    item = gripper.channels[channel]
    if not item.calibrated:
        raise UmiError("请先完成闭合、中立和张开位置标定。")
    original_strength = item.config["close_grip_strength"]
    strength = original_strength

    def open_to_maximum():
        gripper.goto(1.0, item.index)
        result = gripper.wait(channel=item.index)
        if result["state"] == "异常":
            raise UmiError("张开夹爪失败：" + result["message"])

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
                    gripper.set_grip_strength(original_strength, item.index)
                print("已跳过夹紧测试。\n")
                return None
            if waiting_to_close and key in {"\r", "\n"}:
                gripper.goto(0.0, item.index)
                result = gripper.wait(channel=item.index)
                if result["state"] == "异常":
                    raise UmiError("闭合夹爪失败：" + result["message"])
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
                gripper.set_grip_strength(updated, item.index)
                strength = updated
                print(f"当前闭合力度已更新为：{strength}。正在重新张开夹爪。")
                open_to_maximum()
                print("请重新放入测试物，按回车自动闭合；Q 退出。")
                waiting_to_close = True
                continue
            if not waiting_to_close and key in {"\r", "\n"}:
                print(f"已确认闭合力度：{strength}。\n")
                return strength
