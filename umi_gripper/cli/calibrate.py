"""通过 W/S 分别完成 0、1 号夹爪舵机标定。"""

import argparse
import os
import sys
import threading
from contextlib import ExitStack, contextmanager

from ..bus import STEPS_PER_TURN, UmiError
from ..config import _read_calibration_for_update, config_file, save_calibration_channel
from ..gripper import Gripper
from .discovery import choose_servo_number, discover_servos

STATUS_DISPLAY_INTERVAL_S = 0.5
STATUS_POSITION_THRESHOLD_STEPS = 4
CALIBRATION_POINTS = (
    ("closed_position_steps", "闭合位置"),
    ("neutral_position_steps", "中立位置"),
    ("open_position_steps", "张开位置"),
)


def _channel_name(channel):
    side = "左" if channel == 0 else "右"
    return f"{side}夹爪（通道 {channel}）"


def _status_text(readings):
    entries = []
    for channel, reading in readings.items():
        position = "待读取" if reading.raw_position is None else f"{reading.raw_position:g}"
        status = reading.state
        if reading.state == "error" and reading.message:
            status = f"{status}（{reading.message}）"
        entries.append(
            f"{_channel_name(channel)} 位置 {position}，"
            f"负载 {reading.effort:g}，电流 {reading.current * 1000:g}mA，"
            f"状态 {status}"
        )
    return "状态：" + "；".join(entries)


def _status_changed(previous, current):
    if previous.state != current.state or previous.message != current.message:
        return True
    before, after = previous.raw_position, current.raw_position
    if before is None or after is None:
        return before != after
    return abs(after - before) >= STATUS_POSITION_THRESHOLD_STEPS


def _display_status(grippers, stop_event):
    last_printed = {}
    while not stop_event.wait(STATUS_DISPLAY_INTERVAL_S):
        readings = {channel: gripper.read() for channel, gripper in grippers.items()}
        if not last_printed or any(
            _status_changed(last_printed[channel], reading) for channel, reading in readings.items()
        ):
            print(_status_text(readings), flush=True)
            last_printed = readings


@contextmanager
def status_display(grippers, enabled):
    if not enabled:
        yield
        return
    stop_event = threading.Event()

    thread = threading.Thread(target=_display_status, args=(grippers, stop_event), daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop_event.set()
        thread.join()


def select_channels(candidates):
    selected = {}
    for channel in (0, 1):
        name = _channel_name(channel)
        print(f"\n为{name}选择舵机")
        while True:
            number = choose_servo_number(candidates, allow_empty=True)
            if number is None:
                return None
            if number == 0:
                print(f"{name}已留空。")
                break
            item = candidates[number - 1]
            if any(
                item["port"] == previous["port"] and item["servo_id"] == previous["servo_id"]
                for previous in selected.values()
            ):
                print("同一串口和 ID 不能分配给两个通道，请重新选择。")
                continue
            selected[channel] = item
            print(f"{name}已选择舵机 ID {item['servo_id']}（{item['port']}）。")
            break
    return selected


def optional_grip_test(gripper, channel):
    if (
        input(f"{_channel_name(channel)}三点标定完成。按 Enter 试夹，或输入 q 后按 Enter 跳过 > ")
        .strip()
        .lower()
        == "q"
    ):
        print("已跳过夹紧测试。")
        return
    guided_grip_test(gripper)


def main(*, config_path=None, log=False):
    config_path = config_file(config_path)
    print("夹爪初始化：请确认跳线在 B 位置。")
    if input("按回车扫描串口和 ID；输入 q 退出：").strip().lower() == "q":
        return
    candidates = discover_servos()
    if not candidates:
        print("未找到舵机。请检查 USB、供电、跳线和舵机线。")
        return
    selected = select_channels(candidates)
    if selected is None:
        return
    if not selected:
        print("至少需要配置一个舵机。")
        return
    try:
        _read_calibration_for_update(config_path)
        with ExitStack() as stack:
            print("将逐通道设置单圈绝对位置模式和中位。请使用 W/S 标定。")
            for channel, item in selected.items():
                gripper = stack.enter_context(Gripper._open_for_calibration(channel, item))
                with status_display({channel: gripper}, log):
                    points = {}
                    for step, (field, name) in enumerate(CALIBRATION_POINTS, 1):
                        position = calibrate_position(gripper, name, step)
                        if position is None:
                            print(f"{_channel_name(channel)}已设置中位。请重新完成标定后再使用该夹爪。")
                            return
                        points[field] = position
                    gripper._set_calibration(points)
                    optional_grip_test(gripper, channel)
                save_calibration_channel(config_path, channel, gripper._config_snapshot())
                print(f"{_channel_name(channel)}的标定已保存到 {config_path}。")
        print("标定完成并已保存。之后创建 Gripper 对象会直接读取当前位置。")
    except UmiError as exc:
        print(f"初始化失败：{exc}")


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


def manual_step(gripper, direction):
    if direction not in {-1, 1}:
        raise ValueError("手动方向只能为 -1 或 1。")
    position = gripper.read().raw_position
    if position is None:
        raise UmiError(f"{_channel_name(gripper._channel)}尚未读到当前位置。")
    config = gripper._config_snapshot()
    target = max(0, min(STEPS_PER_TURN - 1, int(position) + direction * config["calibration_step_steps"]))
    gripper._set_position(target, speed=min(config["speed"], 600))


def calibrate_position(gripper, name, step):
    print(f"\n{_channel_name(gripper._channel)} 第 {step}/{len(CALIBRATION_POINTS)} 步，标定{name}。")
    print("按 W / S 移动，按 Enter 记录当前位置，按 Q 取消标定。")
    with _KeyReader() as reader:
        while True:
            key = reader.read().lower()
            if key in {"w", "s"}:
                try:
                    manual_step(gripper, 1 if key == "w" else -1)
                except UmiError as exc:
                    print(exc)
            elif key in {"\r", "\n", "q"}:
                gripper._stop(timeout=1.0, allow_invalid=key == "q")
                return None if key == "q" else int(gripper.read().raw_position)


def guided_grip_test(gripper):
    config = gripper._config_snapshot()
    if config["closed_position_steps"] is None:
        raise UmiError("请先完成闭合、中立和张开位置标定。")
    original_strength = config["close_grip_strength"]
    strength = original_strength

    def open_to_maximum():
        gripper.set_target(0.0)
        result = gripper.wait()
        if result.state == "error":
            raise UmiError("张开夹爪失败：" + result.message)

    open_to_maximum()
    print(
        f"\n{_channel_name(gripper._channel)}试夹，当前闭合力度：{strength}，"
        f"张开力度：{config['open_grip_strength']}。"
        "请放入测试物，按回车自动闭合；Q 退出。"
    )
    waiting_to_close = True
    with _KeyReader() as reader:
        while True:
            key = reader.read().lower()
            if key == "q":
                open_to_maximum()
                if strength != original_strength:
                    gripper._set_grip_strength("close_grip_strength", original_strength)
                print("已跳过夹紧测试。\n")
                return None
            if waiting_to_close and key in {"\r", "\n"}:
                gripper.set_target(1.0)
                result = gripper.wait()
                if result.state == "error":
                    raise UmiError("闭合夹爪失败：" + result.message)
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
                gripper._set_grip_strength("close_grip_strength", updated)
                strength = updated
                print(f"当前闭合力度已更新为：{strength}。正在重新张开夹爪。")
                open_to_maximum()
                print("请重新放入测试物，按回车自动闭合；Q 退出。")
                waiting_to_close = True
                continue
            if not waiting_to_close and key in {"\r", "\n"}:
                print(f"已确认闭合力度：{strength}。\n")
                return strength


def cli():
    parser = argparse.ArgumentParser(description="标定夹爪舵机")
    parser.add_argument("--config", help=f"标定 JSON 输出路径（默认 {config_file()}）")
    parser.add_argument("--log", action="store_true", help="每 0.5 秒检查反馈，有变化时显示状态")
    args = parser.parse_args()
    main(config_path=args.config, log=args.log)
