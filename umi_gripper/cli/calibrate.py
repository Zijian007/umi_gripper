"""通过 W/S 分别完成 0、1 号夹爪舵机标定。"""

import argparse
import threading
from contextlib import contextmanager

from ..bus import UmiError
from ..compat import Gripper
from ..config import default_servo_config, save_config
from ..maintenance import discover_servos

STATUS_DISPLAY_INTERVAL_S = 0.5


def _status_text(gripper):
    entries = []
    for channel in gripper.channels:
        state = gripper.get_state(channel)
        telemetry = state.get("telemetry")
        if telemetry is None:
            entries.append(f"{channel} 号：正在读取状态")
            continue
        entries.append(
            f"{channel} 号：位置 {telemetry['raw_position']}，"
            f"负载 {telemetry['load']}，电流 {telemetry['current_ma']}mA，"
            f"状态 {state['state']}"
        )
    return "状态：" + "；".join(entries)


@contextmanager
def status_display(gripper, enabled):
    if not enabled:
        yield
        return
    stop_event = threading.Event()

    def display():
        while not stop_event.wait(STATUS_DISPLAY_INTERVAL_S):
            print(_status_text(gripper))

    thread = threading.Thread(target=display, daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop_event.set()
        thread.join()


def choose(candidates, channel, forbidden=None):
    print(f"\n选择 {channel} 号舵机：")
    print("  0. 本通道为空")
    for number, item in enumerate(candidates, 1):
        print(
            f"  {number}. 串口 {item['port']}，ID {item['servo_id']}，速度 {item['baudrate']}"
        )
    while True:
        answer = input("输入序号，或输入 q 退出：").strip().lower()
        if answer == "q":
            return "quit"
        if answer == "0":
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(candidates):
            item = candidates[int(answer) - 1]
            if forbidden == (item["port"], item["servo_id"]):
                print("同一串口和 ID 不能分配给两个通道，请重新选择。")
            else:
                return item
        else:
            print("序号无效，请重新输入。")


def optional_grip_test(gripper, channel):
    if (
        input(f"{channel} 号三点标定完成。按回车进行可选夹紧测试；输入 q 跳过：")
        .strip()
        .lower()
        == "q"
    ):
        print("已跳过夹紧测试。")
        return
    gripper.guided_grip_test(channel)


def main(*, config_path, log=False):
    print("夹爪初始化：请确认跳线在 B 位置。")
    if input("按回车扫描串口和 ID；输入 q 退出：").strip().lower() == "q":
        return
    candidates = discover_servos()
    if not candidates:
        print("未找到舵机。请检查 USB、供电、跳线和舵机线。")
        return
    first = choose(candidates, 0)
    if first == "quit":
        return
    second = choose(
        candidates, 1, None if first is None else (first["port"], first["servo_id"])
    )
    if second == "quit":
        return
    config = {
        "servos": [
            default_servo_config(first) if first else None,
            default_servo_config(second) if second else None,
        ]
    }
    try:
        with Gripper(
            config_path, config=config, allow_uninitialized=True, initialize_calibration=True,
        ) as gripper, status_display(gripper, log):
            print("已切换为单圈绝对位置模式，并将各舵机当前安装角度设为中位。请使用 W/S 逐通道标定。")
            saved = [None, None]
            for channel in gripper.channels:
                points = {}
                for field, name in (
                    ("closed_position_steps", "闭合位置"),
                    ("neutral_position_steps", "中立位置"),
                    ("open_position_steps", "张开位置"),
                ):
                    position = gripper.calibrate_position(name, channel)
                    if position is None:
                        return
                    points[field] = position
                gripper.set_calibration(channel, points)
                optional_grip_test(gripper, channel)
                saved[channel] = gripper.config["servos"][channel]
                save_config({"servos": saved}, config_path)
                print(f"{channel} 号舵机标定已保存。")
        print("标定完成并已保存。之后创建 Gripper 对象会直接读取当前位置。")
    except UmiError as exc:
        print(f"初始化失败：{exc}")


def cli():
    parser = argparse.ArgumentParser(description="标定夹爪舵机")
    parser.add_argument("--config", default="gripper_config.json", help="标定 JSON 路径")
    parser.add_argument("--log", action="store_true", help="每 0.5 秒显示一次舵机状态")
    args = parser.parse_args()
    main(config_path=args.config, log=args.log)
