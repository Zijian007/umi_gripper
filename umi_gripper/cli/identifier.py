"""手动核对和修改总线舵机 ID。"""

import argparse

from ..bus import ServoBus, UmiError
from ..session import _maintenance_port
from .discovery import choose_servo_number, discover_servos


def change_servo_id(port: str, baudrate: int, old_id: int, new_id: int) -> None:
    with _maintenance_port(port) as device, ServoBus(device, baudrate) as bus:
        bus.verify(old_id)
        bus.change_id(old_id, new_id)


def main():
    print("舵机 ID 工具：请确认跳线在 B 位置。")
    print("修改 ID 时，同一串口只能连接目标舵机；同 ID 舵机会一起被修改。")
    while True:
        if input("按回车扫描；输入 q 退出：").strip().lower() == "q":
            return
        candidates = discover_servos()
        if not candidates:
            print("未找到舵机。请检查 USB、供电、跳线和舵机线。")
            continue
        number = choose_servo_number(candidates)
        if number is None:
            return
        selected = candidates[number - 1]
        print(
            f"当前：串口 {selected['port']}，ID {selected['servo_id']}，版本编号 {selected['model']}。"
        )
        answer = (
            input("输入新 ID（0-253），直接回车仅核对，或 q 返回扫描：").strip().lower()
        )
        if answer in {"", "q"}:
            continue
        if not answer.isdigit() or not 0 <= int(answer) <= 253:
            print("ID 无效，请输入 0 到 253 的整数。")
            continue
        new_id = int(answer)
        if new_id == selected["servo_id"]:
            print("新 ID 与当前 ID 相同，无需修改。")
            continue
        if any(
            item["port"] == selected["port"] and item["servo_id"] == new_id
            for item in candidates
        ):
            print("该串口上已发现此 ID。请重新输入新 ID 或退出。")
            continue
        try:
            change_servo_id(selected["port"], selected["baudrate"], selected["servo_id"], new_id)
            print(
                f"ID 已从 {selected['servo_id']} 修改为 {new_id}，且已重新确认通信正常。"
            )
        except UmiError as exc:
            print(f"修改失败：{exc}")


def cli():
    argparse.ArgumentParser(description="扫描、核对和修改舵机 ID").parse_args()
    main()
