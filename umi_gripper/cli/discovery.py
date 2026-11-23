"""Discover attached servos and display the shared selection menu."""

from ..session import _PortBusy, _maintenance_port

DEFAULT_BAUDRATES = (1_000_000, 500_000, 250_000, 115_200, 57_600, 38_400)


def discover_servos(baudrates: tuple[int, ...] = DEFAULT_BAUDRATES) -> list[dict]:
    if not baudrates:
        return []
    from serial.tools import list_ports

    found = []
    for info in list_ports.comports():
        try:
            with _maintenance_port(info.device) as port:
                found.extend(_scan_port(port, info.device, baudrates))
        except _PortBusy:
            continue
    return found


def _scan_port(port: str, device: str, baudrates: tuple[int, ...]) -> list[dict]:
    from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

    handler = PortHandler(port)
    try:
        opened = handler.openPort()
    except Exception:
        return []
    if not opened:
        return []
    found = []
    try:
        packet = sms_sts(handler)
        for baudrate in baudrates:
            if not handler.setBaudRate(baudrate):
                continue
            for sid in range(254):
                model, result, error = packet.ping(sid)
                if result == COMM_SUCCESS and not error:
                    found.append({"port": device, "servo_id": sid, "baudrate": baudrate, "model": model})
            if found:
                break
    finally:
        handler.closePort()
    return found


def choose_servo_number(candidates: list[dict], *, allow_empty: bool = False) -> int | None:
    """Return a displayed number, 0 for an empty slot, or None for quit."""
    if allow_empty:
        print("  选项 0 | 留空这个通道")
    for number, item in enumerate(candidates, 1):
        print(f"  选项 {number} | 舵机 ID {item['servo_id']} | {item['port']} | {item['baudrate']} baud")
    minimum = 0 if allow_empty else 1
    while True:
        answer = input("输入「选项」后的数字（如 1），按 Enter 确认，q 退出 > ").strip().lower()
        if answer == "q":
            return None
        if not answer:
            print("请先输入选项号，再按 Enter。")
            continue
        if answer.isdecimal() and minimum <= (number := int(answer)) <= len(candidates):
            return number
        print("选项号无效，请按菜单中的选项号输入。")
