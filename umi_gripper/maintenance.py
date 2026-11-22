"""Explicit serial discovery for the maintenance tools."""

DEFAULT_BAUDRATES = (1_000_000, 500_000, 250_000, 115_200, 57_600, 38_400)


def _scan(baudrates: tuple[int, ...]) -> list[dict]:
    from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
    from serial.tools import list_ports

    found = []
    for info in list_ports.comports():
        handler, opened = PortHandler(info.device), False
        try:
            try:
                opened = handler.openPort()
            except Exception:
                continue
            if not opened:
                continue
            packet = sms_sts(handler)
            for baudrate in baudrates:
                if not handler.setBaudRate(baudrate):
                    continue
                for sid in range(254):
                    model, result, error = packet.ping(sid)
                    if result == COMM_SUCCESS and not error:
                        found.append(
                            {
                                "port": info.device,
                                "servo_id": sid,
                                "baudrate": baudrate,
                                "model": model,
                            }
                        )
        finally:
            if opened:
                handler.closePort()
    return found


def discover_servos(baudrates: tuple[int, ...] = DEFAULT_BAUDRATES) -> list[dict]:
    if not baudrates:
        return []
    found = _scan((baudrates[0],))
    return found if found else _scan(baudrates[1:])
