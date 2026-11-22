"""Feetech serial port for one UMI servo bus. Motion scheduling lives in ``runtime.py``."""

from __future__ import annotations

import threading
import time
from typing import Any, Protocol

STEPS_PER_TURN = 4096
# One read covers present position through present current (addresses 56–70).
_PRESENT_POSITION = 56
_FEEDBACK_LENGTH = 15

_COMM_FAILURE = {
    -1: "port busy",
    -2: "transmit failed",
    -3: "status packet failed",
    -4: "bad instruction",
    -5: "still waiting",
    -6: "no status packet",
    -7: "corrupt status packet",
    -9: "not available",
}


class UmiError(RuntimeError):
    def __init__(self, message: str, *, code: int | None = None, resource: Any = None):
        super().__init__(message)
        self.code = code
        self.resource = resource


def _word(data: list[int], offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def _signed(value: int, bit: int) -> int:
    if value & (1 << bit):
        return -(value & ~(1 << bit))
    return value


def decode_status_block(data: list[int]) -> dict:
    """Decode one STS feedback block starting at address 56.

    Position uses bit 15 as the sign, then wraps into one turn. Load (address
    60) is PWM duty in 0.1% counts with bit 10 as the sign. Current (address
    69) is 6.5 mA counts with bit 15 as the sign. Speed, voltage, and the
    moving flag sit in the same block and are not used.
    """
    if len(data) < _FEEDBACK_LENGTH:
        raise UmiError(f"read feedback failed: short status packet ({len(data)} bytes)")
    position = _signed(_word(data, 0), 15) % STEPS_PER_TURN
    load = _signed(_word(data, 4), 10)
    current = _signed(_word(data, 13), 15)
    return {
        "raw_position": position,
        "load": load,
        "current_ma": round(current * 6.5),
        "temperature_c": data[7],
    }


def _comm_message(action: str, result: int) -> str:
    reason = _COMM_FAILURE.get(result, "communication error")
    return f"{action} failed: {reason} ({result})"


class ServoBusPort(Protocol):
    """Controller-facing bus contract; successful reads provide explicit position/status feedback."""

    def open(self) -> None: ...
    def close(self) -> None: ...
    def verify(self, sid: int) -> int: ...
    def configure_position_mode(self, sid: int) -> None: ...
    def set_current_position_as_middle(self, sid: int) -> None: ...
    def check_position_mode(self, sid: int) -> None: ...
    def prepare(self, sid: int, strength: int) -> None: ...
    def set_grip_strength(self, sid: int, strength: int) -> None: ...
    def set_position(self, sid: int, position: int, speed: int, accel: int = 50) -> None: ...
    def telemetry(self, sid: int) -> dict: ...


class ServoBus:
    """One physical serial port shared by servo IDs at the same baudrate.

    Threads: the UMI session worker polls and commands; owner threads open, prepare
    and close. Lock: ``_io_lock`` is a leaf lock held for exactly one serial
    transaction or for ``open``/``close``; bus methods never take other locks.
    ``close()`` therefore waits for an in-flight transaction, and any later
    transaction raises ``UmiError`` instead of touching a closed port.
    """

    def __init__(self, port: str, baudrate: int):
        from scservo_sdk import PortHandler

        self.port_name = port
        self.baudrate = baudrate
        self.port = PortHandler(port)
        self.packet = None
        self.opened = False
        self._io_lock = threading.Lock()

    def _txrx(self, action: str, method: str, *args: Any) -> Any:
        """Run one packet transaction under ``_io_lock``; refuse unless the port is open."""
        with self._io_lock:
            if not self.opened or self.packet is None:
                raise UmiError(f"{action} failed: serial port {self.port_name} is not open")
            return getattr(self.packet, method)(*args)

    @staticmethod
    def _check(result: int, error: int, action: str) -> None:
        from scservo_sdk import COMM_SUCCESS

        if result != COMM_SUCCESS:
            raise UmiError(_comm_message(action, result), code=result)
        if error:
            raise UmiError(f"{action} failed: servo error ({error})", code=error)

    def open(self) -> None:
        from scservo_sdk import sms_sts

        with self._io_lock:
            try:
                opened = self.port.openPort()
            except Exception as exc:
                raise UmiError(f"cannot open serial port {self.port_name}: {exc}") from exc
            if not opened:
                raise UmiError(f"cannot open serial port {self.port_name}")
            self.opened = True
            try:
                if not self.port.setBaudRate(self.baudrate):
                    raise UmiError(f"cannot set baudrate {self.baudrate}")
                self.packet = sms_sts(self.port)
            except Exception:
                self.port.closePort()
                self.opened = False
                raise

    def close(self) -> None:
        """Close after any in-flight transaction; later transactions raise ``UmiError``."""
        with self._io_lock:
            if self.opened:
                self.port.closePort()
                self.opened = False
                self.packet = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def _read1(self, sid: int, address: int, action: str) -> int:
        value, result, error = self._txrx(action, "read1ByteTxRx", sid, address)
        self._check(result, error, action)
        return value

    def _write1(self, sid: int, address: int, value: int, action: str) -> None:
        result, error = self._txrx(action, "write1ByteTxRx", sid, address, value)
        self._check(result, error, action)

    def _write2(self, sid: int, address: int, value: int, action: str) -> None:
        result, error = self._txrx(action, "write2ByteTxRx", sid, address, value)
        self._check(result, error, action)

    def verify(self, sid: int) -> int:
        action = f"ping servo id {sid}"
        model, result, error = self._txrx(action, "ping", sid)
        self._check(result, error, action)
        return model

    def configure_position_mode(self, sid: int) -> None:
        from scservo_sdk import SMS_STS_TORQUE_ENABLE

        self._write1(sid, SMS_STS_TORQUE_ENABLE, 1, "开启扭矩")
        self._write1(sid, 55, 0, "解锁舵机设置")
        self._write2(sid, 9, 0, "设置最小角度")
        self._write2(sid, 11, STEPS_PER_TURN - 1, "设置最大角度")
        self._write1(sid, 33, 0, "切换绝对位置模式")
        self._write1(sid, 55, 1, "锁定舵机设置")

    def set_current_position_as_middle(self, sid: int) -> None:
        """使用舵机的置中指令，将当前安装角度设为 2048。"""
        from scservo_sdk import SMS_STS_TORQUE_ENABLE

        self._write1(sid, SMS_STS_TORQUE_ENABLE, 128, "将当前位置设为中位")
        time.sleep(0.05)
        position, result, error = self._txrx("确认中位设置", "ReadPos", sid)
        self._check(result, error, "确认中位设置")
        if not 2044 <= position <= 2052:
            raise UmiError("舵机未能将当前位置设为中位，请检查舵机型号和通信。")

    def change_id(self, old_id: int, new_id: int) -> None:
        self._write1(old_id, 55, 0, "解锁舵机设置")
        self._write1(old_id, 5, new_id, "写入新 ID")
        self._write1(new_id, 55, 1, "锁定舵机设置")
        self.verify(new_id)

    def check_position_mode(self, sid: int) -> None:
        if self._read1(sid, 33, "read servo mode") != 0:
            raise UmiError(
                f"servo id {sid} is not in absolute position mode; run umi_gripper calibration"
            )

    def prepare(self, sid: int, strength: int) -> None:
        from scservo_sdk import SMS_STS_TORQUE_ENABLE

        self.set_grip_strength(sid, strength)
        self._write1(sid, SMS_STS_TORQUE_ENABLE, 1, "enable torque")

    def set_grip_strength(self, sid: int, strength: int) -> None:
        self._write2(sid, 48, strength * 10, "set grip strength")

    def set_position(self, sid: int, position: int, speed: int, accel: int = 50) -> None:
        # WritePosEx packs accel, goal position, PWM open-loop speed 0, run speed.
        # Address 44 is unused in position mode; it is not a move time.
        # Accel 0 means maximum acceleration.
        result, error = self._txrx(
            "write target position", "WritePosEx", sid, position, speed, accel
        )
        self._check(result, error, "write target position")

    def telemetry(self, sid: int) -> dict:
        from scservo_sdk import COMM_SUCCESS

        data, result, _error = self._txrx(
            "read feedback", "readTxRx", sid, _PRESENT_POSITION, _FEEDBACK_LENGTH
        )
        if result != COMM_SUCCESS:
            raise UmiError(_comm_message("read feedback", result), code=result)
        # The packet error byte repeats address 65. The measured bytes are still valid.
        return decode_status_block(list(data))
