"""Shared buses, worker state and cleanup."""

import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from tests.support import FakeServoBus, fake_bus_at_midpoint, write_calibration
from umi_gripper import Gripper, UmiError, session as umi_session
from umi_gripper.bus import ServoBus, decode_status_block
from umi_gripper.cli.identifier import change_servo_id
from umi_gripper.config import default_servo_config


def _telemetry(temperature_c: int) -> dict:
    return {
        "raw_position": 1500,
        "load": 0,
        "current_ma": 0,
        "temperature_c": temperature_c,
    }


def _session_with_channel():
    session = umi_session._UmiSession("/dev/ttyUMI_TEST", 1_000_000)
    item = umi_session._Channel(
        index=1,
        servo_id=1,
        config={
            **default_servo_config(),
            "closed_position_steps": 2000,
            "open_position_steps": 1000,
            "neutral_position_steps": 1500,
            "speed": 1200,
            "servo_id": 1,
        },
        position=1500,
        position_valid=True,
        state="ready",
        message="ready",
    )
    session.bus = Mock()
    session.bus.telemetry.return_value = _telemetry(25)
    item.initial_ready.set()
    item.stopped.set()
    session.channels[1] = item
    return session, item


class UmiOvertempTest(unittest.TestCase):
    def test_one_hot_sample_does_not_stop_and_cool_clears_the_timer(self):
        session, item = _session_with_channel()
        clock = {"t": 10.0}
        session.bus.telemetry.return_value = _telemetry(80)
        with patch.object(umi_session.time, "monotonic", lambda: clock["t"]):
            session._update(item)
            self.assertIsNone(item.pending_stop)
            self.assertEqual(item.message, "ready")
            clock["t"] = 10.2
            session.bus.telemetry.return_value = _telemetry(36)
            session._update(item)
            self.assertIsNone(item.overtemp_since)
            self.assertFalse(item.overtemp_latched)

    def test_held_overtemp_stops_then_clears_below_65(self):
        session, item = _session_with_channel()
        clock = {"t": 10.0}
        session.bus.telemetry.return_value = _telemetry(80)
        with patch.object(umi_session.time, "monotonic", lambda: clock["t"]):
            session._update(item)
            clock["t"] = 10.5
            session._update(item)
            self.assertIsNotNone(item.pending_stop)
            session._update(item)
            self.assertEqual(item.state, "error")
            self.assertEqual(item.message, "servo overtemperature")
            self.assertIsNone(item.pending_stop)
            session.bus.telemetry.return_value = _telemetry(66)
            session._update(item)
            self.assertEqual(item.state, "error")
            self.assertTrue(item.overtemp_latched)
            session.bus.telemetry.return_value = _telemetry(36)
            session._update(item)
            self.assertEqual(item.state, "ready")
            self.assertEqual(item.message, "ready")
            self.assertFalse(item.overtemp_latched)

    def test_configured_trip_replaces_the_default_70c(self):
        session, item = _session_with_channel()
        item.config["overtemp_c"] = 90
        item.config["overtemp_clear_c"] = 80
        item.config["overtemp_hold_s"] = 0.5
        clock = {"t": 10.0}
        session.bus.telemetry.return_value = _telemetry(80)
        with patch.object(umi_session.time, "monotonic", lambda: clock["t"]):
            session._update(item)
            clock["t"] = 11.0
            session._update(item)
            self.assertIsNone(item.pending_stop)
            self.assertEqual(item.message, "ready")


class UmiCommandTest(unittest.TestCase):
    def test_repeated_target_preserves_stall_timer_and_blocked_result(self):
        session, item = _session_with_channel()
        clock = [10.0]
        with patch.object(umi_session.time, "monotonic", lambda: clock[0]):
            session.set_close_01(1, 0.0)
            session._update(item)
            for step in range(1, 7):
                clock[0] = 10.0 + step / 10
                session.set_close_01(1, 0.0001)  # Rounds to the same position.
                session._update(item)
            self.assertEqual(session.reading(1).state, "blocked")
            self.assertEqual(session.bus.set_position.call_count, 2)  # Target, then hold.

            session.set_close_01(1, 0.0, retry=True)
            session._update(item)
            self.assertEqual(session.reading(1).state, "moving")
            self.assertEqual(session.bus.set_position.call_count, 3)
            session.stop_channel(1, timeout=0)
            session._update(item)
            session.set_close_01(1, 0.0)
            session._update(item)
            self.assertEqual(session.bus.set_position.call_count, 5)

    def test_old_arrival_keeps_new_target(self):
        session, item = _session_with_channel()
        session.set_close_01(1, 0.5)
        old_sample = session._ingest(item)
        session.set_close_01(1, 1.0)

        session._follow_target(item, old_sample)

        self.assertEqual(item.target, 2000)
        self.assertIsNone(item.pending_stop)
        session._update(item)
        session.bus.set_position.assert_called_once_with(1, 2000, 1200, item.config["accel"])

    def test_old_stall_cannot_stop_or_hold_a_new_target(self):
        for direction, old_target, new_target, new_steps, expected_state in (
            ("open", 0.0, 1.0, 2000, "grasped"),
            ("close", 1.0, 0.0, 1000, "blocked"),
        ):
            with self.subTest(direction=direction):
                session, item = _session_with_channel()
                clock = [10.0]
                with patch.object(umi_session.time, "monotonic", lambda: clock[0]):
                    session.set_close_01(1, old_target)
                    session._update(item)
                    old_sample = session._ingest(item)
                    session.set_close_01(1, new_target)
                    session._update(item)
                    clock[0] += 1.0

                    session._watch_stall(item, old_sample, direction)

                    self.assertEqual(item.target, new_steps)
                    self.assertEqual(item.state, "moving")
                    self.assertIsNone(item.blocked_direction)
                    self.assertIsNone(item.pending_stop)
                    session._update(item)
                    session._update(item)
                    self.assertEqual(item.state, expected_state)

    def test_pending_stop_finishes_before_new_target_is_sent(self):
        session, item = _session_with_channel()
        session.stop_channel(1, timeout=0)
        session.set_close_01(1, 1.0)

        session._update(item)
        session.bus.set_position.assert_called_once_with(1, 1500, 1200, item.config["accel"])
        self.assertIsNone(item.pending_stop)
        self.assertEqual(item.state, "moving")
        self.assertEqual(item.target, 2000)
        session._update(item)
        session.bus.set_position.assert_called_with(1, 2000, 1200, item.config["accel"])

    def test_stop_write_does_not_complete_a_newer_stop(self):
        session, item = _session_with_channel()
        session.stop_channel(1, timeout=0)
        first_stop = item.pending_stop
        session.bus.set_position.side_effect = lambda *_: session.stop_channel(1, timeout=0)

        session._update(item)

        self.assertIsNotNone(item.pending_stop)
        self.assertIsNot(item.pending_stop, first_stop)
        self.assertFalse(item.stopped.is_set())
        session.bus.set_position.side_effect = None
        session._update(item)
        self.assertIsNone(item.pending_stop)
        self.assertTrue(item.stopped.is_set())
        self.assertEqual(item.state, "ready")

    def test_invalid_initial_position_reports_the_feedback_error(self):
        session, item = _session_with_channel()
        item.initial_ready.clear()
        session.bus.telemetry.return_value = {**_telemetry(25), "raw_position": 2500}

        session._update(item)

        with self.assertRaisesRegex(UmiError, "position 2500 outside calibration") as error:
            session._wait_initial(item)
        self.assertEqual(item.state, "error")
        self.assertEqual(item.message, str(error.exception))
        session.bus.set_position.assert_not_called()

    def test_grip_strength_timeout_cancels_a_queued_write(self):
        session, item = _session_with_channel()
        original_strength = item.config["close_grip_strength"]
        with patch.object(umi_session, "_STRENGTH_UPDATE_TIMEOUT_S", 0):
            with self.assertRaisesRegex(UmiError, "timed out before the write started"):
                session.set_grip_strength(1, "close_grip_strength", 55)

        session._apply_pending_strength(item)
        session.bus.set_grip_strength.assert_not_called()
        self.assertEqual(item.config["close_grip_strength"], original_strength)

    def test_in_flight_strength_write_does_not_consume_the_next_request(self):
        session, item = _session_with_channel()
        write_started = threading.Event()
        finish_write = threading.Event()

        def write_strength(_sid, strength):
            if strength == 55:
                write_started.set()
                if not finish_write.wait(2):
                    raise UmiError("test did not release the first write")

        session.bus.set_grip_strength.side_effect = write_strength
        worker = threading.Thread(target=session._apply_pending_strength, args=(item,))

        def start_first_write():
            worker.start()
            self.assertTrue(write_started.wait(2))

        def finish_first_write():
            finish_write.set()
            if worker.ident is not None:
                worker.join(2)
                self.assertFalse(worker.is_alive())

        self.addCleanup(finish_first_write)
        with (
            patch.object(session.wakeup, "set", side_effect=start_first_write),
            patch.object(umi_session, "_STRENGTH_UPDATE_TIMEOUT_S", 0),
        ):
            with self.assertRaisesRegex(UmiError, "write may still complete"):
                session.set_grip_strength(1, "close_grip_strength", 55)

        def apply_next_request():
            finish_first_write()
            self.assertEqual(item.config["close_grip_strength"], 55)
            session._apply_pending_strength(item)

        with patch.object(session.wakeup, "set", side_effect=apply_next_request):
            session.set_grip_strength(1, "close_grip_strength", 60)
        self.assertEqual(item.config["close_grip_strength"], 60)
        session.bus.set_grip_strength.assert_called_with(1, 60)

    def test_set_close_01_targets_closed_steps(self):
        session, item = _session_with_channel()
        gripper = Gripper(session, 1, 1)
        gripper.set_target(1.0)
        self.assertEqual(item.target, 2000)
        gripper.set_target(0.0)
        self.assertEqual(item.target, 1000)


class UmiSessionSharingTest(unittest.TestCase):
    def test_port_aliases_share_one_worker_across_calibrations(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            port = root / "ttyACM0"
            port.touch()
            alias = root / "usb-drive"
            alias.symlink_to(port)
            left_config = write_calibration(root / "left.json", channels=(0,), port=alias)
            right_config = write_calibration(root / "right.json", channels=(1,), port=port)
            bus = FakeServoBus(str(port), 1_000_000, initial_position=1500)
            with patch.object(umi_session, "ServoBus", return_value=bus) as factory:
                with (
                    Gripper.open(config_path=left_config, channel=0) as left,
                    Gripper.open(config_path=right_config, channel=1) as right,
                ):
                    factory.assert_called_once_with(str(port), 1_000_000)
                    worker = left._session.worker
                    self.assertIs(worker, right._session.worker)
                    left.device_info()["calibration"]["servo_id"] = 2
                    left.set_target(0.0)
                    self.assertEqual(left.wait().close_01, 0.0)
                    self.assertEqual(right.read().close_01, 0.5)
                    left.close()
                    self.assertTrue(bus.opened)
                    right.set_target(1.0)
                    self.assertEqual(right.wait().close_01, 1.0)
                self.assertFalse(bus.opened)
                self.assertFalse(worker.is_alive())
                self.assertNotIn(str(port), umi_session._sessions)

    def test_conflicting_servo_or_baudrate_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = write_calibration(root / "g.json")
            duplicate = write_calibration(root / "duplicate.json")
            wrong_baud = write_calibration(root / "baud.json", channels=(0,), baudrate=57_600)
            with patch.object(umi_session, "ServoBus", side_effect=fake_bus_at_midpoint) as factory:
                with Gripper.open(config_path=path, channel=1) as device:
                    with self.assertRaisesRegex(UmiError, "servo ID 2.*already open"):
                        Gripper.open(config_path=duplicate, channel=1)
                    with self.assertRaisesRegex(UmiError, "already open at 1000000 baud"):
                        Gripper.open(config_path=wrong_baud, channel=0)
                    factory.assert_called_once()
                    self.assertEqual(device.read().state, "ready")

    def test_stop_runs_before_the_next_slow_feedback_poll(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "g.json")
            bus = fake_bus_at_midpoint("/dev/ttyUMI_TEST", 1_000_000)
            with patch.object(umi_session, "ServoBus", return_value=bus):
                with Gripper.open(config_path=path, channel=1, settings={"status_frequency_hz": 1}) as device:
                    device.stop()
                    self.assertEqual(bus.commands[-1], ("position", 2, 1500, 1200, 50))

    def test_maintenance_cannot_change_an_id_on_an_open_control_port(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "g.json")
            with patch.object(umi_session, "ServoBus", side_effect=fake_bus_at_midpoint):
                with Gripper.open(config_path=path, channel=1):
                    with self.assertRaisesRegex(UmiError, "already in use"):
                        change_servo_id("/dev/ttyUMI_TEST", 1_000_000, 2, 3)

    def test_good_feedback_clears_communication_error_without_replaying_motion(self):
        def wait_for(predicate):
            deadline = time.monotonic() + 1.0
            while not predicate():
                if time.monotonic() >= deadline:
                    self.fail("worker did not reach the expected state")
                time.sleep(0.01)

        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "g.json")
            bus = fake_bus_at_midpoint("/dev/ttyUMI_TEST", 1_000_000)
            bus.stalled = True
            with patch.object(umi_session, "ServoBus", return_value=bus):
                with Gripper.open(config_path=path, channel=1) as device:
                    device.set_target(1.0)
                    wait_for(lambda: ("position", 2, 2000, 1200, 50) in bus.commands)
                    bus.telemetry_error = UmiError("no status packet (-6)", code=-6)
                    wait_for(lambda: device.read().state == "error")
                    fault = device.read()
                    self.assertIn("no status packet", fault.message)
                    self.assertFalse(fault.feedback_valid)
                    self.assertIsNone(fault.close_01)
                    self.assertEqual(fault.fault, "communication")
                    self.assertEqual(fault.fault_code, -6)
                    with self.assertRaisesRegex(UmiError, "no status packet"):
                        device.set_target(1.0)
                    commands_at_fault = list(bus.commands)

                    bus.telemetry_error = None
                    wait_for(lambda: device.read().state == "ready")
                    self.assertTrue(device.read().feedback_valid)
                    self.assertIsNone(device.read().fault)
                    self.assertEqual(bus.commands, commands_at_fault)

                    bus.telemetry_error = UmiError("unplugged")
                    wait_for(lambda: device.read().state == "error")

                self.assertFalse(bus.opened)
                bus.telemetry_error = None
                with Gripper.open(config_path=path, channel=1) as reopened:
                    self.assertEqual(reopened.read().state, "ready")


class UmiBusTest(unittest.TestCase):
    def test_status_packet_decoder_preserves_signed_units(self):
        data = [0] * 15
        data[0], data[1] = 0x11, 0x08
        data[4], data[5] = 0x38, 0x04
        data[7] = 36
        data[13], data[14] = 0x04, 0x80
        self.assertEqual(
            decode_status_block(data),
            {"raw_position": 2065, "load": -56, "current_ma": -26, "temperature_c": 36},
        )


class _BlockingPacket:
    """``sms_sts`` stand-in whose feedback read blocks until released."""

    def __init__(self) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()

    def readTxRx(self, sid, address, length):
        self.entered.set()
        self.release.wait(2.0)
        return [0] * length, 0, 0


class UmiShutdownTest(unittest.TestCase):
    def _bus(self, packet):
        port = Mock(openPort=Mock(return_value=True), setBaudRate=Mock(return_value=True))
        sdk = types.SimpleNamespace(
            PortHandler=Mock(return_value=port), sms_sts=Mock(return_value=packet), COMM_SUCCESS=0,
        )
        with patch.dict(sys.modules, {"scservo_sdk": sdk}):
            bus = ServoBus("/dev/ttyUMI_TEST", 1_000_000)
            bus.open()
            return bus, port, sdk

    def test_serial_io_error_reaches_the_session_as_invalid_feedback(self):
        packet = Mock(readTxRx=Mock(side_effect=OSError("USB serial disconnected")))
        bus, _, sdk = self._bus(packet)
        self.addCleanup(bus.close)
        session, item = _session_with_channel()
        session.bus = bus
        item.state = "arrived"
        item.sampled_at_mono_s = 12.0
        with patch.dict(sys.modules, {"scservo_sdk": sdk}):
            session._update(item)
            reading = session.reading(1)
            self.assertEqual(reading.state, "error")
            self.assertEqual(reading.fault, "communication")
            self.assertFalse(reading.feedback_valid)
            self.assertEqual(reading.sampled_at_mono_s, 12.0)
            self.assertIn("USB serial disconnected", reading.message)
            packet.readTxRx.side_effect = None
            packet.readTxRx.return_value = [220, 5] + [0] * 13, 0, 0
            session._update(item)
            reading = session.reading(1)
            self.assertEqual(reading.state, "ready")
            self.assertTrue(reading.feedback_valid)
            packet.WritePosEx.assert_not_called()

    def test_bus_close_waits_for_in_flight_transaction_then_refuses_io(self):
        packet = _BlockingPacket()
        bus, port, sdk = self._bus(packet)
        with patch.dict(sys.modules, {"scservo_sdk": sdk}):
            reader = threading.Thread(target=bus.telemetry, args=(2,), daemon=True)
            reader.start()
            self.assertTrue(packet.entered.wait(1.0))
            closer = threading.Thread(target=bus.close, daemon=True)
            closer.start()
            closer.join(0.05)
            self.assertTrue(closer.is_alive())  # waits for the read on the wire
            port.closePort.assert_not_called()
            packet.release.set()
            closer.join(1.0)
            reader.join(1.0)
            port.closePort.assert_called_once_with()
            with self.assertRaisesRegex(UmiError, "not open"):
                bus.telemetry(2)
        bus.close()  # idempotent
        port.closePort.assert_called_once_with()

    def test_close_keeps_resources_until_worker_and_bus_are_closed(self):
        busy = Mock(side_effect=[False, True])
        session = umi_session._UmiSession("/dev/ttyUMI_TEST", 1_000_000)
        session.worker = Mock()
        bus = Mock(close=Mock(side_effect=[OSError("port busy"), None]))
        item = umi_session._Channel(index=1, servo_id=1, config={}, stopped=Mock())
        device = Gripper(session, 1, 1)
        session.bus = bus
        session.channels[1] = item
        with patch.object(session, "_join_worker", busy):
            with self.assertRaisesRegex(UmiError, "retry close"):
                device.close()
            bus.close.assert_not_called()
            with self.assertRaisesRegex(UmiError, "cleanup is pending"):
                session.open_channel(1, {})
            with self.assertRaisesRegex(OSError, "port busy"):
                device.close()
            device.close()
            device.close()
        self.assertEqual(bus.close.call_count, 2)
        self.assertEqual(busy.call_count, 2)
        self.assertTrue(session._closed)

    def test_failed_initial_feedback_releases_the_port(self):
        bus = fake_bus_at_midpoint("/dev/ttyUMI_TEST", 1_000_000)
        bus.telemetry_error = UmiError("injected read failure")
        with tempfile.TemporaryDirectory() as temp, patch.object(umi_session, "ServoBus", return_value=bus):
            path = write_calibration(Path(temp) / "gripper.json")
            with self.assertRaisesRegex(UmiError, "injected read failure"):
                Gripper.open(config_path=path, channel=1)
        self.assertFalse(bus.opened)
        self.assertNotIn(bus.port_name, umi_session._sessions)

    def test_failed_open_exposes_pending_close_to_the_caller(self):
        bus = Mock(
            verify=Mock(side_effect=UmiError("servo missing")),
            close=Mock(side_effect=[OSError("port busy"), None]),
        )
        with tempfile.TemporaryDirectory() as temp, patch.object(umi_session, "ServoBus", return_value=bus):
            path = write_calibration(Path(temp) / "gripper.json")
            with self.assertRaisesRegex(UmiError, "cleanup incomplete") as failed:
                Gripper.open(config_path=path, channel=1)
            with self.assertRaisesRegex(UmiError, "cleanup is pending") as blocked:
                Gripper.open(config_path=path, channel=1)
            self.assertIsNone(blocked.exception.resource)
            failed.exception.resource.close()
            self.assertEqual(bus.close.call_count, 2)
