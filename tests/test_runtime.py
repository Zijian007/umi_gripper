"""Standalone UMI runtime, configuration, hardware feedback and shutdown."""

import json
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from umi_gripper import UmiError, UmiGripper
from umi_gripper import runtime as umi_gripper
from umi_gripper.bus import ServoBus
from umi_gripper.config import default_servo_config, load_servo_config


def _calibration_json(path: Path, *, channels=(1,)) -> Path:
    servos = [None, None]
    for channel in channels:
        servos[channel] = {
            "serial_port": "/dev/ttyUMI_TEST",
            "servo_id": channel + 1,
            "baudrate": 1_000_000,
            "closed_position_steps": 2000,
            "neutral_position_steps": 1500,
            "open_position_steps": 1000,
            "speed": 1200,
            "open_grip_strength": 40,
            "close_grip_strength": 40,
            "status_frequency_hz": 50,
            "calibration_step_steps": 128,
            "motion_timeout_s": 30,
        }
    path.write_text(json.dumps({"servos": servos}), encoding="utf-8")
    return path


def _telemetry(temperature_c: int) -> dict:
    return {
        "raw_position": 1500,
        "load": 0,
        "current_ma": 0,
        "temperature_c": temperature_c,
    }


class UmiOvertempTest(unittest.TestCase):
    def _session(self):
        session = umi_gripper._UmiSession(Path("/tmp/unused.json"))
        item = umi_gripper._Channel(
            index=1,
            config={
                **default_servo_config(),
                "closed_position_steps": 2000,
                "open_position_steps": 1000,
                "neutral_position_steps": 1500,
                "speed": 1200,
                "servo_id": 1,
            },
            bus=Mock(),
            position=1500,
            position_valid=True,
            state="ready",
            message="ready",
        )
        item.initial_ready.set()
        item.stopped.set()
        session.channels[1] = item
        return session, item

    def test_one_hot_sample_does_not_stop_and_cool_clears_the_timer(self):
        session, item = self._session()
        clock = {"t": 10.0}
        item.bus.telemetry.return_value = _telemetry(80)
        with patch.object(umi_gripper.time, "monotonic", lambda: clock["t"]):
            session._update(item)
            self.assertFalse(item.stop_pending)
            self.assertEqual(item.message, "ready")
            clock["t"] = 10.2
            item.bus.telemetry.return_value = _telemetry(36)
            session._update(item)
            self.assertIsNone(item.overtemp_since)
            self.assertFalse(item.overtemp_latched)

    def test_held_overtemp_stops_then_clears_below_65(self):
        session, item = self._session()
        clock = {"t": 10.0}
        item.bus.telemetry.return_value = _telemetry(80)
        with patch.object(umi_gripper.time, "monotonic", lambda: clock["t"]):
            session._update(item)
            clock["t"] = 10.5
            session._update(item)
            self.assertTrue(item.stop_pending)
            session._update(item)
            self.assertEqual(item.state, "error")
            self.assertEqual(item.message, "servo overtemperature")
            self.assertFalse(item.stop_pending)
            item.bus.telemetry.return_value = _telemetry(66)
            session._update(item)
            self.assertEqual(item.state, "error")
            self.assertTrue(item.overtemp_latched)
            item.bus.telemetry.return_value = _telemetry(36)
            session._update(item)
            self.assertEqual(item.state, "ready")
            self.assertEqual(item.message, "ready")
            self.assertFalse(item.overtemp_latched)

    def test_configured_trip_replaces_the_default_70c(self):
        session, item = self._session()
        item.config["overtemp_c"] = 90
        item.config["overtemp_clear_c"] = 80
        item.config["overtemp_hold_s"] = 0.5
        clock = {"t": 10.0}
        item.bus.telemetry.return_value = _telemetry(80)
        with patch.object(umi_gripper.time, "monotonic", lambda: clock["t"]):
            session._update(item)
            clock["t"] = 11.0
            session._update(item)
            self.assertFalse(item.stop_pending)
            self.assertEqual(item.message, "ready")

class UmiPolarityTest(unittest.TestCase):
    def test_close_01_inverts_umi_width(self):
        config = {
            "closed_position_steps": 2000,
            "open_position_steps": 1000,
        }
        self.assertAlmostEqual(UmiGripper.width(config, 2000), 0.0)
        self.assertAlmostEqual(UmiGripper.width(config, 1000), 1.0)
        self.assertAlmostEqual(UmiGripper.close_01(config, 2000), 1.0)
        self.assertAlmostEqual(UmiGripper.close_01(config, 1000), 0.0)
        self.assertAlmostEqual(UmiGripper.close_01(config, 1500), 0.5)

    def test_set_close_01_targets_closed_steps(self):
        session = umi_gripper._UmiSession(Path("/tmp/unused.json"))
        item = umi_gripper._Channel(
            index=1,
            config={
                "closed_position_steps": 2000,
                "open_position_steps": 1000,
                "speed": 1200,
            },
            bus=Mock(),
            position=1500,
            position_valid=True,
        )
        session.channels[1] = item
        gripper = UmiGripper(session, 1)
        gripper.set_target(1.0)
        self.assertEqual(item.target, 2000)
        gripper.set_target(0.0)
        self.assertEqual(item.target, 1000)

class UmiFactoryTest(unittest.TestCase):
    def setUp(self):
        umi_gripper._sessions.clear()

    def tearDown(self):
        umi_gripper._sessions.clear()

    def test_uncalibrated_channel_falls_back_to_other(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _calibration_json(Path(tmp) / "g.json", channels=(1,))
            session = Mock()
            session.config_path = path
            opened: list[int] = []

            def open_channel(channel, config):
                opened.append(channel)
                self.assertEqual(config["servo_id"], 2)

            session.open_channel = open_channel
            with patch.object(umi_gripper, "_session_for", return_value=session):
                device = UmiGripper.open(
                    side="left",
                    options={"channel": 0, "config_path": str(path)},
                )
            self.assertEqual(opened, [1])
            self.assertEqual(device._channel, 1)

    def test_both_channels_missing_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "g.json"
            path.write_text(json.dumps({"servos": [None, None]}), encoding="utf-8")
            with self.assertRaisesRegex(UmiError, "also missing"):
                UmiGripper.open(
                    side="left",
                    options={"channel": 0, "config_path": str(path)},
                )

    def test_missing_json_fails(self):
        with self.assertRaisesRegex(UmiError, "not found"):
            UmiGripper.open(
                side="right",
                options={
                    "channel": 1,
                    "config_path": "/tmp/does-not-exist-umi-calib.json",
                },
            )


    def test_shared_session_close_releases_once(self):
        session = Mock()
        session.config_path = Path("/tmp/shared.json")
        left = UmiGripper(session, 0)
        right = UmiGripper(session, 1)
        left.close()
        right.close()
        self.assertEqual(session.release_channel.call_count, 2)
        session.release_channel.assert_any_call(0)
        session.release_channel.assert_any_call(1)

    def test_stop_then_same_target_can_resume(self):
        session = Mock()
        device = UmiGripper(session, 1)
        device.set_target(0.6)
        session.set_close_01.assert_called_once_with(1, 0.6)
        device.stop()
        session.stop_channel.assert_called_once_with(1)
        device.set_target(0.6)
        self.assertEqual(session.set_close_01.call_count, 2)
        device.close()

class UmiRuntimeOptionsTest(unittest.TestCase):
    def test_runtime_options_override_json_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _calibration_json(Path(tmp) / "g.json", channels=(1,))
            loaded = load_servo_config(
                path,
                1,
                {
                    "speed": 800,
                    "open_grip_strength": 20,
                    "close_grip_strength": 30,
                    "status_frequency_hz": 25,
                },
            )
            self.assertEqual(loaded["speed"], 800)
            self.assertEqual(loaded["open_grip_strength"], 20)
            self.assertEqual(loaded["close_grip_strength"], 30)
            self.assertEqual(loaded["status_frequency_hz"], 25)
            self.assertEqual(loaded["closed_position_steps"], 2000)
            self.assertEqual(loaded["accel"], 50)
            self.assertEqual(loaded["position_tolerance_steps"], 12)
            self.assertEqual(loaded["stall_movement_steps"], 3)
            self.assertEqual(loaded["opening_stall_window_s"], 0.5)
            self.assertEqual(loaded["closing_stall_window_s"], 0.2)
            self.assertEqual(loaded["overtemp_c"], 70)
            self.assertEqual(loaded["overtemp_clear_c"], 65)
            self.assertEqual(loaded["overtemp_hold_s"], 0.5)

    def test_yaml_motion_options_override_defaults_including_zero_accel(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _calibration_json(Path(tmp) / "g.json", channels=(1,))
            loaded = load_servo_config(
                path,
                1,
                {
                    "accel": 0,
                    "position_tolerance_steps": 4,
                    "stall_movement_steps": 8,
                    "opening_stall_window_s": 1.0,
                    "closing_stall_window_s": 0.4,
                    "overtemp_c": 80,
                    "overtemp_clear_c": 60,
                    "overtemp_hold_s": 1.5,
                },
            )
            self.assertEqual(loaded["accel"], 0)
            self.assertEqual(loaded["position_tolerance_steps"], 4)
            self.assertEqual(loaded["stall_movement_steps"], 8)
            self.assertEqual(loaded["opening_stall_window_s"], 1.0)
            self.assertEqual(loaded["closing_stall_window_s"], 0.4)
            self.assertEqual(loaded["overtemp_c"], 80)
            self.assertEqual(loaded["overtemp_clear_c"], 60)
            self.assertEqual(loaded["overtemp_hold_s"], 1.5)

    def test_clear_temperature_must_stay_below_the_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _calibration_json(Path(tmp) / "g.json", channels=(1,))
            with self.assertRaises(UmiError):
                load_servo_config(path, 1, {"overtemp_c": 70, "overtemp_clear_c": 70})

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
        session = umi_gripper._UmiSession(Path("/tmp/unused.json"), join_worker=busy)
        session.worker = Mock()
        bus = Mock(close=Mock(side_effect=[OSError("port busy"), None]))
        item = umi_gripper._Channel(index=1, config={}, bus=bus, ref_count=1, stopped=Mock())
        device = UmiGripper(session, 1)
        session.buses = {("/dev/fake", 1_000_000): bus}
        session.channels[1] = item
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

    def test_failed_open_exposes_pending_close_to_the_caller(self):
        bus = Mock(
            verify=Mock(side_effect=UmiError("servo missing")),
            close=Mock(side_effect=[OSError("port busy"), None]),
        )
        with tempfile.TemporaryDirectory() as temp, patch.object(umi_gripper, "ServoBus", return_value=bus):
            path = _calibration_json(Path(temp) / "gripper.json")
            options = {"channel": 1, "config_path": str(path)}
            with self.assertRaisesRegex(UmiError, "cleanup incomplete") as failed:
                UmiGripper.open(side="right", options=options)
            with self.assertRaisesRegex(UmiError, "cleanup is pending") as blocked:
                UmiGripper.open(side="right", options=options)
            self.assertIsNone(blocked.exception.resource)
            failed.exception.resource.close()
            self.assertEqual(bus.close.call_count, 2)
