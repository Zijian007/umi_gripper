"""Public gripper API and runtime settings."""

import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from unittest.mock import Mock, patch

from tests.support import fake_bus_at_midpoint, write_calibration
from umi_gripper import Gripper, UmiError, UmiReading, session as umi_session
from umi_gripper.config import load_servo_config


class UmiGripperTest(unittest.TestCase):
    def test_motion_and_feedback_with_increasing_open_position(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "g.json", closed_position_steps=1000, open_position_steps=2000)
            bus = fake_bus_at_midpoint("/dev/ttyUMI_TEST", 1_000_000)
            with patch.object(umi_session, "ServoBus", return_value=bus):
                with Gripper.open(config_path=path, channel=1, settings={"accel": 7}) as device:
                    reading = device.read()
                    self.assertIsInstance(reading, UmiReading)
                    self.assertEqual(reading.state, "ready")
                    self.assertEqual(reading.raw_position, 1500)
                    self.assertGreater(reading.sampled_at_mono_s, 0.0)
                    with self.assertRaises(FrozenInstanceError):
                        reading.raw_position = 0

                    device.set_target(0.2)
                    result = device.wait(timeout=2)
                    self.assertEqual(result.state, "arrived")
                    self.assertAlmostEqual(result.close_01, 0.2)
                    self.assertGreater(result.sampled_at_mono_s, reading.sampled_at_mono_s)
                    self.assertIn(("position", 2, 1800, 1200, 7), bus.commands)

    def test_only_requested_slot_can_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "g.json")
            with patch.object(umi_session, "ServoBus", side_effect=fake_bus_at_midpoint) as factory:
                with self.assertRaisesRegex(UmiError, "channel 0 is not calibrated"):
                    Gripper.open(config_path=path, channel=0)
                info = Gripper.validate_config(config_path=path, channel=1, settings={"speed": 800})
                factory.assert_not_called()
                with Gripper.open(config_path=path, channel=1, settings={"speed": 800}) as device:
                    self.assertEqual(info, device.device_info())
                    self.assertEqual(info["calibration"]["servo_id"], 2)
                    self.assertEqual(info["runtime"]["speed"], 800)
                    self.assertNotIn("config_path", info)
                    self.assertNotIn("calibration_sha256", info)

    def test_missing_json_fails(self):
        with self.assertRaisesRegex(UmiError, "not found"):
            Gripper.open(config_path="/tmp/does-not-exist-umi-calib.json", channel=1)

    def test_unknown_settings_fail_before_opening_the_bus(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "g.json")
            with patch.object(umi_session, "ServoBus") as factory:
                for key in ("speeed", "servo_id"):
                    with self.subTest(key=key), self.assertRaisesRegex(UmiError, f"unknown UMI settings: '{key}'"):
                        Gripper.open(config_path=path, channel=1, settings={key: 800})
                factory.assert_not_called()

    def test_stop_then_same_target_can_resume(self):
        session = Mock()
        device = Gripper(session, 1, 3)
        device.set_target(0.6)
        session.set_close_01.assert_called_once_with(3, 0.6, retry=False)
        device.stop()
        session.stop_channel.assert_called_once_with(3, timeout=0.5, allow_invalid=False)
        device.set_target(0.6)
        self.assertEqual(session.set_close_01.call_count, 2)
        device.close()

    def test_public_stop_reports_a_timeout(self):
        session = Mock()
        session.stop_channel.return_value = False
        device = Gripper(session, 1, 3)
        with self.assertRaisesRegex(UmiError, "servo ID 3 stop timed out"):
            device.stop()

    def test_wait_timeout_leaves_the_motion_for_the_caller_to_stop(self):
        session = Mock()
        session.reading.return_value.state = "moving"
        device = Gripper(session, 1, 3)
        with self.assertRaisesRegex(UmiError, "servo ID 3 motion timed out"):
            device.wait(timeout=0)
        session.stop_channel.assert_not_called()


class UmiRuntimeOptionsTest(unittest.TestCase):
    def test_runtime_options_override_json_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "g.json", channels=(1,))
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
            path = write_calibration(Path(tmp) / "g.json", channels=(1,))
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
            path = write_calibration(Path(tmp) / "g.json", channels=(1,))
            with self.assertRaises(UmiError):
                load_servo_config(path, 1, {"overtemp_c": 70, "overtemp_clear_c": 70})
