"""Calibration, trial runs and ID changes through the operator commands."""

import json
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import replace
from io import StringIO
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import control_gripper
from tests.support import FakeServoBus, fake_bus_at_midpoint, write_calibration
from umi_gripper import UmiError, UmiReading, config
from umi_gripper import session as umi_session
from umi_gripper.cli import calibrate, demo, discovery, identifier
from umi_gripper.gripper import Gripper


class UmiCliTest(unittest.TestCase):
    def test_direct_control_addresses_only_the_selected_servo(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "calibration.json", channels=(0, 1))
            calibrated = json.loads(path.read_text())
            calibrated["servos"][1]["servo_id"] = 0
            path.write_text(json.dumps(calibrated))
            bus = fake_bus_at_midpoint("/dev/ttyUMI_TEST", 1_000_000)
            with (
                patch.object(umi_session, "ServoBus", return_value=bus),
                patch.object(config, "_read_json", wraps=config._read_json) as read_json,
                redirect_stdout(StringIO()),
            ):
                self.assertEqual(control_gripper.main(["--id", "0", "close", "--config", str(path)]), 0)
                read_json.assert_called_once_with(path)
                self.assertEqual(control_gripper.main(["--channel", "0", "open", "--config", str(path)]), 0)
            self.assertEqual(bus.positions, {0: 2000, 1: 1000})
            self.assertFalse(bus.opened)

            calibrated["servos"][0]["servo_id"] = 0
            path.write_text(json.dumps(calibrated))
            stderr = StringIO()
            with patch.object(umi_session, "ServoBus") as factory, redirect_stderr(stderr):
                self.assertEqual(control_gripper.main(["--id", "0", "close", "--config", str(path)]), 1)
                factory.assert_not_called()
            self.assertIn("对应多个标定通道", stderr.getvalue())

    def test_status_log_skips_idle_noise_and_shows_motion_and_faults(self):
        ready = UmiReading(None, 1.0, "ready", "ready", 0.0, 25.0, 0.0, 2481)
        moving = replace(ready, state="moving", message="moving", raw_position=2483)
        fault = replace(moving, state="error", message="no status packet (-6)", raw_position=2490)
        readings = [
            ready,
            ready,
            replace(ready, raw_position=2482, current=0.04, effort=2.0),
            moving,
            replace(moving, raw_position=2490),
            fault,
            fault,
            replace(ready, raw_position=2490),
        ]
        gripper = Mock()
        gripper.read.side_effect = readings
        stopped = Mock()
        stopped.wait.side_effect = [False] * len(readings) + [True]
        output = StringIO()

        with redirect_stdout(output):
            calibrate._display_status({0: gripper}, stopped)

        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 5)
        self.assertIn("位置 2481", lines[0])
        self.assertIn("状态 moving", lines[1])
        self.assertIn("位置 2490", lines[2])
        self.assertIn("no status packet (-6)", lines[3])
        self.assertIn("状态 ready", lines[4])

    def test_calibration_step_and_new_limits_use_the_open_servo(self):
        port = "/dev/ttyUMI_TEST"
        selected = {"port": port, "baudrate": 1_000_000, "servo_id": 3}
        bus = FakeServoBus(port, 1_000_000)
        with patch.object(umi_session, "ServoBus", return_value=bus) as factory:
            with Gripper._open_for_calibration(0, selected) as device:
                self.assertEqual(bus.commands[:3], [("position_mode", 3), ("set_middle", 3), ("strength", 3, 40)])
                self.assertIsNone(device.read().close_01)
                calibrate.manual_step(device, 1)
                self.assertEqual(device.wait(timeout=2).raw_position, 2176)
                reader = MagicMock()
                reader.__enter__.return_value.read.return_value = "\n"
                output = StringIO()
                with patch.object(calibrate, "_KeyReader", return_value=reader), redirect_stdout(output):
                    self.assertEqual(calibrate.calibrate_position(device, "中立位置", 2), 2176)
                self.assertIn("左夹爪（通道 0） 第 2/3 步，标定中立位置", output.getvalue())

                device._set_calibration({"closed_position_steps": 2300, "neutral_position_steps": 2048,
                                         "open_position_steps": 1700})
                device._set_grip_strength("close_grip_strength", 55)
                device.set_target(0.75)
                self.assertAlmostEqual(device.wait(timeout=2).close_01, 0.75)
                self.assertEqual(device.device_info()["runtime"]["close_grip_strength"], 55)
                factory.assert_called_once()
        self.assertFalse(bus.opened)

    def test_scan_falls_back_per_port_and_finds_every_id_at_its_baudrate(self):
        fast = "/dev/ttyUMI_FAST"
        slow = "/dev/ttyUMI_SLOW"
        responders = {(fast, 1_000_000): {3}, (slow, 115_200): {5, 6}}
        handlers = {}

        def make_port(device):
            handler = Mock(openPort=Mock(return_value=True), device=device, baudrate=None)

            def set_baudrate(baudrate):
                handler.baudrate = baudrate
                return True

            handler.setBaudRate.side_effect = set_baudrate
            handlers[device] = handler
            return handler

        def make_packet(handler):
            def ping(servo_id):
                if servo_id in responders.get((handler.device, handler.baudrate), ()):
                    return 777, 0, 0
                return 0, -6, 0

            return types.SimpleNamespace(ping=ping)

        serial = types.ModuleType("serial")
        serial.__path__ = []
        serial_tools = types.ModuleType("serial.tools")
        ports = [types.SimpleNamespace(device=device) for device in (fast, slow)]
        serial_tools.list_ports = types.SimpleNamespace(comports=Mock(return_value=ports))
        sdk = types.SimpleNamespace(COMM_SUCCESS=0, PortHandler=make_port, sms_sts=make_packet)

        with patch.dict(sys.modules, {"serial": serial, "serial.tools": serial_tools, "scservo_sdk": sdk}):
            found = discovery.discover_servos((1_000_000, 115_200))

        self.assertEqual(
            [(item["port"], item["baudrate"], item["servo_id"]) for item in found],
            [(fast, 1_000_000, 3), (slow, 115_200, 5), (slow, 115_200, 6)],
        )
        handlers[fast].setBaudRate.assert_called_once_with(1_000_000)
        for handler in handlers.values():
            handler.openPort.assert_called_once()
            handler.closePort.assert_called_once()

    def test_calibration_demo_and_gripper_share_the_default_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "umi_gripper"
            root.mkdir()
            (root / "pyproject.toml").touch()
            config_module = root / "umi_gripper" / "config.py"
            path = root / "gripper_config.json"
            found = [
                {"port": "/dev/ttyUMI_TEST", "servo_id": sid, "baudrate": 1_000_000, "model": 777}
                for sid in (3, 5)
            ]
            output = StringIO()
            with (
                patch.object(config, "__file__", str(config_module)),
                patch.object(sys, "argv", ["umi-calibrate"]),
                patch.object(umi_session, "ServoBus", side_effect=FakeServoBus) as factory,
                patch.object(calibrate, "discover_servos", return_value=found),
                patch.object(calibrate, "calibrate_position", side_effect=[2300, 2048, 1700] * 2),
                patch("builtins.input", side_effect=["", "1", "", "1", "2", "q", "q"]),
                redirect_stdout(output),
            ):
                calibrate.cli()
                factory.assert_called_once()
            saved = json.loads(path.read_text())["servos"]
            self.assertEqual([item["servo_id"] for item in saved], [3, 5])
            self.assertEqual([item["closed_position_steps"] for item in saved], [2300, 2300])
            self.assertIn("选项 1 | 舵机 ID 3", output.getvalue())
            self.assertIn("请先输入选项号，再按 Enter。", output.getvalue())
            self.assertIn("左夹爪（通道 0）已选择舵机 ID 3", output.getvalue())
            self.assertIn("右夹爪（通道 1）已选择舵机 ID 5", output.getvalue())
            self.assertIn("同一串口和 ID 不能分配给两个通道", output.getvalue())

            bus = FakeServoBus("/dev/ttyUMI_TEST", 1_000_000)
            with (
                patch.object(config, "__file__", str(config_module)),
                patch.object(sys, "argv", ["umi-demo"]),
                patch.object(config, "_read_json", wraps=config._read_json) as read_json,
                patch.object(umi_session, "ServoBus", return_value=bus) as factory,
                patch("builtins.input", side_effect=["0", "3", "80", "0.8", "1", "s", "5", "0.2", "q"]),
                redirect_stdout(output),
            ):
                demo.cli()
                factory.assert_called_once()
                read_json.assert_called_once_with(path)
                with Gripper.open(channel=1) as gripper:
                    self.assertEqual(gripper.device_info()["calibration"]["servo_id"], 5)
            self.assertIn("请选择列表里的输入值。", output.getvalue())
            self.assertIn("请输入 0 到 1 的开度", output.getvalue())
            self.assertIn("目标开度 80.0% | 实测开度 80.0% | 已到位", output.getvalue())
            self.assertIn("目标开度 100.0% | 实测开度 100.0% | 已到位", output.getvalue())
            self.assertIn("目标开度 20.0% | 实测开度 20.0% | 已到位", output.getvalue())
            self.assertEqual(bus.positions, {3: 1700, 5: 2180})
            self.assertFalse(bus.opened)

    def test_recalibrating_one_side_preserves_the_other_side(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "calibration.json", channels=(0, 1))
            original = json.loads(path.read_text())
            original["rig"] = "dual"
            path.write_text(json.dumps(original))
            found = [{"port": "/dev/ttyUMI_TEST", "servo_id": 1, "baudrate": 1_000_000, "model": 777}]
            with (
                patch.object(umi_session, "ServoBus", side_effect=FakeServoBus),
                patch.object(calibrate, "discover_servos", return_value=found),
                patch.object(calibrate, "calibrate_position", side_effect=[2300, 2048, 1700]),
                patch("builtins.input", side_effect=["", "1", "0", "q"]),
                redirect_stdout(StringIO()),
            ):
                calibrate.main(config_path=path)
            saved = json.loads(path.read_text())
            self.assertEqual(saved["servos"][0]["closed_position_steps"], 2300)
            self.assertEqual(saved["servos"][1], original["servos"][1])
            self.assertEqual(saved["rig"], "dual")

    def test_failed_atomic_replace_leaves_original_calibration_readable(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "calibration.json", channels=(0, 1))
            original = path.read_bytes()
            left = config.load_servo_config(path, 0, {"speed": 800})
            with patch.object(config.os, "replace", side_effect=OSError("failed replace")):
                with self.assertRaisesRegex(UmiError, "failed replace"):
                    config.save_calibration_channel(path, 0, left)
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(list(path.parent.glob(".calibration.json.*")), [path.with_name(".calibration.json.lock")])

    def test_demo_disambiguates_a_servo_id_shared_between_ports(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_calibration(Path(tmp) / "calibration.json", channels=(0, 1))
            saved = json.loads(path.read_text())
            right_port = "/dev/ttyUMI_RIGHT"
            saved["servos"][1].update(servo_id=1, serial_port=right_port)
            path.write_text(json.dumps(saved))
            bus = fake_bus_at_midpoint(right_port, 1_000_000)
            output = StringIO()
            with (
                patch.object(umi_session, "ServoBus", return_value=bus) as factory,
                patch("builtins.input", side_effect=["1", "1 1", "0.8", "q"]),
                redirect_stdout(output),
            ):
                demo.main(path)

            factory.assert_called_once_with(right_port, 1_000_000)
            self.assertIn("输入 1 1  右夹爪（舵机 ID 1，通道 1）", output.getvalue())
            self.assertIn("请选择列表里的输入值。", output.getvalue())
            self.assertEqual(bus.positions, {1: 1200})
            self.assertFalse(bus.opened)

    def test_identifier_uses_the_bus_for_the_confirmed_id_change(self):
        found = [{"port": "/dev/ttyUMI_TEST", "servo_id": 1, "baudrate": 1_000_000, "model": 777}]
        bus = MagicMock()
        bus.__enter__.return_value = bus
        with (
            patch.object(identifier, "discover_servos", return_value=found),
            patch.object(identifier, "ServoBus", return_value=bus),
            patch("builtins.input", side_effect=["", "0", "1", "7", "q"]),
            redirect_stdout(StringIO()),
        ):
            identifier.main()
        bus.verify.assert_called_once_with(1)
        bus.change_id.assert_called_once_with(1, 7)
