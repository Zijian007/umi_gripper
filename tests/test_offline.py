"""Exercise real controller threading and cleanup with an injected bus, no SDK."""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fake_bus import FakeServoBus
from gripper import (
    ConfigError,
    Gripper,
    GripperError,
    GripperReading,
    decode_status_block,
    default_servo_config,
)


def channel(sid=1, port="fake"):
    config = default_servo_config()
    config.update(serial_port=port, servo_id=sid, baudrate=1000000,
                  closed_position_steps=1000, neutral_position_steps=2048,
                  open_position_steps=3000, status_frequency_hz=100)
    return config


class ControllerTest(unittest.TestCase):
    def make_gripper(self, servos):
        gripper = Gripper(config={"servos": servos}, bus_factory=FakeServoBus)
        self.addCleanup(gripper.disconnect)
        return gripper

    def test_move_and_confirm_arrival(self):
        gripper = self.make_gripper([channel(), None])
        gripper.goto(0.8)
        self.assertEqual(gripper.wait(timeout=2)["state"], "已到位")
        self.assertAlmostEqual(gripper.get_width(), 0.8, places=2)

    def test_read_returns_structured_feedback_with_stable_status(self):
        gripper = self.make_gripper([channel(), None])
        reading = gripper.read()
        self.assertIsInstance(reading, GripperReading)
        self.assertEqual(reading.status, "ready")
        self.assertEqual(reading.position_steps, 2048)
        self.assertGreater(reading.sampled_at, 0.0)
        self.assertAlmostEqual(reading.current_a, 0.0)

    def test_state_telemetry_is_a_snapshot(self):
        gripper = self.make_gripper([channel(), None])
        state = gripper.get_state()
        state["telemetry"]["raw_position"] = 0
        self.assertEqual(gripper.read().position_steps, 2048)

    def test_named_open_close_and_channel_stop(self):
        gripper = self.make_gripper([channel(), None])
        gripper.close_gripper()
        self.assertEqual(gripper.wait(timeout=2)["status"], "arrived")
        self.assertAlmostEqual(gripper.get_width(), 0.0, places=2)
        gripper.open_gripper()
        self.assertEqual(gripper.wait(timeout=2)["status"], "arrived")
        self.assertAlmostEqual(gripper.get_width(), 1.0, places=2)
        gripper.goto(0.4)
        stopped = gripper.stop()
        self.assertEqual(stopped.status, "ready")

    def test_configured_acceleration_is_sent_and_feedback_has_timestamp(self):
        config = channel()
        config["accel"] = 7
        gripper = self.make_gripper([config, None])
        self.assertGreater(gripper.get_state()["sampled_at"], 0.0)
        gripper.goto(0.8)
        self.assertEqual(gripper.wait(timeout=2)["state"], "已到位")
        position_commands = [
            command for command in gripper.buses["fake", 1000000].commands
            if command[0] == "position"
        ]
        self.assertTrue(position_commands)
        self.assertEqual(position_commands[-1][-1], 7)

    def test_status_packet_decoder_preserves_signed_units(self):
        data = [0] * 15
        data[0], data[1] = 0x11, 0x08
        data[4], data[5] = 0x38, 0x04
        data[7] = 36
        data[13], data[14] = 0x04, 0x80
        self.assertEqual(
            decode_status_block(data),
            {
                "raw_position": 2065,
                "load": -56,
                "current_ma": -26,
                "temperature_c": 36,
            },
        )

    def test_overtemperature_requires_hold_and_cooling_clears_error(self):
        config = channel()
        config["overtemp_hold_s"] = 0.05
        gripper = self.make_gripper([config, None])
        bus = gripper.buses["fake", 1000000]
        bus.temperature_c = 80
        for _ in range(60):
            if gripper.get_state()["state"] == "异常":
                break
            time.sleep(0.02)
        self.assertEqual(gripper.get_state()["state"], "异常")
        bus.temperature_c = 60
        # A subsequent good sample clears the latched error without resuming a target.
        for _ in range(20):
            if gripper.get_state()["state"] == "就绪":
                break
            time.sleep(0.02)
        self.assertEqual(gripper.get_state()["state"], "就绪")

    def test_good_sample_clears_communication_error_without_resuming_motion(self):
        gripper = self.make_gripper([channel(), None])
        bus = gripper.buses["fake", 1000000]
        bus.telemetry_error = GripperError("无状态包", code=-6)
        for _ in range(20):
            if gripper.get_state()["state"] == "异常":
                break
            time.sleep(0.02)
        self.assertEqual(gripper.get_state()["state"], "异常")
        self.assertEqual(gripper.get_state()["error_code"], -6)
        bus.telemetry_error = None
        for _ in range(20):
            if gripper.get_state()["state"] == "就绪":
                break
            time.sleep(0.02)
        self.assertEqual(gripper.get_state()["state"], "就绪")
        self.assertEqual(gripper.get_state()["error_code"], 0)

    def test_invalid_overtemperature_configuration_is_rejected(self):
        config = channel()
        config["overtemp_clear_c"] = config["overtemp_c"]
        with self.assertRaises(ConfigError):
            Gripper(config={"servos": [config, None]}, bus_factory=FakeServoBus)

    def test_shared_port_uses_one_bus_and_requires_channel(self):
        gripper = self.make_gripper([channel(1), channel(2)])
        self.assertEqual(len(gripper.buses), 1)
        with self.assertRaises(GripperError):
            gripper.goto(.5)
        gripper.goto(.2, 1)
        self.assertEqual(gripper.wait(channel=1, timeout=2)["state"], "已到位")

    def test_disconnect_stops_worker_and_closes_bus(self):
        gripper = self.make_gripper([channel(), None])
        bus = next(iter(gripper.buses.values()))
        gripper.disconnect()
        gripper.disconnect()
        self.assertFalse(gripper.worker.is_alive())
        self.assertFalse(bus.opened)

    def test_failed_initial_read_rolls_back(self):
        buses = []

        def factory(*args):
            bus = FakeServoBus(*args)
            bus.telemetry_error = GripperError("injected read failure")
            buses.append(bus)
            return bus

        with self.assertRaisesRegex(GripperError, "injected read failure"):
            Gripper(config={"servos": [channel(), None]}, bus_factory=factory)
        self.assertFalse(buses[0].opened)


if __name__ == "__main__":
    unittest.main()
