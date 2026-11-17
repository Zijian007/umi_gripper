"""Exercise real controller threading and cleanup with an injected bus, no SDK."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fake_bus import FakeServoBus
from gripper import Gripper, GripperError, default_servo_config


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
