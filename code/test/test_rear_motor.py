"""Hardware-free unit tests for the C10B rear-motor component."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.rear_motor import (  # noqa: E402
    ChassisCommand,
    UnsupportedWheelCommand,
    WheelSpeeds,
    build_velocity_frame,
    wheel_speeds_to_chassis,
)


class RearMotorProtocolTests(unittest.TestCase):
    def test_verified_straight_100_frame(self) -> None:
        command = ChassisCommand(100, 0, WheelSpeeds(100, 100))
        self.assertEqual(
            build_velocity_frame(command),
            bytes.fromhex("7B 00 00 00 64 00 00 00 00 1F 7D"),
        )

    def test_linked_reverse(self) -> None:
        command = wheel_speeds_to_chassis(-120, -120)
        self.assertEqual(command.linear_mm_s, -120)
        self.assertEqual(command.angular_mrad_s, 0)

    def test_feasible_independent_targets(self) -> None:
        command = wheel_speeds_to_chassis(80, 120)
        self.assertEqual(command.linear_mm_s, 100)
        self.assertEqual(command.angular_mrad_s, 244)
        represented = command.represented_wheel_speeds()
        self.assertAlmostEqual(represented.left_mm_s, 79.992, places=3)
        self.assertAlmostEqual(represented.right_mm_s, 120.008, places=3)

    def test_rejects_in_place_rotation(self) -> None:
        with self.assertRaises(UnsupportedWheelCommand):
            wheel_speeds_to_chassis(-100, 100)

    def test_rejects_turn_below_firmware_radius(self) -> None:
        with self.assertRaisesRegex(UnsupportedWheelCommand, "turn radius"):
            wheel_speeds_to_chassis(0, 100)

    def test_rejects_configured_speed_limit(self) -> None:
        with self.assertRaisesRegex(ValueError, "configured limit"):
            wheel_speeds_to_chassis(301, 301)


if __name__ == "__main__":
    unittest.main()
