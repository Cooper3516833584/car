from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_ros_bridge.cmd_vel_conversion import UnsafeTwist, twist_to_ackermann  # noqa: E402


class CmdVelConversionTests(unittest.TestCase):
    def test_forward_left_turn(self) -> None:
        target = twist_to_ackermann(0.2, 0.2)
        self.assertGreater(target.steering_rad, 0.0)
        self.assertTrue(target.forward)

    def test_reverse_left_turn_reverses_steering(self) -> None:
        target = twist_to_ackermann(-0.1, 0.1)
        self.assertLess(target.steering_rad, 0.0)
        self.assertFalse(target.forward)

    def test_rejects_in_place_rotation(self) -> None:
        with self.assertRaises(UnsafeTwist):
            twist_to_ackermann(0.0, 0.2)


if __name__ == "__main__":
    unittest.main()
