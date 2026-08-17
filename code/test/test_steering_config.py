"""Steering calibration regression tests.

Verifies that the TOML profile reproduces the verified current-car servo
mapping and that the mechanical range guards keep working.  The regression
values are: -0.12 rad -> 1454 us, 0.00 rad -> 1580 us, +0.12 rad -> 1728 us.
"""

from __future__ import annotations

from pathlib import Path
import unittest

from components.steering_servo import (
    DEFAULT_STEERING_CALIBRATION,
    SteeringCalibration,
    SteeringCommand,
    YawDirection,
    make_steering_command,
    steering_angle_to_pulse_us,
    yaw_to_steering_command,
)
from config.factory import build_steering_calibration
from config.loader import load_car_config

REPO_ROOT = Path(__file__).resolve().parents[2]
REAL_CONFIG = REPO_ROOT / "configs" / "cooper_rock5a_l150.toml"


class SteeringConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_car_config(REAL_CONFIG)
        self.calibration = build_steering_calibration(self.config)

    def test_known_calibration_points_from_profile(self) -> None:
        self.assertEqual(1454, steering_angle_to_pulse_us(-0.12, self.calibration))
        self.assertEqual(1580, steering_angle_to_pulse_us(0.00, self.calibration))
        self.assertEqual(1728, steering_angle_to_pulse_us(0.12, self.calibration))

    def test_profile_calibration_matches_default_component_calibration(self) -> None:
        self.assertEqual(self.calibration, DEFAULT_STEERING_CALIBRATION)

    def test_mechanical_range_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            steering_angle_to_pulse_us(0.491, self.calibration)
        with self.assertRaises(ValueError):
            steering_angle_to_pulse_us(-0.321, self.calibration)

    def test_commands_carry_calibrated_pulse(self) -> None:
        command = make_steering_command(0.12, self.calibration)
        self.assertIsInstance(command, SteeringCommand)
        self.assertEqual(1728, command.pulse_us)
        self.assertEqual(YawDirection.LEFT, command.yaw_direction)

    def test_yaw_direction_magnitude_uses_calibration(self) -> None:
        left = yaw_to_steering_command(
            YawDirection.LEFT, 0.12, self.calibration
        )
        right = yaw_to_steering_command(
            YawDirection.RIGHT, 0.12, self.calibration
        )
        self.assertEqual(1728, left.pulse_us)
        self.assertEqual(1454, right.pulse_us)

    def test_invalid_calibration_rejected(self) -> None:
        with self.assertRaises(ValueError):
            SteeringCalibration(direction_sign=2.0)
        with self.assertRaises(ValueError):
            SteeringCalibration(center_us=100)
        with self.assertRaises(ValueError):
            SteeringCalibration(logical_right_max_rad=0.1)


if __name__ == "__main__":
    unittest.main()
