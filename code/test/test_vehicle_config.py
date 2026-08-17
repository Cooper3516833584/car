"""Vehicle geometry/configuration regression tests.

Verifies that building the vehicle from the TOML profile reproduces the
verified current-car parameters and that Navigation and Ackermann share one
geometry, while the physical track and the C10B firmware track stay distinct.
"""

from __future__ import annotations

from pathlib import Path
import unittest

from components.ackermann_drive import plan_ackermann_motion
from components.navigation import VehicleGeometry
from components.rear_motor import MotorDirection
from config.factory import (
    build_ackermann_drive,
    build_vehicle_geometry,
    derive_steering_clamp_rad,
)
from config.loader import load_car_config

REPO_ROOT = Path(__file__).resolve().parents[2]
REAL_CONFIG = REPO_ROOT / "configs" / "cooper_rock5a_l150.toml"


class VehicleConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_car_config(REAL_CONFIG)

    def test_geometry_from_config_matches_verified_car(self) -> None:
        geometry = build_vehicle_geometry(self.config)

        self.assertAlmostEqual(14.25, geometry.wheelbase_cm)
        self.assertAlmostEqual(11.71, geometry.track_width_cm)
        self.assertAlmostEqual(23.0, geometry.body_length_cm)
        self.assertAlmostEqual(14.5, geometry.body_width_cm)
        self.assertAlmostEqual(35.0, geometry.min_turn_radius_cm)
        self.assertAlmostEqual(7.125, geometry.rear_axle_to_body_center_cm)

    def test_default_geometry_equals_current_car_profile(self) -> None:
        default = VehicleGeometry()
        from_config = build_vehicle_geometry(self.config)
        self.assertEqual(default, from_config)

    def test_navigation_and_ackermann_share_same_geometry(self) -> None:
        geometry = build_vehicle_geometry(self.config)
        drive = build_ackermann_drive(self.config)

        self.assertAlmostEqual(
            geometry.wheelbase_cm * 10.0, drive.wheelbase_mm
        )
        self.assertAlmostEqual(
            geometry.track_width_cm * 10.0, drive.track_width_mm
        )
        # The C10B protocol track is deliberately different and stays on the
        # rear-motor driver, never merged into the physical Ackermann track.
        self.assertAlmostEqual(
            164.0, drive.rear_motors.track_width_mm
        )
        self.assertNotAlmostEqual(
            drive.rear_motors.track_width_mm, drive.track_width_mm
        )

    def test_drive_from_config_keeps_verified_values(self) -> None:
        drive = build_ackermann_drive(self.config)

        self.assertEqual("/dev/ttyACM0", drive.rear_motors.device)
        self.assertAlmostEqual(142.5, drive.wheelbase_mm)
        self.assertAlmostEqual(117.1, drive.track_width_mm)
        self.assertAlmostEqual(164.0, drive.firmware_track_width_mm)
        self.assertAlmostEqual(300.0, drive.rear_motors.max_wheel_speed_mm_s)
        self.assertAlmostEqual(350.0, drive.rear_motors.min_turn_radius_mm)
        self.assertFalse(drive.rear_motors.allow_in_place_rotation)

    def test_ackermann_plan_uses_physical_then_firmware_track(self) -> None:
        plan = plan_ackermann_motion(
            100.0,
            0.12,
            direction=MotorDirection.FORWARD,
            wheelbase_mm=142.5,
            track_width_mm=117.1,
            firmware_track_width_mm=164.0,
            min_turn_radius_mm=350.0,
        )
        self.assertIsNotNone(plan.turn_radius_mm)
        self.assertLess(
            plan.rear.requested.left_mm_s,
            plan.rear.requested.right_mm_s,
        )
        # The rear command is rebuilt with the C10B firmware track, so the
        # protocol quantization reproduces the requested wheels.
        represented = plan.rear.represented_wheel_speeds(164.0)
        self.assertAlmostEqual(
            represented.left_mm_s, plan.rear.requested.left_mm_s, delta=0.1
        )
        self.assertAlmostEqual(
            represented.right_mm_s, plan.rear.requested.right_mm_s, delta=0.1
        )

    def test_derived_steering_clamp_matches_verified_car(self) -> None:
        # The main program clamps post-fusion steering to the servo right
        # bound and the geometry-derived left bound; with the current profile
        # this reproduces the historical -0.32 / +0.336 limits.
        minimum, maximum = derive_steering_clamp_rad(self.config)
        self.assertAlmostEqual(-0.32, minimum)
        self.assertAlmostEqual(0.336, maximum, delta=0.001)

    def test_camera_control_wheelbase_comes_from_profile(self) -> None:
        from components.camera_line_follower import LineControlConfig

        control = LineControlConfig.from_wheelbase_mm(
            self.config.vehicle.geometry.wheelbase_mm
        )
        self.assertAlmostEqual(14.25, control.wheelbase_cm)

    def test_invalid_geometry_rejected(self) -> None:
        with self.assertRaises(ValueError):
            VehicleGeometry.from_config(
                wheelbase_mm=-1.0,
                track_width_mm=117.1,
                body_length_mm=230.0,
                body_width_mm=145.0,
                min_turn_radius_mm=350.0,
                rear_axle_to_body_center_mm=71.25,
            )


if __name__ == "__main__":
    unittest.main()
