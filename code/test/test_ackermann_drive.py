"""Hardware-free tests for calibrated steering and Ackermann linkage."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.ackermann_drive import AckermannDrive, plan_ackermann_motion  # noqa: E402
from components.rear_motor import MotorDirection  # noqa: E402
from components.steering_servo import (  # noqa: E402
    FrontSteeringServo,
    STEERING_CENTER_US,
    SteeringCalibration,
    YawDirection,
    steering_angle_to_pulse_us,
    yaw_to_steering_command,
)

# A clearly non-Cooper calibration: different direction sign, tighter
# mechanical range, linear curve and centre at 1500 us.  Used to prove the
# planning paths really use the injected calibration instead of the module
# default Cooper values.
CUSTOM_CALIBRATION = SteeringCalibration(
    direction_sign=1.0,
    logical_right_max_rad=-0.20,
    logical_left_max_rad=0.25,
    calibration_min_rad=-0.20,
    calibration_max_rad=0.25,
    pwm_min_us=700,
    pwm_max_us=2300,
    factory_center_us=1500,
    center_us=1500,
    curve_a3=0.0,
    curve_a2=0.0,
    curve_a1=-1.0,
    curve_a0=0.0,
    curve_scale=1000.0,
)


class CustomSteeringCalibrationTests(unittest.TestCase):
    def test_plan_uses_injected_calibration_not_module_default(self) -> None:
        plan = plan_ackermann_motion(
            100,
            0.10,
            steering_calibration=CUSTOM_CALIBRATION,
        )
        self.assertEqual(
            steering_angle_to_pulse_us(0.10, CUSTOM_CALIBRATION),
            plan.steering.pulse_us,
        )
        # 0.10 rad with the custom curve is far from the Cooper default pulse.
        self.assertNotEqual(
            steering_angle_to_pulse_us(0.10),
            plan.steering.pulse_us,
        )

    def test_plan_rejects_angle_beyond_custom_mechanical_range(self) -> None:
        # 0.30 rad exceeds the custom logical_left_max_rad of 0.25 and must be
        # rejected even though it is inside the Cooper range (+0.49).
        with self.assertRaises(ValueError):
            plan_ackermann_motion(
                100,
                0.30,
                steering_calibration=CUSTOM_CALIBRATION,
            )

    def test_drive_plan_uses_servo_calibration(self) -> None:
        steering = FrontSteeringServo(calibration=CUSTOM_CALIBRATION)
        drive = AckermannDrive(
            steering=steering,
            hardware_lock_path=None,
        )
        plan = drive._plan(100.0, 0.10, MotorDirection.FORWARD, True)
        self.assertEqual(
            steering_angle_to_pulse_us(0.10, CUSTOM_CALIBRATION),
            plan.steering.pulse_us,
        )

    def test_drive_set_yaw_uses_servo_calibration(self) -> None:
        steering = FrontSteeringServo(calibration=CUSTOM_CALIBRATION)
        drive = AckermannDrive(
            steering=steering,
            hardware_lock_path=None,
        )
        drive._speed_mm_s = 100.0
        import unittest.mock

        with unittest.mock.patch(
            "components.ackermann_drive.yaw_to_steering_command",
            wraps=yaw_to_steering_command,
        ) as yaw_command:
            with self.assertRaises(RuntimeError):  # drive is not started
                drive.set_yaw(YawDirection.LEFT, 0.10)
        # The servo calibration must be forwarded, not the module default.
        self.assertEqual(CUSTOM_CALIBRATION, yaw_command.call_args[0][2])


class SteeringCalibrationTests(unittest.TestCase):
    def test_known_calibration_points(self) -> None:
        self.assertEqual(steering_angle_to_pulse_us(0.0), 1580)
        self.assertEqual(steering_angle_to_pulse_us(-0.12), 1454)
        self.assertEqual(steering_angle_to_pulse_us(0.12), 1728)

    def test_explicit_yaw_direction(self) -> None:
        left = yaw_to_steering_command(YawDirection.LEFT, 0.12)
        right = yaw_to_steering_command(YawDirection.RIGHT, 0.12)
        self.assertEqual(left.angle_rad, 0.12)
        self.assertEqual(right.angle_rad, -0.12)
        self.assertGreater(left.pulse_us, STEERING_CENTER_US)
        self.assertLess(right.pulse_us, STEERING_CENTER_US)

    def test_asymmetric_mechanical_limits(self) -> None:
        with self.assertRaises(ValueError):
            steering_angle_to_pulse_us(0.491)
        with self.assertRaises(ValueError):
            steering_angle_to_pulse_us(-0.321)


class AckermannLinkageTests(unittest.TestCase):
    def test_owned_rear_driver_accepts_explicit_wheel_speed_limit(self) -> None:
        drive = AckermannDrive(max_wheel_speed_mm_s=600.0)
        self.assertEqual(drive.rear_motors.max_wheel_speed_mm_s, 600.0)

    def test_owned_rear_driver_propagates_explicit_pivot_capability(self) -> None:
        drive = AckermannDrive(allow_in_place_rotation=True)
        self.assertTrue(drive.rear_motors.allow_in_place_rotation)

    def test_straight_motion_keeps_rear_wheels_equal(self) -> None:
        plan = plan_ackermann_motion(100, 0.0)
        self.assertEqual(plan.rear.requested.left_mm_s, 100)
        self.assertEqual(plan.rear.requested.right_mm_s, 100)
        self.assertIsNone(plan.turn_radius_mm)

    def test_left_yaw_makes_left_rear_wheel_slower(self) -> None:
        plan = plan_ackermann_motion(100, 0.12)
        self.assertGreater(plan.turn_radius_mm, 350)
        self.assertGreater(plan.steering.pulse_us, STEERING_CENTER_US)
        self.assertLess(
            plan.rear.requested.left_mm_s,
            plan.rear.requested.right_mm_s,
        )
        self.assertGreater(plan.rear.angular_mrad_s, 0)

    def test_right_yaw_servo_and_rear_differential_have_same_sign(self) -> None:
        plan = plan_ackermann_motion(100, -0.12)
        self.assertLess(plan.steering.pulse_us, STEERING_CENTER_US)
        self.assertGreater(
            plan.rear.requested.left_mm_s,
            plan.rear.requested.right_mm_s,
        )
        self.assertLess(plan.rear.angular_mrad_s, 0)

    def test_reverse_preserves_ackermann_geometry(self) -> None:
        plan = plan_ackermann_motion(
            100,
            0.12,
            direction=MotorDirection.REVERSE,
        )
        self.assertGreater(
            plan.rear.requested.left_mm_s,
            plan.rear.requested.right_mm_s,
        )
        self.assertLess(plan.rear.angular_mrad_s, 0)

    def test_linkage_can_be_disabled(self) -> None:
        plan = plan_ackermann_motion(
            100,
            0.2,
            rear_differential_linked=False,
        )
        self.assertEqual(plan.rear.requested.left_mm_s, 100)
        self.assertEqual(plan.rear.requested.right_mm_s, 100)
        self.assertIsNone(plan.turn_radius_mm)

    def test_outer_wheel_speed_limit_is_checked_before_apply(self) -> None:
        with self.assertRaisesRegex(ValueError, "configured limit"):
            plan_ackermann_motion(300, 0.32)

    def test_physical_track_and_c10b_protocol_track_are_distinct(self) -> None:
        physical_track_mm = 117.1
        firmware_track_mm = 164.0
        plan = plan_ackermann_motion(
            100,
            0.20,
            track_width_mm=physical_track_mm,
            firmware_track_width_mm=firmware_track_mm,
        )
        self.assertIsNotNone(plan.turn_radius_mm)
        expected_delta = 100.0 / plan.turn_radius_mm * physical_track_mm
        self.assertAlmostEqual(
            plan.rear.requested.right_mm_s - plan.rear.requested.left_mm_s,
            expected_delta,
        )
        represented = plan.rear.represented_wheel_speeds(firmware_track_mm)
        self.assertAlmostEqual(
            represented.left_mm_s,
            plan.rear.requested.left_mm_s,
            delta=0.1,
        )
        self.assertAlmostEqual(
            represented.right_mm_s,
            plan.rear.requested.right_mm_s,
            delta=0.1,
        )

    def test_physical_radius_is_rejected_before_protocol_conversion(self) -> None:
        with self.assertRaisesRegex(ValueError, "physical turn radius"):
            plan_ackermann_motion(
                100,
                0.49,
                track_width_mm=117.1,
                firmware_track_width_mm=164.0,
                min_turn_radius_mm=350.0,
            )


if __name__ == "__main__":
    unittest.main()
