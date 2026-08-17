"""Competition drive composition regression tests (no real hardware).

Verifies the exact wiring the formal entries use: TOML profile -> main config
-> AckermannDrive -> RearMotorDriver/FrontSteeringServo -> configured PWM HAL
and configured SteeringCalibration.  Guards against the P0 regression where
the configured PWM output exists but the formal program never injects it.
"""

from __future__ import annotations

from pathlib import Path
import unittest

from components.ackermann_drive import AckermannDrive
from components.steering_servo import FrontSteeringServo
from config.factory import (
    build_ackermann_drive,
    build_steering_calibration,
)
from config.loader import load_car_config
from components.radar_camera_line_following import (
    build_argument_parser,
    build_main_config,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
REAL_CONFIG = REPO_ROOT / "configs" / "cooper_rock5a_l150.toml"


class FakePWM:
    """In-memory PWMOutput used only to prove start() wiring."""

    def __init__(self) -> None:
        self.started = False
        self.pulse_us: int | None = None
        self.disabled = False

    def start(self) -> None:
        self.started = True

    def set_pulse_us(self, pulse_us: int) -> None:
        self.pulse_us = int(pulse_us)

    def disable(self) -> None:
        self.disabled = True

    def close(self) -> None:
        self.started = False


class CompetitionDriveCompositionTests(unittest.TestCase):
    def test_main_config_carries_configured_steering_servo(self) -> None:
        car_config = load_car_config(REAL_CONFIG)
        args = build_argument_parser().parse_args([])
        main_config = build_main_config(
            car_config,
            mission="task1",
            cli_args=args,
            config_path=str(REAL_CONFIG),
        )

        self.assertIsNotNone(main_config.steering_servo)
        assert main_config.steering_servo is not None
        # The servo must hold the TOML-built calibration, not the module
        # default (center 1580 us on the Cooper car).
        self.assertEqual(
            build_steering_calibration(car_config),
            main_config.steering_servo.calibration,
        )
        # And a real PWM HAL instance must be attached (not started yet).
        self.assertIsNotNone(main_config.steering_servo._pwm)

    def test_competition_drive_steering_has_configured_pwm(self) -> None:
        car_config = load_car_config(REAL_CONFIG)
        args = build_argument_parser().parse_args([])
        main_config = build_main_config(
            car_config,
            mission="task2",
            cli_args=args,
            config_path=str(REAL_CONFIG),
        )

        drive = AckermannDrive.from_config(
            device=main_config.motor_device or None,
            wheelbase_mm=main_config.wheelbase_mm,
            track_width_mm=main_config.physical_track_width_mm,
            firmware_track_width_mm=main_config.firmware_track_width_mm,
            max_wheel_speed_mm_s=300.0,
            min_turn_radius_mm=main_config.min_turn_radius_mm,
            allow_in_place_rotation=main_config.allow_in_place_rotation,
            steering_calibration=main_config.steering_calibration,
            hardware_lock_path=None,
            steering=main_config.steering_servo,
        )

        self.assertIsNotNone(drive.steering._pwm)

    def test_factory_drive_steering_has_configured_pwm(self) -> None:
        car_config = load_car_config(REAL_CONFIG)
        drive = build_ackermann_drive(car_config)
        self.assertIsNotNone(drive.steering._pwm)

    def test_steering_start_with_fake_pwm_does_not_fail_on_missing_pwm(
        self,
    ) -> None:
        # Simulate the formal path with an in-memory PWM so no real
        # /sys/class/pwm is touched: drive.steering.start() must not fail
        # with "front steering servo has no PWM output".
        car_config = load_car_config(REAL_CONFIG)
        pwm = FakePWM()
        steering = FrontSteeringServo(
            calibration=build_steering_calibration(car_config),
            pwm=pwm,
        )
        drive = AckermannDrive.from_config(
            device=None,
            wheelbase_mm=car_config.vehicle.geometry.wheelbase_mm,
            track_width_mm=(
                car_config.vehicle.geometry.physical_track_width_mm
            ),
            firmware_track_width_mm=(
                car_config.vehicle.drive.firmware_track_width_mm
            ),
            max_wheel_speed_mm_s=300.0,
            min_turn_radius_mm=(
                car_config.vehicle.drive.min_turn_radius_mm
            ),
            allow_in_place_rotation=False,
            steering_calibration=build_steering_calibration(car_config),
            hardware_lock_path=None,
            steering=steering,
        )

        drive.steering.start()

        self.assertTrue(pwm.started)
        # Centre command writes the calibration centre pulse.
        self.assertEqual(1580, pwm.pulse_us)
        drive.steering.close()


if __name__ == "__main__":
    unittest.main()
