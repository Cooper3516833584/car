"""Configuration loader tests: validation, precedence and CLI merging."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import unittest
import uuid

from config.loader import (
    CAR_CONFIG_ENV_VAR,
    ConfigError,
    load_car_config,
    resolve_config_path,
)
from components.radar_camera_line_following import (
    build_argument_parser,
    build_main_config,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
REAL_CONFIG = REPO_ROOT / "configs" / "cooper_rock5a_l150.toml"

MINIMAL_TOML = """
[profile]
name = "test"
description = "test profile"

[hardware.steering_pwm]
backend = "linux-sysfs"
channel = 0
period_ns = 20000000
polarity = "normal"
chip_device_match = "fd8b0000.pwm"

[hardware.alarm_gpio]
backend = "linux-sysfs-bank"
sysfs_root = "/sys/class/gpio"
bank_label = "gpio4"
line_offset = 11
active_low = true

[devices.motor]
port = "/dev/ttyACM0"

[devices.radar]
port = "/dev/ttyS6"

[devices.hc14]
port = "/dev/ttyUSB0"

[devices.screen]
enabled = true
port = "/dev/ttyUSB9"
baudrate = 9600

[devices.camera]
source = 0
width = 640
height = 360
fps = 30.0
fourcc = "MJPG"
backend = "v4l2"

[vehicle.geometry]
wheelbase_mm = 142.5
physical_track_width_mm = 117.1
body_length_mm = 230.0
body_width_mm = 145.0
wheel_thickness_mm = 26.4
outer_wheel_width_mm = 143.5
rear_axle_to_body_center_mm = 71.25

[vehicle.drive]
firmware_track_width_mm = 164.0
min_turn_radius_mm = 350.0
default_max_wheel_speed_mm_s = 300.0
allow_in_place_rotation = false

[vehicle.steering]
direction_sign = -1.0
logical_right_max_rad = -0.32
logical_left_max_rad = 0.49
calibration_min_rad = -0.49
calibration_max_rad = 0.32
pwm_min_us = 800
pwm_max_us = 2200
factory_center_us = 1501
center_us = 1580
curve_a3 = -0.628
curve_a2 = 1.269
curve_a1 = -1.772
curve_a0 = 1.573
curve_scale = 640.62

[sensors.radar.mount]
x_forward_cm = 0.0
y_left_cm = 0.0
yaw_cw_deg = 0.0

[missions.task1]
fleet_mission_request_state = 13
completion_alarm_seconds = 1.0
ab_speed_cm_s = 8.0
bc_speed_cm_s = 15.0
cd_speed_cm_s = 20.0
da_speed_cm_s = 15.0

[missions.task2]
fleet_mission_request_state = 14
completion_alarm_seconds = 1.0
ab_speed_cm_s = 25.0
bc_speed_cm_s = 9.0
cd_speed_before_retakeoff_cm_s = 4.0
cd_speed_after_retakeoff_cm_s = 30.0
da_speed_cm_s = 30.0
"""


class ConfigLoaderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = (
            Path(__file__).resolve().parent / f"_hal_tmp_{uuid.uuid4().hex}"
        )
        self.tmp.mkdir()
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def _write(self, name: str, content: str) -> Path:
        path = self.tmp / name
        path.write_text(content, encoding="utf-8")
        return path

    def test_current_profile_loads_with_verified_values(self) -> None:
        config = load_car_config(REAL_CONFIG)

        self.assertEqual("Cooper ROCK 5A + WHEELTEC L150", config.profile.name)
        self.assertEqual(142.5, config.vehicle.geometry.wheelbase_mm)
        self.assertEqual(117.1, config.vehicle.geometry.physical_track_width_mm)
        self.assertEqual(164.0, config.vehicle.drive.firmware_track_width_mm)
        self.assertEqual("/dev/ttyACM0", config.devices.motor.port)
        self.assertEqual("/dev/ttyS6", config.devices.radar.port)
        self.assertEqual(20.0, config.missions.common.radar_center_behind_a_cm)
        self.assertEqual(
            (8.0, 15.0, 20.0, 15.0),
            (
                config.missions.task1.ab_speed_cm_s,
                config.missions.task1.bc_speed_cm_s,
                config.missions.task1.cd_speed_cm_s,
                config.missions.task1.da_speed_cm_s,
            ),
        )
        self.assertEqual(
            (25.0, 9.0, 4.0, 30.0, 30.0),
            (
                config.missions.task2.ab_speed_cm_s,
                config.missions.task2.bc_speed_cm_s,
                config.missions.task2.cd_speed_before_retakeoff_cm_s,
                config.missions.task2.cd_speed_after_retakeoff_cm_s,
                config.missions.task2.da_speed_cm_s,
            ),
        )

    def test_physical_and_firmware_track_widths_are_independent(self) -> None:
        config = load_car_config(REAL_CONFIG)
        physical = config.vehicle.geometry.physical_track_width_mm
        firmware = config.vehicle.drive.firmware_track_width_mm
        self.assertEqual(117.1, physical)
        self.assertEqual(164.0, firmware)
        # Both survive independently and never collapse into one value.
        self.assertNotEqual(physical, firmware)
        self.assertIsInstance(config.vehicle.geometry.physical_track_width_mm, float)
        self.assertIsInstance(config.vehicle.drive.firmware_track_width_mm, float)

    def test_missing_config_file_raises(self) -> None:
        with self.assertRaisesRegex(ConfigError, "not found"):
            load_car_config(self.tmp / "missing.toml")

    def test_invalid_vehicle_value_raises_with_field_name(self) -> None:
        path = self._write(
            "bad_geometry.toml",
            MINIMAL_TOML.replace("wheelbase_mm = 142.5", "wheelbase_mm = -1"),
        )
        with self.assertRaisesRegex(ConfigError, "wheelbase_mm"):
            load_car_config(path)

    def test_missing_device_port_raises(self) -> None:
        path = self._write(
            "no_motor_port.toml",
            MINIMAL_TOML.replace(
                '[devices.motor]\nport = "/dev/ttyACM0"',
                "[devices.motor]",
            ),
        )
        with self.assertRaisesRegex(ConfigError, r"\[devices\.motor\] port"):
            load_car_config(path)

    def test_missing_vehicle_geometry_field_raises(self) -> None:
        path = self._write(
            "no_wheelbase.toml",
            MINIMAL_TOML.replace("wheelbase_mm = 142.5\n", ""),
        )
        with self.assertRaisesRegex(
            ConfigError, r"\[vehicle\.geometry\] wheelbase_mm"
        ):
            load_car_config(path)

    def test_missing_firmware_track_raises(self) -> None:
        path = self._write(
            "no_firmware_track.toml",
            MINIMAL_TOML.replace("firmware_track_width_mm = 164.0\n", ""),
        )
        with self.assertRaisesRegex(
            ConfigError, r"\[vehicle\.drive\] firmware_track_width_mm"
        ):
            load_car_config(path)

    def test_missing_servo_center_raises(self) -> None:
        path = self._write(
            "no_center_us.toml",
            MINIMAL_TOML.replace("center_us = 1580\n", ""),
        )
        with self.assertRaisesRegex(
            ConfigError, r"\[vehicle\.steering\] center_us"
        ):
            load_car_config(path)

    def test_missing_pwm_chip_match_raises(self) -> None:
        path = self._write(
            "no_chip_match.toml",
            MINIMAL_TOML.replace('chip_device_match = "fd8b0000.pwm"\n', ""),
        )
        with self.assertRaisesRegex(
            ConfigError, r"\[hardware\.steering_pwm\] chip_device_match"
        ):
            load_car_config(path)

    def test_missing_alarm_line_offset_raises(self) -> None:
        path = self._write(
            "no_line_offset.toml",
            MINIMAL_TOML.replace("line_offset = 11\n", ""),
        )
        with self.assertRaisesRegex(
            ConfigError, r"\[hardware\.alarm_gpio\] line_offset"
        ):
            load_car_config(path)

    def test_missing_radar_mount_raises(self) -> None:
        path = self._write(
            "no_radar_mount.toml",
            MINIMAL_TOML.replace(
                "x_forward_cm = 0.0\ny_left_cm = 0.0\nyaw_cw_deg = 0.0\n",
                "",
            ),
        )
        with self.assertRaisesRegex(
            ConfigError, r"\[sensors\.radar\.mount\] x_forward_cm"
        ):
            load_car_config(path)

    def test_missing_devices_section_raises(self) -> None:
        path = self._write(
            "no_devices.toml",
            MINIMAL_TOML.replace(
                "[devices.motor]\nport = \"/dev/ttyACM0\"\n",
                "",
            ),
        )
        with self.assertRaisesRegex(ConfigError, r"\[devices\.motor\]"):
            load_car_config(path)

    def test_invalid_steering_direction_raises(self) -> None:
        path = self._write(
            "bad_direction.toml",
            MINIMAL_TOML.replace("direction_sign = -1.0", "direction_sign = 2.0"),
        )
        with self.assertRaisesRegex(ConfigError, "direction_sign"):
            load_car_config(path)

    def test_bad_toml_syntax_raises(self) -> None:
        path = self._write("broken.toml", "this is not [valid toml")
        with self.assertRaises(ConfigError):
            load_car_config(path)

    def test_resolve_config_path_priority_cli_env_default(self) -> None:
        cli = self.tmp / "cli.toml"
        env_path = self.tmp / "env.toml"
        cli.write_text("", encoding="utf-8")
        env_path.write_text("", encoding="utf-8")

        old_env = os.environ.get(CAR_CONFIG_ENV_VAR)
        try:
            os.environ[CAR_CONFIG_ENV_VAR] = str(env_path)
            self.assertEqual(cli, resolve_config_path(cli))
            self.assertEqual(env_path, resolve_config_path(None))
        finally:
            if old_env is None:
                os.environ.pop(CAR_CONFIG_ENV_VAR, None)
            else:
                os.environ[CAR_CONFIG_ENV_VAR] = old_env

        self.assertEqual(REPO_ROOT / "configs" / "cooper_rock5a_l150.toml",
                         resolve_config_path(None))

    def test_cli_explicit_values_override_toml(self) -> None:
        config = load_car_config(REAL_CONFIG)
        args = build_argument_parser().parse_args(
            [
                "--radar-port",
                "/dev/ttyUSB9",
                "--ab-speed-cm-s",
                "18",
                "--startup-scans",
                "5",
            ]
        )
        main_config = build_main_config(
            config, mission="task1", cli_args=args, config_path=str(REAL_CONFIG)
        )

        self.assertEqual("/dev/ttyUSB9", main_config.radar_port)
        self.assertEqual(18.0, main_config.ab_speed_cm_s)
        self.assertEqual(5, main_config.startup_scan_count)
        # Unspecified values still come from the profile.
        self.assertEqual(15.0, main_config.bc_speed_cm_s)

    def test_runtime_radar_center_overrides_toml(self) -> None:
        config = load_car_config(REAL_CONFIG)
        state_file = self.tmp / "runtime" / "car_state.json"
        state_file.parent.mkdir()
        state_file.write_text(
            '{"radar_center_behind_a_cm": 36.5}', encoding="utf-8"
        )

        import config.models as models

        runtime = models.RuntimeStateConfig(
            enabled=True, state_file=str(state_file)
        )
        replaced = models.CarConfig(
            profile=config.profile,
            hardware=config.hardware,
            devices=config.devices,
            vehicle=config.vehicle,
            sensors=config.sensors,
            missions=config.missions,
            runtime=runtime,
            schema_version=config.schema_version,
        )
        args = build_argument_parser().parse_args([])
        main_config = build_main_config(
            replaced, mission="task1", cli_args=args, config_path=str(REAL_CONFIG)
        )
        self.assertEqual(36.5, main_config.radar_center_behind_a_cm)

    def test_cli_radar_center_beats_runtime_state(self) -> None:
        config = load_car_config(REAL_CONFIG)
        state_file = self.tmp / "runtime" / "car_state.json"
        state_file.parent.mkdir()
        state_file.write_text(
            '{"radar_center_behind_a_cm": 36.5}', encoding="utf-8"
        )

        import config.models as models

        runtime = models.RuntimeStateConfig(
            enabled=True, state_file=str(state_file)
        )
        replaced = models.CarConfig(
            profile=config.profile,
            hardware=config.hardware,
            devices=config.devices,
            vehicle=config.vehicle,
            sensors=config.sensors,
            missions=config.missions,
            runtime=runtime,
            schema_version=config.schema_version,
        )
        args = build_argument_parser().parse_args(
            ["--radar-center-behind-a-cm", "20"]
        )
        main_config = build_main_config(
            replaced, mission="task1", cli_args=args, config_path=str(REAL_CONFIG)
        )
        self.assertEqual(20.0, main_config.radar_center_behind_a_cm)

    def test_task2_speeds_from_profile(self) -> None:
        config = load_car_config(REAL_CONFIG)
        args = build_argument_parser().parse_args([])
        main_config = build_main_config(
            config, mission="task2", cli_args=args, config_path=str(REAL_CONFIG)
        )
        self.assertEqual(
            (25.0, 9.0, 4.0, 30.0, 30.0),
            (
                main_config.ab_speed_cm_s,
                main_config.bc_speed_cm_s,
                main_config.cd_speed_cm_s,
                main_config.cd_second_speed_cm_s,
                main_config.da_speed_cm_s,
            ),
        )


if __name__ == "__main__":
    unittest.main()
