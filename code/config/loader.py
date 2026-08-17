"""TOML profile loading and path resolution for the car application.

Path priority:

1. CLI ``--config`` value
2. ``CAR_CONFIG`` environment variable
3. repository default ``configs/cooper_rock5a_l150.toml``

If the resolved file does not exist the loader raises ``ConfigError``.  The
program must never silently fall back to a hidden set of hardcoded vehicle
parameters.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Mapping

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10 and earlier.
    import tomli as tomllib  # type: ignore[no-redef]

from . import models


class ConfigError(RuntimeError):
    """The car configuration is missing, malformed or fails validation."""


CAR_CONFIG_ENV_VAR: str = "CAR_CONFIG"
DEFAULT_CONFIG_FILENAME: str = "cooper_rock5a_l150.toml"
CONFIGS_DIRECTORY_NAME: str = "configs"


def _default_config_path() -> Path:
    """Repository ``configs/cooper_rock5a_l150.toml``.

    ``loader.py`` lives in ``code/config``, so the repository root is two
    levels up.
    """
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / CONFIGS_DIRECTORY_NAME / DEFAULT_CONFIG_FILENAME


def resolve_config_path(cli_value: str | os.PathLike[str] | None = None) -> Path:
    """Resolve the profile path with CLI > env > repository default priority."""
    if cli_value is not None and os.fspath(cli_value).strip():
        return Path(cli_value).expanduser()
    environment = os.environ.get(CAR_CONFIG_ENV_VAR, "").strip()
    if environment:
        return Path(environment).expanduser()
    return _default_config_path()


def _as_mapping(value: Any, section: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"[{section}] must be a TOML table")
    return value


def _get_table(
    document: Mapping[str, Any],
    key: str,
    *,
    label: str | None = None,
    required: bool = True,
) -> Mapping[str, Any]:
    """Fetch a nested table.

    ``key`` is the local key inside ``document``; ``label`` is the dotted
    TOML path used in error messages (e.g. ``devices.motor``).
    """
    section = label or key
    if key not in document:
        if required:
            raise ConfigError(f"missing required section [{section}]")
        return {}
    return _as_mapping(document[key], section)


def _get_float(
    table: Mapping[str, Any],
    section: str,
    key: str,
    default: float,
) -> float:
    value = table.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"[{section}] {key} must be a number") from exc


def _get_int(
    table: Mapping[str, Any],
    section: str,
    key: str,
    default: int,
) -> int:
    value = table.get(key, default)
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"[{section}] {key} must be an integer") from exc


def _get_bool(
    table: Mapping[str, Any],
    section: str,
    key: str,
    default: bool,
) -> bool:
    value = table.get(key, default)
    if not isinstance(value, bool):
        raise ConfigError(f"[{section}] {key} must be a boolean")
    return value


def _get_str(
    table: Mapping[str, Any],
    section: str,
    key: str,
    default: str,
) -> str:
    value = table.get(key, default)
    if not isinstance(value, str):
        raise ConfigError(f"[{section}] {key} must be a string")
    return value


def _require_key(
    table: Mapping[str, Any],
    section: str,
    key: str,
) -> None:
    """Fail fast when a critical vehicle/hardware field is missing.

    The TOML profile is the single source of truth for the running car; a
    profile that omits a critical field must not silently fall back to the
    verified Cooper car's values.
    """
    if key not in table:
        raise ConfigError(f"missing required key [{section}] {key}")


def _get_float_tuple(
    table: Mapping[str, Any],
    section: str,
    key: str,
    default: tuple[float, ...],
) -> tuple[float, ...]:
    value = table.get(key, default)
    if not isinstance(value, (list, tuple)):
        raise ConfigError(f"[{section}] {key} must be an array")
    result: list[float] = []
    for index, item in enumerate(value):
        try:
            result.append(float(item))
        except (TypeError, ValueError) as exc:
            raise ConfigError(
                f"[{section}] {key}[{index}] must be a number"
            ) from exc
    return tuple(result)


def _get_norm_point_list(
    table: Mapping[str, Any],
    section: str,
    key: str,
    default: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    value = table.get(key, default)
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ConfigError(
            f"[{section}] {key} must contain exactly four [x, y] pairs"
        )
    points: list[tuple[float, float]] = []
    for index, item in enumerate(value):
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ConfigError(
                f"[{section}] {key}[{index}] must be an [x, y] pair"
            )
        try:
            points.append((float(item[0]), float(item[1])))
        except (TypeError, ValueError) as exc:
            raise ConfigError(
                f"[{section}] {key}[{index}] must contain numbers"
            ) from exc
    return tuple(points)


def _build_hardware(document: Mapping[str, Any]) -> models.HardwareConfig:
    hardware = _get_table(document, "hardware", required=True)
    pwm_table = _get_table(
        hardware, "steering_pwm", label="hardware.steering_pwm", required=True
    )
    gpio_table = _get_table(
        hardware, "alarm_gpio", label="hardware.alarm_gpio", required=True
    )

    # Critical PWM fields must be explicitly provided; physical_pin /
    # pin_function / device_tree_overlay stay optional documentation.
    for key in ("backend", "channel", "period_ns", "polarity", "chip_device_match"):
        _require_key(pwm_table, "hardware.steering_pwm", key)
    for key in ("backend", "bank_label", "line_offset", "active_low"):
        _require_key(gpio_table, "hardware.alarm_gpio", key)

    steering_pwm = models.SteeringPWMConfig(
        backend=_get_str(
            pwm_table, "hardware.steering_pwm", "backend", "linux-sysfs"
        ),
        channel=_get_int(pwm_table, "hardware.steering_pwm", "channel", 0),
        period_ns=_get_int(
            pwm_table, "hardware.steering_pwm", "period_ns", 20_000_000
        ),
        polarity=_get_str(
            pwm_table, "hardware.steering_pwm", "polarity", "normal"
        ),
        chip_device_match=_get_str(
            pwm_table,
            "hardware.steering_pwm",
            "chip_device_match",
            "fd8b0000.pwm",
        ),
        physical_pin=_get_int(
            pwm_table, "hardware.steering_pwm", "physical_pin", 23
        ),
        pin_function=_get_str(
            pwm_table, "hardware.steering_pwm", "pin_function", "PWM0_M2"
        ),
        device_tree_overlay=_get_str(
            pwm_table,
            "hardware.steering_pwm",
            "device_tree_overlay",
            "rk3588-pwm0-m2",
        ),
    )
    alarm_gpio = models.AlarmGPIOConfig(
        backend=_get_str(
            gpio_table, "hardware.alarm_gpio", "backend", "linux-sysfs-bank"
        ),
        sysfs_root=_get_str(
            gpio_table, "hardware.alarm_gpio", "sysfs_root", "/sys/class/gpio"
        ),
        bank_label=_get_str(gpio_table, "hardware.alarm_gpio", "bank_label", "gpio4"),
        line_offset=_get_int(
            gpio_table, "hardware.alarm_gpio", "line_offset", 11
        ),
        active_low=_get_bool(
            gpio_table, "hardware.alarm_gpio", "active_low", True
        ),
        physical_pin=_get_int(
            gpio_table, "hardware.alarm_gpio", "physical_pin", 11
        ),
        pin_function=_get_str(
            gpio_table, "hardware.alarm_gpio", "pin_function", "GPIO4_B3"
        ),
    )
    return models.HardwareConfig(
        steering_pwm=steering_pwm,
        alarm_gpio=alarm_gpio,
    )


def _build_devices(document: Mapping[str, Any]) -> models.DevicesConfig:
    devices = _get_table(document, "devices", required=True)
    motor = _get_table(devices, "motor", label="devices.motor", required=True)
    radar = _get_table(devices, "radar", label="devices.radar", required=True)
    hc14 = _get_table(devices, "hc14", label="devices.hc14", required=True)
    screen = _get_table(devices, "screen", label="devices.screen", required=False)
    camera = _get_table(devices, "camera", label="devices.camera", required=False)
    _require_key(motor, "devices.motor", "port")
    _require_key(radar, "devices.radar", "port")
    _require_key(hc14, "devices.hc14", "port")
    return models.DevicesConfig(
        motor=models.MotorDeviceConfig(
            port=_get_str(motor, "devices.motor", "port", ""),
        ),
        radar=models.RadarDeviceConfig(
            port=_get_str(radar, "devices.radar", "port", ""),
        ),
        hc14=models.HC14DeviceConfig(
            port=_get_str(hc14, "devices.hc14", "port", ""),
        ),
        screen=models.ScreenDeviceConfig(
            enabled=_get_bool(screen, "devices.screen", "enabled", True),
            port=_get_str(screen, "devices.screen", "port", ""),
            baudrate=_get_int(screen, "devices.screen", "baudrate", 9600),
        ),
        camera=models.CameraDeviceConfig(
            source=_get_camera_source(camera),
            width=_get_int(camera, "devices.camera", "width", 640),
            height=_get_int(camera, "devices.camera", "height", 360),
            fps=_get_float(camera, "devices.camera", "fps", 30.0),
            fourcc=_get_str(camera, "devices.camera", "fourcc", "MJPG"),
            backend=_get_str(camera, "devices.camera", "backend", "v4l2"),
        ),
    )


def _get_camera_source(table: Mapping[str, Any]) -> int | str:
    value = table.get("source", 0)
    if isinstance(value, bool):
        raise ConfigError("[devices.camera] source must be an int or string")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        if not value:
            raise ConfigError("[devices.camera] source must not be empty")
        try:
            return int(value)
        except ValueError:
            return value
    raise ConfigError("[devices.camera] source must be an int or string")


def _build_vehicle(document: Mapping[str, Any]) -> models.VehicleConfig:
    vehicle = _get_table(document, "vehicle", required=True)
    geometry = _get_table(
        vehicle, "geometry", label="vehicle.geometry", required=True
    )
    drive = _get_table(vehicle, "drive", label="vehicle.drive", required=True)
    steering = _get_table(
        vehicle, "steering", label="vehicle.steering", required=True
    )

    # Measured geometry must be explicit; wheel_thickness / outer_wheel_width
    # stay optional diagnostics and never override physical_track_width_mm.
    for key in (
        "wheelbase_mm",
        "physical_track_width_mm",
        "body_length_mm",
        "body_width_mm",
        "rear_axle_to_body_center_mm",
    ):
        _require_key(geometry, "vehicle.geometry", key)
    for key in (
        "firmware_track_width_mm",
        "min_turn_radius_mm",
        "default_max_wheel_speed_mm_s",
        "allow_in_place_rotation",
    ):
        _require_key(drive, "vehicle.drive", key)
    for key in (
        "direction_sign",
        "logical_right_max_rad",
        "logical_left_max_rad",
        "calibration_min_rad",
        "calibration_max_rad",
        "pwm_min_us",
        "pwm_max_us",
        "factory_center_us",
        "center_us",
        "curve_a3",
        "curve_a2",
        "curve_a1",
        "curve_a0",
        "curve_scale",
    ):
        _require_key(steering, "vehicle.steering", key)
    return models.VehicleConfig(
        geometry=models.VehicleGeometryConfig(
            wheelbase_mm=_get_float(
                geometry, "vehicle.geometry", "wheelbase_mm", 142.5
            ),
            physical_track_width_mm=_get_float(
                geometry,
                "vehicle.geometry",
                "physical_track_width_mm",
                117.1,
            ),
            body_length_mm=_get_float(
                geometry, "vehicle.geometry", "body_length_mm", 230.0
            ),
            body_width_mm=_get_float(
                geometry, "vehicle.geometry", "body_width_mm", 145.0
            ),
            wheel_thickness_mm=_get_float(
                geometry, "vehicle.geometry", "wheel_thickness_mm", 26.4
            ),
            outer_wheel_width_mm=_get_float(
                geometry, "vehicle.geometry", "outer_wheel_width_mm", 143.5
            ),
            rear_axle_to_body_center_mm=_get_float(
                geometry,
                "vehicle.geometry",
                "rear_axle_to_body_center_mm",
                71.25,
            ),
        ),
        drive=models.VehicleDriveConfig(
            firmware_track_width_mm=_get_float(
                drive, "vehicle.drive", "firmware_track_width_mm", 164.0
            ),
            min_turn_radius_mm=_get_float(
                drive, "vehicle.drive", "min_turn_radius_mm", 350.0
            ),
            default_max_wheel_speed_mm_s=_get_float(
                drive,
                "vehicle.drive",
                "default_max_wheel_speed_mm_s",
                300.0,
            ),
            allow_in_place_rotation=_get_bool(
                drive, "vehicle.drive", "allow_in_place_rotation", False
            ),
        ),
        steering=models.SteeringCalibrationConfig(
            direction_sign=_get_float(
                steering, "vehicle.steering", "direction_sign", -1.0
            ),
            logical_right_max_rad=_get_float(
                steering, "vehicle.steering", "logical_right_max_rad", -0.32
            ),
            logical_left_max_rad=_get_float(
                steering, "vehicle.steering", "logical_left_max_rad", 0.49
            ),
            calibration_min_rad=_get_float(
                steering, "vehicle.steering", "calibration_min_rad", -0.49
            ),
            calibration_max_rad=_get_float(
                steering, "vehicle.steering", "calibration_max_rad", 0.32
            ),
            pwm_min_us=_get_int(
                steering, "vehicle.steering", "pwm_min_us", 800
            ),
            pwm_max_us=_get_int(
                steering, "vehicle.steering", "pwm_max_us", 2200
            ),
            factory_center_us=_get_int(
                steering, "vehicle.steering", "factory_center_us", 1501
            ),
            center_us=_get_int(
                steering, "vehicle.steering", "center_us", 1580
            ),
            curve_a3=_get_float(
                steering, "vehicle.steering", "curve_a3", -0.628
            ),
            curve_a2=_get_float(
                steering, "vehicle.steering", "curve_a2", 1.269
            ),
            curve_a1=_get_float(
                steering, "vehicle.steering", "curve_a1", -1.772
            ),
            curve_a0=_get_float(
                steering, "vehicle.steering", "curve_a0", 1.573
            ),
            curve_scale=_get_float(
                steering, "vehicle.steering", "curve_scale", 640.62
            ),
        ),
    )


def _build_sensors(document: Mapping[str, Any]) -> models.SensorsConfig:
    sensors = _get_table(document, "sensors", required=True)
    radar = _get_table(sensors, "radar", label="sensors.radar", required=True)
    mount = _get_table(
        radar, "mount", label="sensors.radar.mount", required=True
    )
    for key in ("x_forward_cm", "y_left_cm", "yaw_cw_deg"):
        _require_key(mount, "sensors.radar.mount", key)
    camera = _get_table(sensors, "camera", label="sensors.camera", required=False)
    perspective = _get_table(
        camera, "perspective", label="sensors.camera.perspective", required=False
    )
    line = _get_table(camera, "line", label="sensors.camera.line", required=False)
    return models.SensorsConfig(
        radar=models.SensorRadarConfig(
            mount=models.RadarMountConfig(
                x_forward_cm=_get_float(
                    mount, "sensors.radar.mount", "x_forward_cm", 0.0
                ),
                y_left_cm=_get_float(
                    mount, "sensors.radar.mount", "y_left_cm", 0.0
                ),
                yaw_cw_deg=_get_float(
                    mount, "sensors.radar.mount", "yaw_cw_deg", 0.0
                ),
            )
        ),
        camera=models.SensorCameraConfig(
            perspective=models.CameraPerspectiveConfig(
                source_points_norm=_get_norm_point_list(
                    perspective,
                    "sensors.camera.perspective",
                    "source_points_norm",
                    (
                        (0.02, 0.66),
                        (0.93, 0.66),
                        (0.68, 0.02),
                        (0.23, 0.02),
                    ),
                ),
                output_width_px=_get_int(
                    perspective,
                    "sensors.camera.perspective",
                    "output_width_px",
                    320,
                ),
                output_height_px=_get_int(
                    perspective,
                    "sensors.camera.perspective",
                    "output_height_px",
                    400,
                ),
                ground_width_cm=_get_float(
                    perspective,
                    "sensors.camera.perspective",
                    "ground_width_cm",
                    80.0,
                ),
                ground_depth_cm=_get_float(
                    perspective,
                    "sensors.camera.perspective",
                    "ground_depth_cm",
                    100.0,
                ),
            ),
            line=models.CameraLineConfig(
                require_adaptive_confirmation=_get_bool(
                    line,
                    "sensors.camera.line",
                    "require_adaptive_confirmation",
                    False,
                ),
                scan_near_cm=_get_float(
                    line, "sensors.camera.line", "scan_near_cm", 12.0
                ),
                scan_far_cm=_get_float(
                    line, "sensors.camera.line", "scan_far_cm", 72.0
                ),
                minimum_band_fill_ratio=_get_float(
                    line,
                    "sensors.camera.line",
                    "minimum_band_fill_ratio",
                    0.20,
                ),
                use_expected_width_window=_get_bool(
                    line,
                    "sensors.camera.line",
                    "use_expected_width_window",
                    True,
                ),
                expected_line_width_cm=_get_float(
                    line,
                    "sensors.camera.line",
                    "expected_line_width_cm",
                    28.0,
                ),
                minimum_line_width_cm=_get_float(
                    line,
                    "sensors.camera.line",
                    "minimum_line_width_cm",
                    10.0,
                ),
                maximum_line_width_cm=_get_float(
                    line,
                    "sensors.camera.line",
                    "maximum_line_width_cm",
                    40.0,
                ),
                maximum_line_internal_gap_cm=_get_float(
                    line,
                    "sensors.camera.line",
                    "maximum_line_internal_gap_cm",
                    8.0,
                ),
                maximum_center_jump_cm=_get_float(
                    line,
                    "sensors.camera.line",
                    "maximum_center_jump_cm",
                    18.0,
                ),
                morphology_close_size=_get_int(
                    line,
                    "sensors.camera.line",
                    "morphology_close_size",
                    9,
                ),
                polynomial_smoothing_alpha=_get_float(
                    line,
                    "sensors.camera.line",
                    "polynomial_smoothing_alpha",
                    0.32,
                ),
                transverse_stop_max_height_cm=_get_float(
                    line,
                    "sensors.camera.line",
                    "transverse_stop_max_height_cm",
                    8.0,
                ),
                round_marker_min_height_cm=_get_float(
                    line,
                    "sensors.camera.line",
                    "round_marker_min_height_cm",
                    12.0,
                ),
                continuity_weight=_get_float(
                    line, "sensors.camera.line", "continuity_weight", 0.12
                ),
            ),
        ),
    )


def _build_missions(document: Mapping[str, Any]) -> models.MissionsConfig:
    missions = _get_table(document, "missions", required=False)
    common = _get_table(missions, "common", label="missions.common", required=False)
    task1 = _get_table(missions, "task1", label="missions.task1", required=False)
    task2 = _get_table(missions, "task2", label="missions.task2", required=False)
    control = _get_table(
        missions, "control", label="missions.control", required=False
    )
    control_camera = _get_table(
        control, "camera", label="missions.control.camera", required=False
    )

    def _control_float(name: str, default: float) -> float:
        return _get_float(control, "missions.control", name, default)

    def _control_camera_float(name: str, default: float) -> float:
        return _get_float(
            control_camera, "missions.control.camera", name, default
        )

    def _control_int(name: str, default: int) -> int:
        return _get_int(control, "missions.control", name, default)

    return models.MissionsConfig(
        common=models.MissionCommonConfig(
            radar_center_behind_a_cm=_get_float(
                common, "missions.common", "radar_center_behind_a_cm", 20.0
            ),
            startup_scan_count=_get_int(
                common, "missions.common", "startup_scan_count", 3
            ),
            calibration_timeout_s=_get_float(
                common, "missions.common", "calibration_timeout_s", 30.0
            ),
            camera_correction_enabled=_get_bool(
                common, "missions.common", "camera_correction_enabled", True
            ),
            fleet_position_reporting_enabled=_get_bool(
                common,
                "missions.common",
                "fleet_position_reporting_enabled",
                True,
            ),
        ),
        task1=models.Task1Config(
            fleet_mission_request_state=_get_int(
                task1, "missions.task1", "fleet_mission_request_state", 13
            ),
            completion_alarm_seconds=_get_float(
                task1, "missions.task1", "completion_alarm_seconds", 1.0
            ),
            ab_speed_cm_s=_get_float(
                task1, "missions.task1", "ab_speed_cm_s", 8.0
            ),
            bc_speed_cm_s=_get_float(
                task1, "missions.task1", "bc_speed_cm_s", 15.0
            ),
            cd_speed_cm_s=_get_float(
                task1, "missions.task1", "cd_speed_cm_s", 20.0
            ),
            da_speed_cm_s=_get_float(
                task1, "missions.task1", "da_speed_cm_s", 15.0
            ),
        ),
        task2=models.Task2Config(
            fleet_mission_request_state=_get_int(
                task2, "missions.task2", "fleet_mission_request_state", 14
            ),
            completion_alarm_seconds=_get_float(
                task2, "missions.task2", "completion_alarm_seconds", 1.0
            ),
            ab_speed_cm_s=_get_float(
                task2, "missions.task2", "ab_speed_cm_s", 25.0
            ),
            bc_speed_cm_s=_get_float(
                task2, "missions.task2", "bc_speed_cm_s", 9.0
            ),
            cd_speed_before_retakeoff_cm_s=_get_float(
                task2,
                "missions.task2",
                "cd_speed_before_retakeoff_cm_s",
                4.0,
            ),
            cd_speed_after_retakeoff_cm_s=_get_float(
                task2,
                "missions.task2",
                "cd_speed_after_retakeoff_cm_s",
                30.0,
            ),
            da_speed_cm_s=_get_float(
                task2, "missions.task2", "da_speed_cm_s", 30.0
            ),
        ),
        control=models.MissionControlConfig(
            camera_lateral_deadband_cm=_control_camera_float(
                "lateral_deadband_cm", 10.0
            ),
            camera_steering_gain_rad_per_cm=_control_camera_float(
                "steering_gain_rad_per_cm", 0.010
            ),
            camera_max_steering_correction_rad=_control_camera_float(
                "max_steering_correction_rad", 0.140
            ),
            ab_start_alignment_full_end_progress_cm=_control_float(
                "ab_start_alignment_full_end_progress_cm", 80.0
            ),
            ab_start_alignment_fade_end_progress_cm=_control_float(
                "ab_start_alignment_fade_end_progress_cm", 100.0
            ),
            ab_start_heading_gain=_control_float("ab_start_heading_gain", 1.30),
            ab_start_max_heading_correction_rad=_control_float(
                "ab_start_max_heading_correction_rad", 0.180
            ),
            ab_start_max_total_camera_correction_rad=_control_float(
                "ab_start_max_total_camera_correction_rad", 0.220
            ),
            ab_start_min_valid_frames=_control_int(
                "ab_start_min_valid_frames", 2
            ),
            ab_start_max_curvature_per_cm=_control_float(
                "ab_start_max_curvature_per_cm", 0.003
            ),
            ab_start_max_forward_heading_change_rad=_control_float(
                "ab_start_max_forward_heading_change_rad", 0.080
            ),
            ab_line_assist_full_end_progress_cm=_control_float(
                "ab_line_assist_full_end_progress_cm", 100.0
            ),
            ab_line_assist_fade_end_progress_cm=_control_float(
                "ab_line_assist_fade_end_progress_cm", 135.0
            ),
            ab_line_assist_lateral_deadband_cm=_control_float(
                "ab_line_assist_lateral_deadband_cm", 2.0
            ),
            ab_line_assist_gain_rad_per_cm=_control_float(
                "ab_line_assist_gain_rad_per_cm", 0.005
            ),
            ab_line_assist_max_correction_rad=_control_float(
                "ab_line_assist_max_correction_rad", 0.060
            ),
            ab_line_assist_min_valid_frames=_control_int(
                "ab_line_assist_min_valid_frames", 3
            ),
            ab_line_assist_max_curvature_per_cm=_control_float(
                "ab_line_assist_max_curvature_per_cm", 0.004
            ),
            ab_line_assist_max_forward_heading_change_rad=_control_float(
                "ab_line_assist_max_forward_heading_change_rad", 0.100
            ),
            ab_lateral_alignment_learning_start_progress_cm=_control_float(
                "ab_lateral_alignment_learning_start_progress_cm", 50.0
            ),
            ab_lateral_alignment_learning_end_progress_cm=_control_float(
                "ab_lateral_alignment_learning_end_progress_cm", 90.0
            ),
            ab_lateral_alignment_ramp_start_progress_cm=_control_float(
                "ab_lateral_alignment_ramp_start_progress_cm", 60.0
            ),
            ab_lateral_alignment_full_progress_cm=_control_float(
                "ab_lateral_alignment_full_progress_cm", 100.0
            ),
            ab_lateral_alignment_terminal_fade_distance_cm=_control_float(
                "ab_lateral_alignment_terminal_fade_distance_cm", 65.0
            ),
            ab_lateral_alignment_min_valid_frames=_control_int(
                "ab_lateral_alignment_min_valid_frames", 4
            ),
            ab_lateral_alignment_required_measurements=_control_int(
                "ab_lateral_alignment_required_measurements", 5
            ),
            ab_lateral_alignment_max_abs_offset_cm=_control_float(
                "ab_lateral_alignment_max_abs_offset_cm", 12.0
            ),
            ab_lateral_alignment_max_mad_cm=_control_float(
                "ab_lateral_alignment_max_mad_cm", 1.5
            ),
            ab_lateral_alignment_max_curvature_per_cm=_control_float(
                "ab_lateral_alignment_max_curvature_per_cm", 0.003
            ),
            ab_lateral_alignment_max_forward_heading_change_rad=_control_float(
                "ab_lateral_alignment_max_forward_heading_change_rad", 0.080
            ),
            bc_entry_limit_end_progress_cm=_control_float(
                "bc_entry_limit_end_progress_cm", 210.0
            ),
            bc_entry_min_right_correction_rad=_control_float(
                "bc_entry_min_right_correction_rad", -0.012
            ),
            final_da_visual_window_cm=_control_float(
                "final_da_visual_window_cm", 65.0
            ),
            final_da_visual_hold_s=_control_float(
                "final_da_visual_hold_s", 0.65
            ),
            final_a_max_camera_error_cm=_control_float(
                "final_a_max_camera_error_cm", 6.0
            ),
            fleet_terminal_report_grace_s=_control_float(
                "fleet_terminal_report_grace_s", 3.0
            ),
            fleet_trace_drain_timeout_s=_control_float(
                "fleet_trace_drain_timeout_s", 6.0
            ),
            steering_min_rad=(
                _get_float(control, "missions.control", "steering_min_rad", 0.0)
                if "steering_min_rad" in control
                else None
            ),
            steering_max_rad=(
                _get_float(control, "missions.control", "steering_max_rad", 0.0)
                if "steering_max_rad" in control
                else None
            ),
        ),
    )


def _build_runtime(document: Mapping[str, Any]) -> models.RuntimeStateConfig:
    runtime = _get_table(document, "runtime", required=False)
    return models.RuntimeStateConfig(
        enabled=_get_bool(runtime, "runtime", "enabled", True),
        state_file=_get_str(runtime, "runtime", "state_file", "runtime/car_state.json"),
        allowed_radar_center_behind_a_cm=_get_float_tuple(
            runtime,
            "runtime",
            "allowed_radar_center_behind_a_cm",
            (20.0, 36.5),
        ),
    )


def load_car_config(
    path: str | os.PathLike[str] | None = None,
) -> models.CarConfig:
    """Load and validate one TOML profile; raises ``ConfigError`` on failure."""
    resolved = resolve_config_path(path)
    if not resolved.is_file():
        raise ConfigError(
            f"car configuration file not found: {resolved}\n"
            "pass --config <path> or set CAR_CONFIG to select a profile"
        )
    try:
        raw = resolved.read_bytes()
    except OSError as exc:
        raise ConfigError(f"cannot read configuration file {resolved}: {exc}") from exc
    try:
        document = tomllib.loads(raw.decode("utf-8"))
    except (tomllib.TOMLDecodeError, UnicodeDecodeError) as exc:
        raise ConfigError(f"invalid TOML in {resolved}: {exc}") from exc
    if not isinstance(document, Mapping):
        raise ConfigError(f"configuration file {resolved} must be a TOML table")

    profile_table = _get_table(document, "profile", required=False)
    schema_table = _get_table(document, "schema", required=False)
    try:
        profile = models.ProfileConfig(
            name=_get_str(profile_table, "profile", "name", "unnamed profile"),
            description=_get_str(
                profile_table, "profile", "description", ""
            ),
        )
        car_config = models.CarConfig(
            profile=profile,
            hardware=_build_hardware(document),
            devices=_build_devices(document),
            vehicle=_build_vehicle(document),
            sensors=_build_sensors(document),
            missions=_build_missions(document),
            runtime=_build_runtime(document),
            schema_version=_get_int(
                schema_table, "schema", "version", models.DEFAULT_SCHEMA_VERSION
            ),
        )
    except (ValueError, TypeError) as exc:
        raise ConfigError(f"invalid configuration in {resolved}: {exc}") from exc
    return car_config
