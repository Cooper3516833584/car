"""Composition helpers that build validated components from one profile.

These builders are plain functions (never global singletons) so tests can
still instantiate every component directly.  The competition main program is
the only place that calls them.
"""

from __future__ import annotations

import math

from .models import CarConfig

from components.ackermann_drive import AckermannDrive
from components.camera_line_correction import CameraLineCorrectionConfig
from components.camera_line_follower import (
    LineControlConfig,
    LineVisionConfig,
    PerspectiveConfig,
)
from components.navigation import VehicleGeometry
from components.radar_driver import RadarMount
from components.rear_motor import RearMotorDriver
from components.steering_servo import (
    FrontSteeringServo,
    SteeringCalibration,
)


def build_steering_calibration(
    config: CarConfig,
) -> SteeringCalibration:
    """Convert the TOML steering section into the servo calibration object."""
    steering = config.vehicle.steering
    return SteeringCalibration(
        direction_sign=steering.direction_sign,
        logical_right_max_rad=steering.logical_right_max_rad,
        logical_left_max_rad=steering.logical_left_max_rad,
        calibration_min_rad=steering.calibration_min_rad,
        calibration_max_rad=steering.calibration_max_rad,
        pwm_min_us=steering.pwm_min_us,
        pwm_max_us=steering.pwm_max_us,
        factory_center_us=steering.factory_center_us,
        center_us=steering.center_us,
        curve_a3=steering.curve_a3,
        curve_a2=steering.curve_a2,
        curve_a1=steering.curve_a1,
        curve_a0=steering.curve_a0,
        curve_scale=steering.curve_scale,
    )


def build_vehicle_geometry(config: CarConfig) -> VehicleGeometry:
    """Build the single navigation geometry used by Navigation/planning.

    The servo mechanical limits come from the profile's ``[vehicle.steering]``
    section so planning/clamp calculations follow the real servo range.
    """
    geometry = config.vehicle.geometry
    steering = config.vehicle.steering
    return VehicleGeometry.from_config(
        wheelbase_mm=geometry.wheelbase_mm,
        track_width_mm=geometry.physical_track_width_mm,
        body_length_mm=geometry.body_length_mm,
        body_width_mm=geometry.body_width_mm,
        min_turn_radius_mm=config.vehicle.drive.min_turn_radius_mm,
        rear_axle_to_body_center_mm=geometry.rear_axle_to_body_center_mm,
        left_steering_limit_rad=steering.logical_left_max_rad,
        right_steering_limit_rad=steering.logical_right_max_rad,
    )


def build_rear_motor(config: CarConfig) -> RearMotorDriver:
    """Build the C10B rear-motor driver from the vehicle profile."""
    drive = config.vehicle.drive
    return RearMotorDriver(
        device=config.devices.motor.port,
        max_wheel_speed_mm_s=drive.default_max_wheel_speed_mm_s,
        track_width_mm=drive.firmware_track_width_mm,
        min_turn_radius_mm=drive.min_turn_radius_mm,
        allow_in_place_rotation=drive.allow_in_place_rotation,
    )


def build_steering_servo(config: CarConfig) -> FrontSteeringServo:
    """Build the steering servo with its profile calibration and PWM output."""
    from hal.pwm import LinuxSysfsPWMOutput

    pwm_config = config.hardware.steering_pwm
    pwm = LinuxSysfsPWMOutput(
        sysfs_root="/sys/class/pwm",
        chip_device_match=pwm_config.chip_device_match,
        channel=pwm_config.channel,
        period_ns=pwm_config.period_ns,
        polarity=pwm_config.polarity,
    )
    return FrontSteeringServo(
        calibration=build_steering_calibration(config),
        pwm=pwm,
    )


def build_ackermann_drive(
    config: CarConfig,
    *,
    max_wheel_speed_mm_s: float | None = None,
    hardware_lock_path: str | None = None,
) -> AckermannDrive:
    """Build the unified drive from the vehicle profile.

    ``max_wheel_speed_mm_s`` overrides the profile default when the mission
    needs a higher outer-wheel limit (the competition main does this).
    The steering servo is built with the profile's PWM HAL so the produced
    drive can actually start on hardware.
    """
    geometry = config.vehicle.geometry
    drive = config.vehicle.drive
    speed_limit = (
        drive.default_max_wheel_speed_mm_s
        if max_wheel_speed_mm_s is None
        else float(max_wheel_speed_mm_s)
    )
    return AckermannDrive.from_config(
        device=config.devices.motor.port,
        wheelbase_mm=geometry.wheelbase_mm,
        track_width_mm=geometry.physical_track_width_mm,
        firmware_track_width_mm=drive.firmware_track_width_mm,
        max_wheel_speed_mm_s=speed_limit,
        min_turn_radius_mm=drive.min_turn_radius_mm,
        allow_in_place_rotation=drive.allow_in_place_rotation,
        steering_calibration=build_steering_calibration(config),
        hardware_lock_path=hardware_lock_path,
        steering=build_steering_servo(config),
    )


def build_radar_mount(config: CarConfig) -> RadarMount:
    """Build the radar mount from the TOML [sensors.radar.mount] section."""
    mount = config.sensors.radar.mount
    return RadarMount(
        x_forward_cm=mount.x_forward_cm,
        y_left_cm=mount.y_left_cm,
        yaw_cw_deg=mount.yaw_cw_deg,
    )


def build_line_vision_config(config: CarConfig) -> LineVisionConfig:
    """Build the current front-camera vision calibration from the profile."""
    camera = config.devices.camera
    perspective = config.sensors.camera.perspective
    line = config.sensors.camera.line
    return LineVisionConfig(
        frame_width=camera.width,
        frame_height=camera.height,
        camera_fps=camera.fps,
        capture_backend_v4l2=(camera.backend == "v4l2"),
        fourcc=camera.fourcc,
        perspective=PerspectiveConfig(
            source_points_norm=perspective.source_points_norm,
            output_width_px=perspective.output_width_px,
            output_height_px=perspective.output_height_px,
            ground_width_cm=perspective.ground_width_cm,
            ground_depth_cm=perspective.ground_depth_cm,
        ),
        require_adaptive_confirmation=line.require_adaptive_confirmation,
        scan_near_cm=line.scan_near_cm,
        scan_far_cm=line.scan_far_cm,
        minimum_band_fill_ratio=line.minimum_band_fill_ratio,
        use_expected_width_window=line.use_expected_width_window,
        expected_line_width_cm=line.expected_line_width_cm,
        minimum_line_width_cm=line.minimum_line_width_cm,
        maximum_line_width_cm=line.maximum_line_width_cm,
        maximum_line_internal_gap_cm=line.maximum_line_internal_gap_cm,
        maximum_center_jump_cm=line.maximum_center_jump_cm,
        morphology_close_size=line.morphology_close_size,
        polynomial_smoothing_alpha=line.polynomial_smoothing_alpha,
        transverse_stop_max_height_cm=line.transverse_stop_max_height_cm,
        round_marker_min_height_cm=line.round_marker_min_height_cm,
        continuity_weight=line.continuity_weight,
    )


def build_camera_correction_config(
    config: CarConfig,
) -> CameraLineCorrectionConfig:
    """Build the soft camera-steering correction gates from the profile."""
    control = config.missions.control
    return CameraLineCorrectionConfig(
        lateral_deadband_cm=control.camera_lateral_deadband_cm,
        steering_gain_rad_per_cm=control.camera_steering_gain_rad_per_cm,
        maximum_abs_correction_rad=control.camera_max_steering_correction_rad,
    )


def build_line_control_config(config: CarConfig) -> LineControlConfig:
    """Build the camera line-follower control config.

    The wheelbase comes from the single vehicle profile so the camera
    controller never maintains a second wheelbase truth.
    """
    return LineControlConfig.from_wheelbase_mm(
        config.vehicle.geometry.wheelbase_mm
    )


def derive_steering_clamp_rad(
    config: CarConfig,
) -> tuple[float, float]:
    """Return the (min, max) steering clamp applied after camera fusion.

    By default the clamp is derived from the vehicle profile: the right limit
    is the servo mechanical right bound, the left limit is the geometry limit
    that satisfies the minimum turn radius.  Explicit ``missions.control``
    values override the derivation.
    """
    control = config.missions.control
    calibration = build_steering_calibration(config)
    geometry = build_vehicle_geometry(config)
    minimum = (
        calibration.logical_right_max_rad
        if control.steering_min_rad is None
        else float(control.steering_min_rad)
    )
    maximum = (
        geometry.max_left_steering_rad
        if control.steering_max_rad is None
        else float(control.steering_max_rad)
    )
    if not math.isfinite(minimum) or not math.isfinite(maximum):
        raise ValueError("derived steering clamp must be finite")
    if minimum > maximum:
        raise ValueError("steering clamp min must not exceed max")
    return minimum, maximum
