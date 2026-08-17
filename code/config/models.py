"""Strongly-typed configuration models for the competition car.

Every field below is either a protocol/hardware description (kept in code per
the project rules) or vehicle/sensor/mission data that must live in the TOML
profile.  All validation is fail-fast: a bad TOML value raises ``ValueError``
with the offending field name instead of silently driving the car with a
wrong value.

The dataclass defaults mirror the *verified* Cooper ROCK 5A + WHEELTEC L150
profile so that pure unit tests and API compatibility keep working without a
TOML file on disk.  The real program always loads ``CarConfig`` from a TOML
file and never relies on these defaults to drive the car.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Final, Sequence


def _require_finite(name: str, value: float) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _require_positive(name: str, value: float) -> float:
    number = _require_finite(name, value)
    if number <= 0.0:
        raise ValueError(f"{name} must be positive")
    return number


def _require_non_negative(name: str, value: float) -> float:
    number = _require_finite(name, value)
    if number < 0.0:
        raise ValueError(f"{name} cannot be negative")
    return number


def _require_int_in_range(name: str, value: int, minimum: int, maximum: int) -> int:
    integer = int(value)
    if not minimum <= integer <= maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
    return integer


@dataclass(frozen=True, slots=True)
class ProfileConfig:
    """Identity of one verified vehicle profile."""

    name: str = "Cooper ROCK 5A + WHEELTEC L150"
    description: str = "Current verified competition car"

    def __post_init__(self) -> None:
        if not self.name or not self.description:
            raise ValueError("profile name and description must not be empty")


# ============================================================
# 主控板 / IO
# ============================================================


@dataclass(frozen=True, slots=True)
class SteeringPWMConfig:
    """Linux sysfs PWM output used by the front steering servo.

    ``chip_device_match`` selects the pwmchip by resolving to a device path
    containing this substring (e.g. ``fd8b0000.pwm`` on the ROCK 5A).
    ``physical_pin`` / ``pin_function`` / ``device_tree_overlay`` are
    documentation/diagnostics only and never participate in steering math.
    """

    backend: str = "linux-sysfs"
    channel: int = 0
    period_ns: int = 20_000_000
    polarity: str = "normal"
    chip_device_match: str = "fd8b0000.pwm"
    physical_pin: int = 23
    pin_function: str = "PWM0_M2"
    device_tree_overlay: str = "rk3588-pwm0-m2"

    def __post_init__(self) -> None:
        if self.backend != "linux-sysfs":
            raise ValueError(
                f"unsupported steering PWM backend {self.backend!r}; "
                "only 'linux-sysfs' is implemented"
            )
        if self.channel < 0:
            raise ValueError("steering PWM channel cannot be negative")
        if self.period_ns <= 0:
            raise ValueError("steering PWM period_ns must be positive")
        if self.polarity not in ("normal", "inversed"):
            raise ValueError("steering PWM polarity must be 'normal' or 'inversed'")
        if not self.chip_device_match:
            raise ValueError("steering PWM chip_device_match must not be empty")


@dataclass(frozen=True, slots=True)
class AlarmGPIOConfig:
    """Linux sysfs bank/line output used by the sound/light alarm.

    The line is selected by resolving the chip whose ``label`` equals
    ``bank_label`` and adding ``line_offset`` to its ``base``.  ``active_low``
    means a raw 0 drives the alarm on (the verified ROCK 5A wiring).
    """

    backend: str = "linux-sysfs-bank"
    sysfs_root: str = "/sys/class/gpio"
    bank_label: str = "gpio4"
    line_offset: int = 11
    active_low: bool = True
    physical_pin: int = 11
    pin_function: str = "GPIO4_B3"

    def __post_init__(self) -> None:
        if self.backend != "linux-sysfs-bank":
            raise ValueError(
                f"unsupported alarm GPIO backend {self.backend!r}; "
                "only 'linux-sysfs-bank' is implemented"
            )
        if not self.sysfs_root:
            raise ValueError("alarm GPIO sysfs_root must not be empty")
        if not self.bank_label:
            raise ValueError("alarm GPIO bank_label must not be empty")
        if self.line_offset < 0:
            raise ValueError("alarm GPIO line_offset cannot be negative")


@dataclass(frozen=True, slots=True)
class HardwareConfig:
    """Board-level IO (PWM for steering, GPIO for the alarm)."""

    steering_pwm: SteeringPWMConfig = field(default_factory=SteeringPWMConfig)
    alarm_gpio: AlarmGPIOConfig = field(default_factory=AlarmGPIOConfig)


# ============================================================
# Linux 设备
# ============================================================


@dataclass(frozen=True, slots=True)
class MotorDeviceConfig:
    """C10B driver-board serial device (currently USB CDC-ACM)."""

    port: str = ""

    def __post_init__(self) -> None:
        if not self.port:
            raise ValueError("devices.motor.port must not be empty")


@dataclass(frozen=True, slots=True)
class RadarDeviceConfig:
    """D500 radar UART device."""

    port: str = ""

    def __post_init__(self) -> None:
        if not self.port:
            raise ValueError("devices.radar.port must not be empty")


@dataclass(frozen=True, slots=True)
class HC14DeviceConfig:
    """HC-14 wireless serial device (stable by-id path preferred)."""

    port: str = ""

    def __post_init__(self) -> None:
        if not self.port:
            raise ValueError("devices.hc14.port must not be empty")


@dataclass(frozen=True, slots=True)
class ScreenDeviceConfig:
    """Serial touch screen used by the mission launcher."""

    enabled: bool = True
    port: str = ""
    baudrate: int = 9600

    def __post_init__(self) -> None:
        if self.enabled and not self.port:
            raise ValueError("devices.screen.port must not be empty when enabled")
        if self.baudrate <= 0:
            raise ValueError("devices.screen.baudrate must be positive")


@dataclass(frozen=True, slots=True)
class CameraDeviceConfig:
    """Camera capture device parameters."""

    source: int | str = 0
    width: int = 640
    height: int = 360
    fps: float = 30.0
    fourcc: str = "MJPG"
    backend: str = "v4l2"

    def __post_init__(self) -> None:
        if isinstance(self.source, int):
            if self.source < 0:
                raise ValueError("devices.camera.source cannot be negative")
        elif not self.source:
            raise ValueError("devices.camera.source must not be empty")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("devices.camera width and height must be positive")
        if _require_positive("devices.camera.fps", self.fps) <= 0.0:
            raise ValueError("devices.camera.fps must be positive")
        if len(self.fourcc) != 4:
            raise ValueError("devices.camera.fourcc must contain four characters")


@dataclass(frozen=True, slots=True)
class DevicesConfig:
    """All serial/camera devices owned by the current board."""

    motor: MotorDeviceConfig = field(default_factory=MotorDeviceConfig)
    radar: RadarDeviceConfig = field(default_factory=RadarDeviceConfig)
    hc14: HC14DeviceConfig = field(default_factory=HC14DeviceConfig)
    screen: ScreenDeviceConfig = field(default_factory=ScreenDeviceConfig)
    camera: CameraDeviceConfig = field(default_factory=CameraDeviceConfig)


# ============================================================
# 实车几何 / 驱动 / 舵机
# ============================================================


@dataclass(frozen=True, slots=True)
class VehicleGeometryConfig:
    """Measured physical vehicle dimensions (millimetres).

    ``physical_track_width_mm`` is the real rear-wheel centre distance used by
    Ackermann geometry / Navigation / path planning.  It is deliberately
    different from ``firmware_track_width_mm`` in :class:`VehicleDriveConfig`.
    """

    wheelbase_mm: float = 142.5
    physical_track_width_mm: float = 117.1
    body_length_mm: float = 230.0
    body_width_mm: float = 145.0
    wheel_thickness_mm: float = 26.4
    outer_wheel_width_mm: float = 143.5
    rear_axle_to_body_center_mm: float = 71.25

    def __post_init__(self) -> None:
        for name in (
            "wheelbase_mm",
            "physical_track_width_mm",
            "body_length_mm",
            "body_width_mm",
            "wheel_thickness_mm",
            "outer_wheel_width_mm",
        ):
            _require_positive(f"vehicle.geometry.{name}", getattr(self, name))
        _require_non_negative(
            "vehicle.geometry.rear_axle_to_body_center_mm",
            self.rear_axle_to_body_center_mm,
        )
        if self.rear_axle_to_body_center_mm > self.body_length_mm / 2.0:
            raise ValueError(
                "vehicle.geometry.rear_axle_to_body_center_mm lies outside "
                "the vehicle body"
            )


@dataclass(frozen=True, slots=True)
class VehicleDriveConfig:
    """C10B drive-board and motion-limit parameters.

    ``firmware_track_width_mm`` is compiled into the C10B firmware and used
    for ``Vz=(right-left)/firmware_track``.  It must never be replaced by the
    physical track width: doing so would scale the rear differential by
    ``164/117.1 ~= 1.40`` and break Ackermann matching.
    """

    firmware_track_width_mm: float = 164.0
    min_turn_radius_mm: float = 350.0
    default_max_wheel_speed_mm_s: float = 300.0
    allow_in_place_rotation: bool = False

    def __post_init__(self) -> None:
        _require_positive(
            "vehicle.drive.firmware_track_width_mm",
            self.firmware_track_width_mm,
        )
        _require_positive(
            "vehicle.drive.min_turn_radius_mm",
            self.min_turn_radius_mm,
        )
        _require_positive(
            "vehicle.drive.default_max_wheel_speed_mm_s",
            self.default_max_wheel_speed_mm_s,
        )
        if self.default_max_wheel_speed_mm_s > 1200.0:
            raise ValueError(
                "vehicle.drive.default_max_wheel_speed_mm_s exceeds the "
                "C10B firmware linear limit of 1200 mm/s"
            )


@dataclass(frozen=True, slots=True)
class SteeringCalibrationConfig:
    """Servo calibration for one physical car.

    ``direction_sign`` converts logical vehicle yaw to the factory curve
    argument (verified -1.0 on this car: logical left evaluates the curve at a
    negative angle).  ``curve_a0`` is the constant term of the factory cubic.
    ``curve_scale`` is the factory pulses-per-radian scale.
    """

    direction_sign: float = -1.0
    logical_right_max_rad: float = -0.32
    logical_left_max_rad: float = 0.49
    calibration_min_rad: float = -0.49
    calibration_max_rad: float = 0.32
    pwm_min_us: int = 800
    pwm_max_us: int = 2200
    factory_center_us: int = 1501
    center_us: int = 1580
    curve_a3: float = -0.628
    curve_a2: float = 1.269
    curve_a1: float = -1.772
    curve_a0: float = 1.573
    curve_scale: float = 640.62

    def __post_init__(self) -> None:
        for name in (
            "direction_sign",
            "logical_right_max_rad",
            "logical_left_max_rad",
            "calibration_min_rad",
            "calibration_max_rad",
            "curve_a3",
            "curve_a2",
            "curve_a1",
            "curve_a0",
            "curve_scale",
        ):
            _require_finite(f"vehicle.steering.{name}", getattr(self, name))
        if self.direction_sign not in (-1.0, 1.0):
            raise ValueError(
                "vehicle.steering.direction_sign must be +1 or -1"
            )
        if self.logical_right_max_rad >= 0.0:
            raise ValueError(
                "vehicle.steering.logical_right_max_rad must be negative"
            )
        if self.logical_left_max_rad <= 0.0:
            raise ValueError(
                "vehicle.steering.logical_left_max_rad must be positive"
            )
        if not self.calibration_min_rad < self.calibration_max_rad:
            raise ValueError(
                "vehicle.steering.calibration_min_rad must be below "
                "calibration_max_rad"
            )
        if not self.pwm_min_us < self.center_us < self.pwm_max_us:
            raise ValueError(
                "vehicle.steering pwm must satisfy "
                "pwm_min_us < center_us < pwm_max_us"
            )
        if self.factory_center_us <= 0 or self.curve_scale <= 0.0:
            raise ValueError(
                "vehicle.steering factory_center_us and curve_scale "
                "must be positive"
            )


@dataclass(frozen=True, slots=True)
class VehicleConfig:
    """Measured geometry, drive-board limits and servo calibration."""

    geometry: VehicleGeometryConfig = field(default_factory=VehicleGeometryConfig)
    drive: VehicleDriveConfig = field(default_factory=VehicleDriveConfig)
    steering: SteeringCalibrationConfig = field(
        default_factory=SteeringCalibrationConfig
    )


# ============================================================
# 传感器
# ============================================================


@dataclass(frozen=True, slots=True)
class RadarMountConfig:
    """Radar origin/heading relative to the vehicle rear-axle centre.

    ``x_forward_cm`` is forward of the rear axle, ``y_left_cm`` is left of it,
    ``yaw_cw_deg`` is clockwise-positive mount yaw relative to the car front.
    """

    x_forward_cm: float = 0.0
    y_left_cm: float = 0.0
    yaw_cw_deg: float = 0.0

    def __post_init__(self) -> None:
        _require_finite("sensors.radar.mount.x_forward_cm", self.x_forward_cm)
        _require_finite("sensors.radar.mount.y_left_cm", self.y_left_cm)
        _require_finite("sensors.radar.mount.yaw_cw_deg", self.yaw_cw_deg)


@dataclass(frozen=True, slots=True)
class SensorRadarConfig:
    mount: RadarMountConfig = field(default_factory=RadarMountConfig)


@dataclass(frozen=True, slots=True)
class CameraPerspectiveConfig:
    """Bird's-eye perspective calibration for the current camera mount."""

    source_points_norm: tuple[tuple[float, float], ...] = (
        (0.02, 0.66),
        (0.93, 0.66),
        (0.68, 0.02),
        (0.23, 0.02),
    )
    output_width_px: int = 320
    output_height_px: int = 400
    ground_width_cm: float = 80.0
    ground_depth_cm: float = 100.0

    def __post_init__(self) -> None:
        if self.output_width_px < 64 or self.output_height_px < 64:
            raise ValueError(
                "sensors.camera.perspective output dimensions are too small"
            )
        _require_positive(
            "sensors.camera.perspective.ground_width_cm",
            self.ground_width_cm,
        )
        _require_positive(
            "sensors.camera.perspective.ground_depth_cm",
            self.ground_depth_cm,
        )
        if len(self.source_points_norm) != 4:
            raise ValueError(
                "sensors.camera.perspective.source_points_norm must contain "
                "exactly four points"
            )
        for index, (x, y) in enumerate(self.source_points_norm):
            if not (0.0 <= float(x) <= 1.0 and 0.0 <= float(y) <= 1.0):
                raise ValueError(
                    "sensors.camera.perspective.source_points_norm point "
                    f"{index} must lie in [0, 1]"
                )


@dataclass(frozen=True, slots=True)
class CameraLineConfig:
    """Black-line vision tuning for the current camera mount/field."""

    require_adaptive_confirmation: bool = False
    scan_near_cm: float = 12.0
    scan_far_cm: float = 72.0
    minimum_band_fill_ratio: float = 0.20
    use_expected_width_window: bool = True
    expected_line_width_cm: float = 28.0
    minimum_line_width_cm: float = 10.0
    maximum_line_width_cm: float = 40.0
    maximum_line_internal_gap_cm: float = 8.0
    maximum_center_jump_cm: float = 18.0
    morphology_close_size: int = 9
    polynomial_smoothing_alpha: float = 0.32
    transverse_stop_max_height_cm: float = 8.0
    round_marker_min_height_cm: float = 12.0
    continuity_weight: float = 0.12

    def __post_init__(self) -> None:
        _require_positive(
            "sensors.camera.line.scan_near_cm", self.scan_near_cm
        )
        _require_positive(
            "sensors.camera.line.scan_far_cm", self.scan_far_cm
        )
        if not self.scan_near_cm < self.scan_far_cm:
            raise ValueError(
                "sensors.camera.line.scan_near_cm must be below scan_far_cm"
            )
        if not (
            0.0 < self.minimum_line_width_cm
            <= self.expected_line_width_cm
            <= self.maximum_line_width_cm
        ):
            raise ValueError(
                "sensors.camera.line line width limits are invalid "
                "(min <= expected <= max)"
            )
        if self.maximum_line_internal_gap_cm < 0.0:
            raise ValueError(
                "sensors.camera.line.maximum_line_internal_gap_cm cannot be "
                "negative"
            )
        if self.morphology_close_size < 1:
            raise ValueError(
                "sensors.camera.line.morphology_close_size must be positive"
            )
        if not 0.0 < self.polynomial_smoothing_alpha <= 1.0:
            raise ValueError(
                "sensors.camera.line.polynomial_smoothing_alpha must be in "
                "(0, 1]"
            )


@dataclass(frozen=True, slots=True)
class SensorCameraConfig:
    perspective: CameraPerspectiveConfig = field(
        default_factory=CameraPerspectiveConfig
    )
    line: CameraLineConfig = field(default_factory=CameraLineConfig)


@dataclass(frozen=True, slots=True)
class SensorsConfig:
    radar: SensorRadarConfig = field(default_factory=SensorRadarConfig)
    camera: SensorCameraConfig = field(default_factory=SensorCameraConfig)


# ============================================================
# 比赛任务
# ============================================================


@dataclass(frozen=True, slots=True)
class MissionCommonConfig:
    """Startup / calibration parameters shared by both tasks."""

    radar_center_behind_a_cm: float = 20.0
    startup_scan_count: int = 3
    calibration_timeout_s: float = 30.0
    camera_correction_enabled: bool = True
    fleet_position_reporting_enabled: bool = True

    def __post_init__(self) -> None:
        _require_non_negative(
            "missions.common.radar_center_behind_a_cm",
            self.radar_center_behind_a_cm,
        )
        if self.startup_scan_count <= 0:
            raise ValueError("missions.common.startup_scan_count must be positive")
        _require_positive(
            "missions.common.calibration_timeout_s",
            self.calibration_timeout_s,
        )


@dataclass(frozen=True, slots=True)
class Task1Config:
    """Task 1 (payload drop) mission parameters."""

    fleet_mission_request_state: int = 13
    completion_alarm_seconds: float = 1.0
    ab_speed_cm_s: float = 8.0
    bc_speed_cm_s: float = 15.0
    cd_speed_cm_s: float = 20.0
    da_speed_cm_s: float = 15.0

    def __post_init__(self) -> None:
        _require_int_in_range(
            "missions.task1.fleet_mission_request_state",
            self.fleet_mission_request_state,
            0,
            255,
        )
        _require_non_negative(
            "missions.task1.completion_alarm_seconds",
            self.completion_alarm_seconds,
        )
        for name in (
            "ab_speed_cm_s",
            "bc_speed_cm_s",
            "cd_speed_cm_s",
            "da_speed_cm_s",
        ):
            _require_positive(f"missions.task1.{name}", getattr(self, name))


@dataclass(frozen=True, slots=True)
class Task2Config:
    """Task 2 (dynamic landing) mission parameters.

    CD uses ``cd_speed_before_retakeoff_cm_s`` until the drone confirms the
    platform retakeoff, then switches to ``cd_speed_after_retakeoff_cm_s``.
    """

    fleet_mission_request_state: int = 14
    completion_alarm_seconds: float = 1.0
    ab_speed_cm_s: float = 25.0
    bc_speed_cm_s: float = 9.0
    cd_speed_before_retakeoff_cm_s: float = 4.0
    cd_speed_after_retakeoff_cm_s: float = 30.0
    da_speed_cm_s: float = 30.0

    def __post_init__(self) -> None:
        _require_int_in_range(
            "missions.task2.fleet_mission_request_state",
            self.fleet_mission_request_state,
            0,
            255,
        )
        _require_non_negative(
            "missions.task2.completion_alarm_seconds",
            self.completion_alarm_seconds,
        )
        for name in (
            "ab_speed_cm_s",
            "bc_speed_cm_s",
            "cd_speed_before_retakeoff_cm_s",
            "cd_speed_after_retakeoff_cm_s",
            "da_speed_cm_s",
        ):
            _require_positive(f"missions.task2.{name}", getattr(self, name))


@dataclass(frozen=True, slots=True)
class MissionControlConfig:
    """Course-control tuning that depends on the camera mount / real-car look.

    Students changing the camera mount or redoing visual calibration edit
    these values in TOML; they never edit the main program.

    ``steering_min_rad`` / ``steering_max_rad`` are optional steering clamps
    applied by the main program after camera fusion.  When omitted they are
    derived from the vehicle profile (right mechanical limit and left minimum
    turn-radius limit), which keeps the verified car behaviour unchanged.
    """

    # [missions.control.camera] - soft camera steering correction.
    camera_lateral_deadband_cm: float = 10.0
    camera_steering_gain_rad_per_cm: float = 0.010
    camera_max_steering_correction_rad: float = 0.140

    # [missions.control] - first-AB strong heading alignment.
    ab_start_alignment_full_end_progress_cm: float = 80.0
    ab_start_alignment_fade_end_progress_cm: float = 100.0
    ab_start_heading_gain: float = 1.30
    ab_start_max_heading_correction_rad: float = 0.180
    ab_start_max_total_camera_correction_rad: float = 0.220
    ab_start_min_valid_frames: int = 2
    ab_start_max_curvature_per_cm: float = 0.003
    ab_start_max_forward_heading_change_rad: float = 0.080

    # First-AB bounded lateral assist while mission-frame alignment ramps in.
    ab_line_assist_full_end_progress_cm: float = 100.0
    ab_line_assist_fade_end_progress_cm: float = 135.0
    ab_line_assist_lateral_deadband_cm: float = 2.0
    ab_line_assist_gain_rad_per_cm: float = 0.005
    ab_line_assist_max_correction_rad: float = 0.060
    ab_line_assist_min_valid_frames: int = 3
    ab_line_assist_max_curvature_per_cm: float = 0.004
    ab_line_assist_max_forward_heading_change_rad: float = 0.100

    # AB lateral alignment learning (translation-only mission-frame offset).
    ab_lateral_alignment_learning_start_progress_cm: float = 50.0
    ab_lateral_alignment_learning_end_progress_cm: float = 90.0
    ab_lateral_alignment_ramp_start_progress_cm: float = 60.0
    ab_lateral_alignment_full_progress_cm: float = 100.0
    ab_lateral_alignment_terminal_fade_distance_cm: float = 65.0
    ab_lateral_alignment_min_valid_frames: int = 4
    ab_lateral_alignment_required_measurements: int = 5
    ab_lateral_alignment_max_abs_offset_cm: float = 12.0
    ab_lateral_alignment_max_mad_cm: float = 1.5
    ab_lateral_alignment_max_curvature_per_cm: float = 0.003
    ab_lateral_alignment_max_forward_heading_change_rad: float = 0.080

    # BC entry (first right semicircle) camera limit.
    bc_entry_limit_end_progress_cm: float = 210.0
    bc_entry_min_right_correction_rad: float = -0.012

    # Terminal-DA visual quality check near A.
    final_da_visual_window_cm: float = 65.0
    final_da_visual_hold_s: float = 0.65
    final_a_max_camera_error_cm: float = 6.0

    # FleetBus terminal reporting / trace drain timing.
    fleet_terminal_report_grace_s: float = 3.0
    fleet_trace_drain_timeout_s: float = 6.0

    # Optional explicit steering clamps; None means "derive from profile".
    steering_min_rad: float | None = None
    steering_max_rad: float | None = None

    def __post_init__(self) -> None:
        if self.ab_start_min_valid_frames <= 0:
            raise ValueError(
                "missions.control.ab_start_min_valid_frames must be positive"
            )
        if self.ab_line_assist_min_valid_frames <= 0:
            raise ValueError(
                "missions.control.ab_line_assist_min_valid_frames must be positive"
            )
        if self.ab_lateral_alignment_min_valid_frames <= 0:
            raise ValueError(
                "missions.control.ab_lateral_alignment_min_valid_frames "
                "must be positive"
            )
        if self.ab_lateral_alignment_required_measurements <= 0:
            raise ValueError(
                "missions.control.ab_lateral_alignment_required_measurements "
                "must be positive"
            )
        if self.steering_min_rad is not None:
            _require_finite("missions.control.steering_min_rad", self.steering_min_rad)
        if self.steering_max_rad is not None:
            _require_finite("missions.control.steering_max_rad", self.steering_max_rad)


@dataclass(frozen=True, slots=True)
class MissionsConfig:
    common: MissionCommonConfig = field(default_factory=MissionCommonConfig)
    task1: Task1Config = field(default_factory=Task1Config)
    task2: Task2Config = field(default_factory=Task2Config)
    control: MissionControlConfig = field(default_factory=MissionControlConfig)


# ============================================================
# Runtime state (现场可改, 不属于车辆静态标定)
# ============================================================


@dataclass(frozen=True, slots=True)
class RuntimeStateConfig:
    """On-site adjustable runtime state (not vehicle static calibration).

    The serial screen may switch ``radar_center_behind_a_cm`` between the
    values in ``allowed_radar_center_behind_a_cm``.  The selection is stored
    in ``state_file`` (JSON) and takes precedence over the TOML default.
    """

    enabled: bool = True
    state_file: str = "runtime/car_state.json"
    allowed_radar_center_behind_a_cm: tuple[float, ...] = (20.0, 36.5)

    def __post_init__(self) -> None:
        if not self.state_file:
            raise ValueError("runtime.state_file must not be empty")
        if len(self.allowed_radar_center_behind_a_cm) < 1:
            raise ValueError(
                "runtime.allowed_radar_center_behind_a_cm must not be empty"
            )


# ============================================================
# 顶层配置
# ============================================================

DEFAULT_SCHEMA_VERSION: Final[int] = 1


@dataclass(frozen=True, slots=True)
class CarConfig:
    """One fully validated vehicle profile."""

    profile: ProfileConfig = field(default_factory=ProfileConfig)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    devices: DevicesConfig = field(default_factory=DevicesConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    sensors: SensorsConfig = field(default_factory=SensorsConfig)
    missions: MissionsConfig = field(default_factory=MissionsConfig)
    runtime: RuntimeStateConfig = field(default_factory=RuntimeStateConfig)
    schema_version: int = DEFAULT_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != DEFAULT_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported config schema_version {self.schema_version}"
            )
