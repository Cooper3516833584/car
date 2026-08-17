#!/usr/bin/env python3
"""Run one radar-driven fixed-track lap with conservative camera correction.

``main_fixed_track_test.py`` remains the unchanged radar-only rollback entry.
This entry uses the same radar calibration, fixed track and Pure Pursuit
controller, then adds only a small camera-derived steering increment when the
black line is reliably visible and the lateral error is already significant.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
import math
import os
from pathlib import Path
import queue
import signal
import statistics
import sys
import threading
import time

from components.ackermann_drive import DEFAULT_HARDWARE_LOCK_PATH
from components.vehicle_defaults import (
    DEFAULT_BODY_LENGTH_MM,
    DEFAULT_BODY_WIDTH_MM,
    DEFAULT_FIRMWARE_TRACK_WIDTH_MM,
    DEFAULT_MIN_TURN_RADIUS_MM,
    DEFAULT_OUTER_WHEEL_WIDTH_MM,
    DEFAULT_PHYSICAL_TRACK_WIDTH_MM,
    DEFAULT_REAR_AXLE_TO_BODY_CENTER_MM,
    DEFAULT_WHEEL_THICKNESS_MM,
    DEFAULT_WHEELBASE_MM,
)
from components import (
    AckermannDrive,
    CompetitionTrack,
    CompetitionTrackFollower,
    CompetitionTrackSpeedProfile,
    DEFAULT_D500_PORT,
    DEFAULT_HC14_PORT,
    D500RadarComponent,
    LineVisionConfig,
    Pose2D,
    RadarLocalizationUpdate,
    RadarMount,
    RadarScan,
    RectangleFieldCalibrator,
    SerialCommunicationDriver,
    SoundLightAlarm,
    AlarmGPIOError,
    TrackFollowerState,
    TrackSegment,
    WallFusionConfig,
    WallLineConfig,
    rebase_calibration_to_start_pose,
)
from components.camera_line_correction import (
    CameraLineCorrectionConfig,
    CameraLineCorrectionState,
    CameraLineSteeringCorrector,
)
from components.fleet_car_node import FleetCarNode
from components.fleet_trace import TraceSamplingOptions
from components.fleet_models import (
    AckReason as FleetAckReason,
    AckStatus as FleetAckStatus,
    CarFleetState,
    CommandResult as FleetCommandResult,
    NodeFlags as FleetNodeFlags,
)
from components.navigation import (
    NavigationPose,
    NavigationState,
    radar_yaw_to_navigation_heading,
)
from components.steering_servo import (
    DEFAULT_STEERING_CALIBRATION,
    SteeringCalibration,
)
from config.loader import load_car_config
from config.models import MissionControlConfig


# FleetBus position reports are replies to the read-only ground-station POLL.
# Coordinates are centimetres relative to this run's radar-rebased start pose.
FLEET_POSITION_STALE_TIMEOUT_S = 0.5

# FleetBus operation-state value reported when the terminal camera/radar
# disagreement makes the reported pose untrustworthy (protocol semantics).
CAR_OPERATION_LOCALIZATION_LOST = 10


LOG = logging.getLogger("radar-camera-line-main")
LOG_FILENAME = "car-main.log"
LOG_MAX_BYTES = 20 * 1024 * 1024
LOG_BACKUP_COUNT = 10
_LOG_LISTENER: QueueListener | None = None


def default_log_dir() -> Path:
    configured = os.environ.get("CAR_LOG_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path(__file__).resolve().parent / "logs"


def configure_logging(
    log_dir: str | os.PathLike[str],
    console_level: str,
) -> Path:
    global _LOG_LISTENER
    shutdown_logging()
    directory = Path(log_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    log_path = directory / LOG_FILENAME
    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)s %(process)d %(threadName)s "
        "%(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    detailed_file = RotatingFileHandler(
        log_path,
        maxBytes=LOG_MAX_BYTES,
        backupCount=LOG_BACKUP_COUNT,
        encoding="utf-8",
    )
    detailed_file.setLevel(logging.DEBUG)
    detailed_file.setFormatter(formatter)
    log_queue: queue.SimpleQueue[logging.LogRecord] = queue.SimpleQueue()
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.addHandler(QueueHandler(log_queue))
    handlers: list[logging.Handler] = [detailed_file]
    if console_level != "OFF":
        console = logging.StreamHandler()
        console.setLevel(getattr(logging, console_level))
        console.setFormatter(formatter)
        handlers.insert(0, console)
    _LOG_LISTENER = QueueListener(
        log_queue,
        *handlers,
        respect_handler_level=True,
    )
    _LOG_LISTENER.start()
    logging.captureWarnings(True)
    LOG.info(
        "logging enabled file=%s console_level=%s",
        log_path,
        console_level,
    )
    return log_path


def shutdown_logging() -> None:
    global _LOG_LISTENER
    listener, _LOG_LISTENER = _LOG_LISTENER, None
    if listener is not None:
        listener.stop()
        for handler in listener.handlers:
            handler.flush()
            handler.close()
    root = logging.getLogger()
    for handler in tuple(root.handlers):
        root.removeHandler(handler)
        handler.close()


def _camera_source(value: str) -> int | str:
    try:
        source = int(value)
    except ValueError:
        if not value:
            raise argparse.ArgumentTypeError("camera source cannot be empty")
        return value
    if source < 0:
        raise argparse.ArgumentTypeError("camera index cannot be negative")
    return source


@dataclass(frozen=True, slots=True)
class MainConfig:
    radar_port: str = DEFAULT_D500_PORT
    radar_mount: RadarMount = RadarMount()
    startup_scan_count: int = 3
    calibration_timeout_s: float = 30.0
    radar_center_behind_a_cm: float = 20.0
    ab_speed_cm_s: float = 8.0
    bc_speed_cm_s: float = 15.0
    cd_speed_cm_s: float = 20.0
    cd_second_speed_cm_s: float | None = None
    da_speed_cm_s: float = 15.0
    camera_source: int | str = 0
    camera_correction_enabled: bool = True
    camera_correction: CameraLineCorrectionConfig = (
        CameraLineCorrectionConfig(
            lateral_deadband_cm=10.0,
            steering_gain_rad_per_cm=0.010,
            maximum_abs_correction_rad=0.140,
        )
    )
    fleet_position_reporting_enabled: bool = True
    fleet_link_port: str = DEFAULT_HC14_PORT
    fleet_position_only: bool = False
    fleet_wait_for_start: bool = False
    fleet_mission_request_state: int | None = None
    completion_alarm_seconds: float = 0.0
    # Course-control tuning from [missions.control] (camera mount dependent).
    mission_control: MissionControlConfig = MissionControlConfig()
    # Steering clamp applied after camera fusion; derived from the profile by
    # the composition root (servo right bound / geometry left bound).
    vehicle_steering_min_rad: float = -0.32
    vehicle_steering_max_rad: float = 0.336
    fleet_terminal_report_grace_s: float = 3.0
    fleet_trace_drain_timeout_s: float = 6.0
    # Current front-camera vision calibration (device + perspective + line
    # tuning) built from [devices.camera] and [sensors.camera].
    vision_config: LineVisionConfig = LineVisionConfig()
    # Unified drive construction data from [devices.motor] / [vehicle].
    motor_device: str = ""
    wheelbase_mm: float = DEFAULT_WHEELBASE_MM
    physical_track_width_mm: float = DEFAULT_PHYSICAL_TRACK_WIDTH_MM
    firmware_track_width_mm: float = DEFAULT_FIRMWARE_TRACK_WIDTH_MM
    min_turn_radius_mm: float = DEFAULT_MIN_TURN_RADIUS_MM
    allow_in_place_rotation: bool = False
    steering_calibration: SteeringCalibration = DEFAULT_STEERING_CALIBRATION
    hardware_lock_path: str | None = DEFAULT_HARDWARE_LOCK_PATH
    # Alarm GPIO from [hardware.alarm_gpio].
    alarm_sysfs_root: str = "/sys/class/gpio"
    alarm_bank_label: str = "gpio4"
    alarm_line_offset: int = 11
    alarm_active_low: bool = True

    def __post_init__(self) -> None:
        if self.startup_scan_count <= 0:
            raise ValueError("startup_scan_count must be positive")
        if self.calibration_timeout_s <= 0.0:
            raise ValueError("calibration_timeout_s must be positive")
        if self.radar_center_behind_a_cm < 0.0:
            raise ValueError("radar_center_behind_a_cm cannot be negative")
        for name, value in (
            ("ab_speed_cm_s", self.ab_speed_cm_s),
            ("bc_speed_cm_s", self.bc_speed_cm_s),
            ("cd_speed_cm_s", self.cd_speed_cm_s),
            ("da_speed_cm_s", self.da_speed_cm_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.cd_second_speed_cm_s is not None and (
            not math.isfinite(self.cd_second_speed_cm_s)
            or self.cd_second_speed_cm_s <= 0.0
        ):
            raise ValueError(
                "cd_second_speed_cm_s must be positive and finite"
            )
        if isinstance(self.camera_source, int) and self.camera_source < 0:
            raise ValueError("camera_source cannot be negative")
        if isinstance(self.camera_source, str) and not self.camera_source:
            raise ValueError("camera_source cannot be empty")
        if not self.fleet_link_port:
            raise ValueError("fleet_link_port cannot be empty")
        if (
            self.fleet_position_only
            and not self.fleet_position_reporting_enabled
        ):
            raise ValueError(
                "fleet_position_only requires FleetBus position reporting"
            )
        if self.fleet_wait_for_start and not self.fleet_position_reporting_enabled:
            raise ValueError("fleet_wait_for_start requires FleetBus")
        if (
            self.fleet_mission_request_state is not None
            and not 0 <= self.fleet_mission_request_state <= 255
        ):
            raise ValueError("fleet_mission_request_state must fit u8")
        if (
            not math.isfinite(self.completion_alarm_seconds)
            or self.completion_alarm_seconds < 0.0
        ):
            raise ValueError(
                "completion_alarm_seconds must be finite and non-negative"
            )
        if (
            not math.isfinite(self.vehicle_steering_min_rad)
            or not math.isfinite(self.vehicle_steering_max_rad)
            or self.vehicle_steering_min_rad > self.vehicle_steering_max_rad
        ):
            raise ValueError("vehicle steering clamp must be finite and ordered")
        for name in (
            "fleet_terminal_report_grace_s",
            "fleet_trace_drain_timeout_s",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

    @property
    def speed_profile(self) -> CompetitionTrackSpeedProfile:
        return CompetitionTrackSpeedProfile(
            self.ab_speed_cm_s,
            self.bc_speed_cm_s,
            self.cd_speed_cm_s,
            self.da_speed_cm_s,
        )

@dataclass(frozen=True, slots=True)
class CarRuntimeSnapshot:
    """Atomic read-only FleetBus view of the local navigation runtime."""

    ready: bool
    map_ready: bool
    pose: NavigationPose | None
    navigation_state: NavigationState
    localization_degraded: bool
    error_code: int
    localization_timeout_s: float = FLEET_POSITION_STALE_TIMEOUT_S


class _CameraCorrectedDrive:
    """Fusion-only drive view that leaves the shared radar follower untouched."""

    def __init__(self, drive: AckermannDrive, steering_adjuster) -> None:
        self._drive = drive
        self._steering_adjuster = steering_adjuster

    def set_motion(
        self,
        centre_speed_mm_s: float,
        steering_angle_rad: float,
        *args,
        **kwargs,
    ):
        adjusted = self._steering_adjuster(
            steering_angle_rad,
            abs(float(centre_speed_mm_s)) / 10.0,
        )
        return self._drive.set_motion(
            centre_speed_mm_s,
            adjusted,
            *args,
            **kwargs,
        )

    def stop(self, *args, **kwargs):
        return self._drive.stop(*args, **kwargs)


class RadarCameraLineApplication:
    """Radar authority with bounded camera steering correction."""

    def __init__(self, config: MainConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._completed_event = threading.Event()
        self._mission_start_event = threading.Event()
        self._scan_event = threading.Event()
        self._startup_scans: list[RadarScan] = []
        self._ready = False
        self._map_ready = False
        self._latest_navigation_pose = None
        self._localization_degraded = False
        self._fleet_error_code = 0
        self._closed = False
        self._last_camera_error: str | None = None
        self._final_da_visual_error_cm: float | None = None
        self._final_da_visual_timestamp_s: float | None = None
        self._terminal_camera_disagreement = False
        self._ab_lateral_alignment_offset_cm = 0.0
        self._ab_lateral_alignment_locked = False
        self._ab_lateral_alignment_measurements_cm: list[float] = []
        self._ab_lateral_alignment_last_camera_timestamp_s: float | None = None
        self._started_at = time.monotonic()
        self._calibrating = False
        self._alarm = None
        self._completion_alarm_lock = threading.Lock()
        self._completion_alarm_started = False
        self._completion_alarm_thread = None
        self._completion_alarm_device = None

        max_wheel_speed_mm_s = max(
            300.0,
            config.speed_profile.max_speed_cm_s * 12.0,
            (config.cd_second_speed_cm_s or 0.0) * 12.0,
        )
        self.drive = AckermannDrive.from_config(
            device=config.motor_device,
            wheelbase_mm=config.wheelbase_mm,
            track_width_mm=config.physical_track_width_mm,
            firmware_track_width_mm=config.firmware_track_width_mm,
            max_wheel_speed_mm_s=max_wheel_speed_mm_s,
            min_turn_radius_mm=config.min_turn_radius_mm,
            allow_in_place_rotation=config.allow_in_place_rotation,
            steering_calibration=config.steering_calibration,
            hardware_lock_path=config.hardware_lock_path,
        )
        self.track = CompetitionTrack.build(
            reference_offset_cm=config.radar_center_behind_a_cm,
        )
        self.camera_corrector = CameraLineSteeringCorrector(
            camera_index=config.camera_source,
            vision_config=self._front_camera_vision_config(),
            correction_config=config.camera_correction,
            on_state_changed=self._on_camera_state,
        )
        self._corrected_drive = _CameraCorrectedDrive(
            self.drive,
            self._adjust_radar_steering,
        )

        # Keep the copied radar follower unchanged and insert correction only
        # in this entry's private drive view.
        self.follower = CompetitionTrackFollower(
            drive=self._corrected_drive,
            track=self.track,
            speed_profile=config.speed_profile,
            on_state_changed=self._on_follower_state,
        )
        self._follower_state = self.follower.state
        self.calibrator = RectangleFieldCalibrator(
            mount=config.radar_mount,
        )
        self.radar = D500RadarComponent(
            port=config.radar_port,
            mount=config.radar_mount,
            on_update=self._on_radar_update,
            on_connected=lambda: LOG.info(
                "D500 connected on %s",
                config.radar_port,
            ),
            on_disconnected=lambda error: LOG.warning(
                "D500 disconnected: %s",
                error,
            ),
        )
        self.fleet_link = None
        self.fleet_node = None
        if config.fleet_position_reporting_enabled:
            self.fleet_link = SerialCommunicationDriver(
                port=config.fleet_link_port,
                on_bytes=self._on_fleet_frame,
                on_connected=lambda: LOG.info(
                    "FleetBus HC-14 connected on %s",
                    config.fleet_link_port,
                ),
                on_disconnected=lambda error: LOG.warning(
                    "FleetBus HC-14 disconnected: %s",
                    error,
                ),
                on_callback_error=lambda error: LOG.error(
                    "FleetBus HC-14 callback failed: %s",
                    error,
                ),
            )
            self.fleet_node = FleetCarNode(
                writer=self._send_fleet_frame,
                state_provider=self._fleet_state,
                on_set_coordinate_frame=self._fleet_unsupported,
                on_navigate=self._fleet_unsupported,
                on_stop=self._fleet_stop,
                on_set_alarm=self._fleet_set_alarm,
                on_start_mission=self._fleet_start_mission,
                on_switch_task2_cd_speed=(
                    self._fleet_switch_task2_cd_speed
                    if config.cd_second_speed_cm_s is not None
                    else None
                ),
                trace_options=TraceSamplingOptions(
                    enabled=True,
                    sample_interval_s=0.50,
                    buffer_capacity=600,
                    min_distance_cm=5.0,
                    stationary_keepalive_s=2.0,
                ),
            )

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready

    def fleet_runtime_snapshot(self) -> CarRuntimeSnapshot:
        """Return one lock-consistent local pose/status snapshot without I/O."""
        with self._lock:
            follower_state = self._follower_state
            if follower_state.completed:
                navigation_state = NavigationState.ARRIVED
            elif follower_state.running:
                navigation_state = NavigationState.FOLLOWING
            else:
                navigation_state = NavigationState.IDLE
            return CarRuntimeSnapshot(
                ready=self._ready,
                map_ready=self._map_ready,
                pose=self._latest_navigation_pose,
                navigation_state=navigation_state,
                localization_degraded=(
                    self._localization_degraded
                    or self._terminal_camera_disagreement
                ),
                error_code=self._fleet_error_code,
            )

    def _front_camera_vision_config(self) -> LineVisionConfig:
        """Return the current front-camera vision calibration.

        The values come from the TOML ``[devices.camera]`` and
        ``[sensors.camera]`` sections via ``MainConfig.vision_config``; a
        student who moves/re-calibrates the camera only edits the profile.
        """
        return self.config.vision_config

    def run(self) -> None:
        LOG.info(
            "vehicle must remain stationary at A during D500 calibration"
        )
        try:
            if self.fleet_node is not None and self.fleet_link is not None:
                self.fleet_node.start()
                self.fleet_link.start()
                LOG.info(
                    "FleetBus relative-position reporting enabled; "
                    "commands other than targeted stop are rejected"
                )
            with self._lock:
                self._calibrating = True
            self._calibrate_radar()
            with self._lock:
                self._calibrating = False
                self._ready = True
            if self.config.fleet_position_only:
                self.radar.set_motion_hint(False)
                self.radar.start()
                LOG.info(
                    "FleetBus position-only mode active; drive and camera "
                    "remain closed and the vehicle must stay stationary"
                )
                while not self._stop_event.wait(0.5):
                    pass
                return
            if self.config.fleet_wait_for_start:
                self.radar.set_motion_hint(False)
                self.radar.start()
                LOG.info("ready; waiting for FleetBus CAR_START_MISSION")
                while (
                    not self._stop_event.is_set()
                    and not self._mission_start_event.wait(0.2)
                ):
                    pass
                if self._stop_event.is_set():
                    return
                LOG.info("FleetBus start command accepted; beginning task 1")
            if self.config.camera_correction_enabled:
                try:
                    self.camera_corrector.start()
                    LOG.info(
                        "soft camera correction started source=%r "
                        "deadband_cm=%.1f max_correction_rad=%.3f",
                        self.config.camera_source,
                        self.config.camera_correction.lateral_deadband_cm,
                        self.config.camera_correction.maximum_abs_correction_rad,
                    )
                except Exception as exc:
                    # Camera is supplemental. A camera problem must never
                    # prevent the known-good radar-only lap from starting.
                    LOG.warning(
                        "camera correction unavailable; continuing radar-only: %s",
                        exc,
                    )
            else:
                LOG.info("camera correction disabled; running radar-only")

            self.drive.start()
            self.follower.start_mission()
            self.radar.set_motion_hint(True)
            self.radar.start()
            LOG.info(
                "one-lap radar+camera tracking started "
                "speeds_cm_s=AB:%.1f,BC:%.1f,CD:%.1f,DA:%.1f "
                "startup_approach_to_a_cm=%.1f",
                self.config.ab_speed_cm_s,
                self.config.bc_speed_cm_s,
                self.config.cd_speed_cm_s,
                self.config.da_speed_cm_s,
                self.config.radar_center_behind_a_cm,
            )
            while (
                not self._stop_event.is_set()
                and not self._completed_event.wait(0.5)
            ):
                pass
            if not self._stop_event.is_set():
                LOG.info(
                    "holding stopped state for %.1fs so D500 localization "
                    "and FleetBus can publish the terminal pose",
                    self.config.fleet_terminal_report_grace_s,
                )
                self._stop_event.wait(self.config.fleet_terminal_report_grace_s)
                if self.fleet_node is not None and not self._stop_event.is_set():
                    LOG.info(
                        "waiting up to %.1fs for FleetBus terminal trace drain",
                        self.config.fleet_trace_drain_timeout_s,
                    )
                    drained = self.fleet_node.wait_for_trace_drain(
                        self.config.fleet_trace_drain_timeout_s,
                        cancel_event=self._stop_event,
                    )
                    if drained:
                        LOG.info("FleetBus terminal trace drain confirmed")
                    else:
                        LOG.warning(
                            "FleetBus terminal trace drain timed out or was cancelled"
                        )
        finally:
            with self._lock:
                self._calibrating = False
            self.close()

    def _on_fleet_frame(self, frame: bytes) -> None:
        if self.fleet_node is not None:
            self.fleet_node.feed_frame(frame)

    def _send_fleet_frame(self, frame: bytes) -> None:
        link = self.fleet_link
        if link is None:
            return
        try:
            link.write(frame)
        except Exception as exc:
            LOG.warning("FleetBus position reply could not be sent: %s", exc)

    def _fleet_state(self) -> CarFleetState:
        now = time.monotonic()
        with self._lock:
            ready = self._ready
            calibrating = self._calibrating
            map_ready = self._map_ready
            degraded = (
                self._localization_degraded
                or self._terminal_camera_disagreement
            )
            pose = self._latest_navigation_pose
            follower_state = self._follower_state
            error_code = self._fleet_error_code
        pose_fresh = (
            pose is not None
            and now - pose.timestamp_s
            <= FLEET_POSITION_STALE_TIMEOUT_S
        )
        pose_valid = (
            ready
            and pose_fresh
            and not self._terminal_camera_disagreement
        )
        flags = 0
        if pose_valid:
            flags |= int(FleetNodeFlags.POSE_VALID)
        if ready:
            flags |= int(FleetNodeFlags.READY)
        if map_ready:
            flags |= int(FleetNodeFlags.MAP_READY)
        if follower_state.running:
            flags |= int(
                FleetNodeFlags.BUSY
                | FleetNodeFlags.ARMED_OR_MOTOR_ACTIVE
            )
        if ready and (degraded or not pose_valid):
            flags |= int(FleetNodeFlags.LOCALIZATION_DEGRADED)

        if pose is None:
            x_cm = y_cm = heading_cdeg = 0
        else:
            x_cm = round(pose.x_cm)
            y_cm = round(pose.y_cm)
            heading_cdeg = round(pose.heading_deg * 100.0) % 36000
        if (
            self.config.fleet_wait_for_start
            and not self._mission_start_event.is_set()
            and self.config.fleet_mission_request_state is not None
        ):
            operation_state = self.config.fleet_mission_request_state
        elif self._terminal_camera_disagreement:
            operation_state = CAR_OPERATION_LOCALIZATION_LOST
        elif follower_state.completed:
            operation_state = 7
        elif follower_state.running:
            operation_state = 4
        elif ready:
            operation_state = 2
        elif calibrating:
            operation_state = 1
        else:
            operation_state = 0
        return CarFleetState(
            flags,
            round((now - self._started_at) * 1000.0) & 0xFFFFFFFF,
            x_cm,
            y_cm,
            heading_cdeg,
            operation_state=operation_state,
            pose_quality=(
                2
                if pose_valid and degraded
                else (4 if pose_valid else (2 if pose is not None else 0))
            ),
            error_code=error_code,
            radar_center_behind_a_centi_cm=round(
                self.config.radar_center_behind_a_cm * 100.0
            ),
        )

    @staticmethod
    def _fleet_unsupported(*_args) -> FleetCommandResult:
        return FleetCommandResult(
            FleetAckStatus.REJECTED,
            FleetAckReason.UNSUPPORTED,
            "fixed-track position-reporting entry is read-only",
        )

    def _fleet_stop(self) -> FleetCommandResult:
        self.request_stop()
        return FleetCommandResult(FleetAckStatus.COMPLETED)

    def _fleet_start_mission(self) -> FleetCommandResult:
        if not self.config.fleet_wait_for_start:
            return self._fleet_unsupported()
        if not self.ready:
            return FleetCommandResult(
                FleetAckStatus.REJECTED,
                FleetAckReason.NOT_READY,
                "car calibration is not complete",
            )
        self._mission_start_event.set()
        return FleetCommandResult(FleetAckStatus.COMPLETED)

    def _fleet_switch_task2_cd_speed(self) -> FleetCommandResult:
        speed_cm_s = self.config.cd_second_speed_cm_s
        if speed_cm_s is None:
            return self._fleet_unsupported()
        if not self.follower.switch_cd_speed(speed_cm_s):
            LOG.warning(
                "task 2 CD speed switch rejected outside active CD segment"
            )
            return FleetCommandResult(
                FleetAckStatus.REJECTED,
                FleetAckReason.NOT_READY,
            )
        LOG.info("task 2 CD speed switched to %.1f cm/s", speed_cm_s)
        return FleetCommandResult(FleetAckStatus.COMPLETED)

    def _build_alarm(self) -> SoundLightAlarm:
        """Build the alarm from the profile's [hardware.alarm_gpio] section."""
        return SoundLightAlarm(
            sysfs_gpio_root=self.config.alarm_sysfs_root,
            bank_label=self.config.alarm_bank_label,
            line_offset=self.config.alarm_line_offset,
            active_low=self.config.alarm_active_low,
        )

    def _fleet_set_alarm(self, active: bool) -> FleetCommandResult:
        try:
            if self._alarm is None:
                self._alarm = self._build_alarm()
                if not self._alarm.is_initialized:
                    self._alarm.initialize()
            self._alarm.set_active(active)
        except AlarmGPIOError as exc:
            LOG.error("could not set car alarm active=%s: %s", active, exc)
            return FleetCommandResult(
                FleetAckStatus.FAILED,
                FleetAckReason.INTERNAL_ERROR,
                str(exc),
            )
        LOG.info("car alarm active=%s by FleetBus", active)
        return FleetCommandResult(FleetAckStatus.COMPLETED)

    def _calibrate_radar(self) -> None:
        with self._lock:
            self._startup_scans.clear()
            self._ready = False
        self.radar.start()
        if not self.radar.serial.wait_connected(
            min(3.0, self.config.calibration_timeout_s)
        ):
            self.radar.close()
            raise RuntimeError(
                f"cannot open configured radar serial device "
                f"{self.config.radar_port}; check the device path, serial "
                "permissions and the board UART wiring/overlay"
            )
        fitted = self._wait_for_rectangle_calibration()
        calibration = rebase_calibration_to_start_pose(fitted)

        self.radar.close()
        self.radar.assembler.reset()
        self.radar.odometry.reset(Pose2D())
        self.radar.global_map.clear()
        self.radar.alignment = calibration.local_to_global
        self.radar.enable_wall_fusion(
            calibration.wall_reference,
            line_config=WallLineConfig(rotation_adaptation=True),
            fusion_config=WallFusionConfig.car_slow_drift(
                position_gain=0.20,
            ),
        )
        with self._lock:
            self._map_ready = True
        LOG.info(
            "calibration complete; startup rear axle rebased to "
            "(0,0,0deg), A is %.1f cm ahead along AB "
            "bounds=x[%.1f,%.1f] y[%.1f,%.1f] fitted_lines=%d",
            self.config.radar_center_behind_a_cm,
            calibration.min_x_cm,
            calibration.max_x_cm,
            calibration.min_y_cm,
            calibration.max_y_cm,
            calibration.fitted_lines,
        )

    def _wait_for_rectangle_calibration(self):
        deadline = time.monotonic() + self.config.calibration_timeout_s
        last_error = (
            f"radar serial device {self.config.radar_port} is open but no "
            "complete scan arrived"
        )
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            self._scan_event.wait(0.5)
            self._scan_event.clear()
            with self._lock:
                scans = tuple(
                    self._startup_scans[-self.config.startup_scan_count :]
                )
            if len(scans) < self.config.startup_scan_count:
                continue
            try:
                return self.calibrator.calibrate(scans)
            except (ValueError, RuntimeError) as exc:
                if str(exc) != last_error:
                    LOG.warning("rectangle calibration retry: %s", exc)
                    last_error = str(exc)
        raise RuntimeError(
            f"rectangle field calibration timed out: {last_error}"
        )

    def _on_radar_update(self, update: RadarLocalizationUpdate) -> None:
        raw_pose: NavigationPose | None = None
        with self._lock:
            ready = self._ready
            if not ready:
                self._startup_scans.append(update.scan)
                limit = self.config.startup_scan_count * 2
                del self._startup_scans[:-limit]
                self._scan_event.set()
                return
            if update.global_pose is None or not update.odometry.accepted:
                self._localization_degraded = True
            else:
                raw_pose = NavigationPose(
                    x_cm=update.global_pose.x_cm,
                    y_cm=update.global_pose.y_cm,
                    heading_deg=radar_yaw_to_navigation_heading(
                        update.global_pose.yaw_cw_deg
                    ),
                    timestamp_s=time.monotonic(),
                )
                self._localization_degraded = False
        control_pose = None
        if raw_pose is not None:
            control_pose = self._ab_lateral_aligned_control_pose(raw_pose)
            with self._lock:
                # FleetBus and Pure Pursuit must expose/consume the same
                # calibrated mission-frame pose.  ICP remains untouched.
                self._latest_navigation_pose = control_pose
        try:
            self.follower.update_from_radar(
                update,
                control_pose_override=control_pose,
            )
        except BaseException:
            LOG.exception("track update failed; stopping")
            self.request_stop()

    def _ab_lateral_aligned_control_pose(
        self,
        radar_pose: NavigationPose,
        *,
        now_s: float | None = None,
    ) -> NavigationPose:
        """Return radar pose plus one robust first-AB lateral translation."""

        now = time.monotonic() if now_s is None else float(now_s)
        control = self.config.mission_control
        camera_state = self.camera_corrector.state
        observation = camera_state.observation
        with self._lock:
            follower_state = self._follower_state
            locked = self._ab_lateral_alignment_locked
            last_camera_timestamp_s = (
                self._ab_lateral_alignment_last_camera_timestamp_s
            )
        learning_allowed = (
            self.config.camera_correction_enabled
            and not locked
            and follower_state.running
            and not follower_state.completed
            and follower_state.segment is TrackSegment.AB
            and control.ab_lateral_alignment_learning_start_progress_cm
            <= follower_state.progress_cm
            < control.ab_lateral_alignment_learning_end_progress_cm
            and camera_state.active
            and camera_state.valid_frames
            >= control.ab_lateral_alignment_min_valid_frames
            and (
                last_camera_timestamp_s is None
                or camera_state.timestamp_s > last_camera_timestamp_s
            )
            and now - camera_state.timestamp_s
            <= self.config.camera_correction.stale_timeout_s
            and observation is not None
            and observation.detected
            and math.isfinite(observation.near_lateral_error_cm)
            and observation.confidence
            >= self.config.camera_correction.minimum_confidence
            and observation.visible_band_count
            >= self.config.camera_correction.minimum_visible_bands
            and observation.fit_rmse_cm
            <= self.config.camera_correction.maximum_fit_rmse_cm
            and not observation.round_marker_detected
            and not observation.transverse_line_detected
            and math.isfinite(observation.curvature_per_cm)
            and abs(observation.curvature_per_cm)
            <= control.ab_lateral_alignment_max_curvature_per_cm
            and math.isfinite(observation.forward_heading_change_rad)
            and abs(observation.forward_heading_change_rad)
            <= control.ab_lateral_alignment_max_forward_heading_change_rad
        )
        if learning_allowed:
            # Positive camera error requests a left correction, meaning the
            # vehicle is physically right (negative mission Y) of the line.
            measured_offset_cm = (
                -float(observation.near_lateral_error_cm)
                - radar_pose.y_cm
            )
            if (
                abs(measured_offset_cm)
                <= control.ab_lateral_alignment_max_abs_offset_cm
            ):
                with self._lock:
                    self._ab_lateral_alignment_measurements_cm.append(
                        measured_offset_cm
                    )
                    self._ab_lateral_alignment_last_camera_timestamp_s = (
                        camera_state.timestamp_s
                    )
                    measurements = tuple(
                        self._ab_lateral_alignment_measurements_cm
                    )
                LOG.debug(
                    "AB visual lateral alignment sample=%d "
                    "camera_error_cm=%.2f raw_radar_y_cm=%.2f "
                    "measured_offset_y_cm=%.2f",
                    len(measurements),
                    observation.near_lateral_error_cm,
                    radar_pose.y_cm,
                    measured_offset_cm,
                )
                if (
                    len(measurements)
                    >= control.ab_lateral_alignment_required_measurements
                ):
                    median_offset_cm = statistics.median(measurements)
                    median_absolute_deviation_cm = statistics.median(
                        abs(value - median_offset_cm)
                        for value in measurements
                    )
                    if (
                        median_absolute_deviation_cm
                        <= control.ab_lateral_alignment_max_mad_cm
                    ):
                        with self._lock:
                            self._ab_lateral_alignment_offset_cm = (
                                median_offset_cm
                            )
                            self._ab_lateral_alignment_locked = True
                        LOG.info(
                            "AB visual lateral alignment locked "
                            "offset_y_cm=%.2f samples=%d mad_cm=%.2f",
                            median_offset_cm,
                            len(measurements),
                            median_absolute_deviation_cm,
                        )

        with self._lock:
            offset_cm = self._ab_lateral_alignment_offset_cm
            locked = self._ab_lateral_alignment_locked
            follower_state = self._follower_state
        scale = self._ab_lateral_alignment_scale(
            follower_state,
            locked=locked,
        )
        if scale <= 0.0:
            return radar_pose
        aligned_pose = NavigationPose(
            x_cm=radar_pose.x_cm,
            y_cm=radar_pose.y_cm + offset_cm * scale,
            heading_deg=radar_pose.heading_deg,
            timestamp_s=radar_pose.timestamp_s,
        )
        LOG.debug(
            "AB lateral mission alignment raw_y_cm=%.2f "
            "offset_y_cm=%.2f scale=%.3f aligned_y_cm=%.2f",
            radar_pose.y_cm,
            offset_cm,
            scale,
            aligned_pose.y_cm,
        )
        return aligned_pose

    def _ab_lateral_alignment_scale(
        self,
        follower_state: TrackFollowerState,
        *,
        locked: bool,
    ) -> float:
        control = self.config.mission_control
        if not locked:
            return 0.0
        if follower_state.completed:
            return 0.0
        progress_cm = follower_state.progress_cm
        remaining_cm = self.track.finish_progress_cm - progress_cm
        if remaining_cm < control.ab_lateral_alignment_terminal_fade_distance_cm:
            return max(
                0.0,
                remaining_cm
                / control.ab_lateral_alignment_terminal_fade_distance_cm,
            )
        if progress_cm <= control.ab_lateral_alignment_ramp_start_progress_cm:
            return 0.0
        if progress_cm >= control.ab_lateral_alignment_full_progress_cm:
            return 1.0
        return (
            progress_cm - control.ab_lateral_alignment_ramp_start_progress_cm
        ) / (
            control.ab_lateral_alignment_full_progress_cm
            - control.ab_lateral_alignment_ramp_start_progress_cm
        )

    def _on_follower_state(self, state: TrackFollowerState) -> None:
        self.camera_corrector.set_curve_mode(
            state.segment in (TrackSegment.BC, TrackSegment.DA)
        )
        with self._lock:
            was_completed = self._follower_state.completed
            self._follower_state = state
        if state.completed:
            now_s = time.monotonic()
            control = self.config.mission_control
            with self._lock:
                visual_error_cm = self._final_da_visual_error_cm
                visual_timestamp_s = self._final_da_visual_timestamp_s
                disagreement = (
                    visual_error_cm is not None
                    and visual_timestamp_s is not None
                    and now_s - visual_timestamp_s
                    <= control.final_da_visual_hold_s
                    and abs(visual_error_cm)
                    > control.final_a_max_camera_error_cm
                )
                self._terminal_camera_disagreement = (
                    disagreement
                    or self.follower.terminal_hard_stop_triggered
                )
            self.radar.set_motion_hint(False)
            self._completed_event.set()
            if self.follower.terminal_hard_stop_triggered:
                LOG.warning(
                    "one lap ended at the terminal safety limit; final "
                    "position/cross-track/heading tolerance was not met"
                )
            elif disagreement:
                LOG.warning(
                    "one lap ended with radar/camera disagreement at A; "
                    "last_visual_error_cm=%.2f pose quality is degraded",
                    visual_error_cm,
                )
            else:
                LOG.info(
                    "startup approach %.1f cm plus one lap complete; "
                    "rear axle stopped at A",
                    self.config.radar_center_behind_a_cm,
                )
            if (
                not was_completed
                and self.config.completion_alarm_seconds > 0.0
            ):
                self._start_completion_alarm()

    def _start_completion_alarm(self) -> None:
        with self._completion_alarm_lock:
            if self._completion_alarm_started:
                return
            self._completion_alarm_started = True
            self._completion_alarm_thread = threading.Thread(
                target=self._run_completion_alarm,
                name="mission1-car-completion-alarm",
                daemon=False,
            )
            self._completion_alarm_thread.start()

    def _run_completion_alarm(self) -> None:
        alarm = None
        try:
            alarm = self._build_alarm()
            if not alarm.is_initialized:
                alarm.initialize()
            alarm.on()
            with self._completion_alarm_lock:
                self._completion_alarm_device = alarm
            self._stop_event.wait(self.config.completion_alarm_seconds)
        except Exception:
            LOG.exception("MISSION1 completion sound/light alarm failed")
        finally:
            try:
                if alarm is not None:
                    alarm.off()
                else:
                    fallback = self._build_alarm()
                    if not fallback.is_initialized:
                        fallback.initialize()
                    fallback.off()
            except Exception:
                LOG.exception("failed to turn off MISSION1 completion alarm")
            finally:
                with self._completion_alarm_lock:
                    self._completion_alarm_device = None

    def request_stop(self) -> None:
        self._stop_event.set()

    def _adjust_radar_steering(
        self,
        radar_steering_rad: float,
        speed_cm_s: float,
    ) -> float:
        """Fuse camera correction and the fixed-course final-DA trim."""

        if not self.config.camera_correction_enabled:
            return float(radar_steering_rad)
        try:
            control = self.config.mission_control
            now_s = time.monotonic()
            observed_correction = self.camera_corrector.correction_for_speed(
                speed_cm_s,
                now_s=now_s,
            )
            ab_start_alignment = self._ab_start_alignment_correction(
                now_s=now_s
            )
            ab_line_assist = self._ab_line_assist_correction(
                now_s=now_s
            )
            lateral_camera_correction = observed_correction
            if ab_line_assist is not None and abs(ab_line_assist) > 1e-9:
                if observed_correction * ab_line_assist < 0.0:
                    lateral_camera_correction = ab_line_assist
                elif abs(ab_line_assist) > abs(observed_correction):
                    lateral_camera_correction = ab_line_assist
            camera_correction = max(
                -control.ab_start_max_total_camera_correction_rad,
                min(
                    control.ab_start_max_total_camera_correction_rad,
                    lateral_camera_correction + ab_start_alignment,
                ),
            )
            course_limited_correction = self._apply_course_camera_limit(
                camera_correction
            )
            correction = course_limited_correction
            combined = float(radar_steering_rad) + correction
            adjusted = max(
                self.config.vehicle_steering_min_rad,
                min(self.config.vehicle_steering_max_rad, combined),
            )
            state = self.camera_corrector.state
            LOG.debug(
                "steering fusion radar_rad=%.4f camera_rad=%.4f "
                "final_rad=%.4f observed_camera_rad=%.4f "
                "ab_start_alignment_rad=%.4f "
                "ab_line_assist_rad=%.4f "
                "course_limited_camera_rad=%.4f "
                "camera_active=%s "
                "camera_error_cm=%.2f camera_confidence=%.2f",
                radar_steering_rad,
                correction,
                adjusted,
                observed_correction,
                ab_start_alignment,
                0.0 if ab_line_assist is None else ab_line_assist,
                course_limited_correction,
                state.active,
                state.lateral_error_cm,
                state.confidence,
            )
            return adjusted
        except Exception:
            LOG.exception(
                "camera steering adjustment failed; using radar command"
            )
            return float(radar_steering_rad)

    def _ab_start_alignment_correction(
        self,
        *,
        now_s: float | None = None,
    ) -> float:
        control = self.config.mission_control
        now = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            follower_state = self._follower_state
        camera_state = self.camera_corrector.state
        observation = camera_state.observation
        if (
            not follower_state.running
            or follower_state.completed
            or follower_state.segment is not TrackSegment.AB
            or follower_state.progress_cm
            >= control.ab_start_alignment_fade_end_progress_cm
            or now - camera_state.timestamp_s
            > self.config.camera_correction.stale_timeout_s
            or camera_state.valid_frames < control.ab_start_min_valid_frames
            or observation is None
            or not observation.detected
            or not math.isfinite(observation.heading_error_rad)
            or observation.confidence
            < self.config.camera_correction.minimum_confidence
            or observation.visible_band_count
            < self.config.camera_correction.minimum_visible_bands
            or observation.fit_rmse_cm
            > self.config.camera_correction.maximum_fit_rmse_cm
            or not math.isfinite(observation.curvature_per_cm)
            or abs(observation.curvature_per_cm)
            > control.ab_start_max_curvature_per_cm
            or not math.isfinite(
                observation.forward_heading_change_rad
            )
            or abs(observation.forward_heading_change_rad)
            > control.ab_start_max_forward_heading_change_rad
            or observation.round_marker_detected
            or observation.transverse_line_detected
        ):
            return 0.0

        fade_span_cm = (
            control.ab_start_alignment_fade_end_progress_cm
            - control.ab_start_alignment_full_end_progress_cm
        )
        fade_scale = (
            1.0
            if follower_state.progress_cm
            <= control.ab_start_alignment_full_end_progress_cm
            else max(
                0.0,
                (
                    control.ab_start_alignment_fade_end_progress_cm
                    - follower_state.progress_cm
                )
                / fade_span_cm,
            )
        )
        requested = (
            control.ab_start_heading_gain
            * float(observation.heading_error_rad)
        )
        bounded = max(
            -control.ab_start_max_heading_correction_rad,
            min(control.ab_start_max_heading_correction_rad, requested),
        )
        return bounded * fade_scale

    def _ab_line_assist_correction(
        self,
        *,
        now_s: float | None = None,
    ) -> float | None:
        """Return first-AB steering assist while pose alignment ramps in."""

        control = self.config.mission_control
        now = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            follower_state = self._follower_state
        camera_state = self.camera_corrector.state
        observation = camera_state.observation
        if (
            not follower_state.running
            or follower_state.completed
            or follower_state.segment is not TrackSegment.AB
            or follower_state.progress_cm
            >= control.ab_line_assist_fade_end_progress_cm
            or now - camera_state.timestamp_s
            > self.config.camera_correction.stale_timeout_s
            or camera_state.valid_frames
            < control.ab_line_assist_min_valid_frames
            or observation is None
            or not observation.detected
            or not math.isfinite(observation.near_lateral_error_cm)
            or observation.confidence
            < self.config.camera_correction.minimum_confidence
            or observation.visible_band_count
            < self.config.camera_correction.minimum_visible_bands
            or observation.fit_rmse_cm
            > self.config.camera_correction.maximum_fit_rmse_cm
            or not math.isfinite(observation.curvature_per_cm)
            or abs(observation.curvature_per_cm)
            > control.ab_line_assist_max_curvature_per_cm
            or not math.isfinite(
                observation.forward_heading_change_rad
            )
            or abs(observation.forward_heading_change_rad)
            > control.ab_line_assist_max_forward_heading_change_rad
            or observation.round_marker_detected
            or observation.transverse_line_detected
        ):
            return None

        magnitude = max(
            0.0,
            abs(float(observation.near_lateral_error_cm))
            - control.ab_line_assist_lateral_deadband_cm,
        )
        requested = math.copysign(
            min(
                control.ab_line_assist_max_correction_rad,
                control.ab_line_assist_gain_rad_per_cm * magnitude,
            ),
            observation.near_lateral_error_cm,
        )
        fade_span_cm = (
            control.ab_line_assist_fade_end_progress_cm
            - control.ab_line_assist_full_end_progress_cm
        )
        fade_scale = (
            1.0
            if follower_state.progress_cm
            <= control.ab_line_assist_full_end_progress_cm
            else max(
                0.0,
                (
                    control.ab_line_assist_fade_end_progress_cm
                    - follower_state.progress_cm
                )
                / fade_span_cm,
            )
        )
        with self._lock:
            alignment_scale = self._ab_lateral_alignment_scale(
                self._follower_state,
                locked=self._ab_lateral_alignment_locked,
            )
        return requested * fade_scale * (1.0 - alignment_scale)

    def _apply_course_camera_limit(self, correction_rad: float) -> float:
        control = self.config.mission_control
        with self._lock:
            state = self._follower_state
        if (
            state.running
            and not state.completed
            and state.segment is TrackSegment.BC
            and state.progress_cm < control.bc_entry_limit_end_progress_cm
        ):
            return max(
                float(correction_rad),
                control.bc_entry_min_right_correction_rad,
            )
        return float(correction_rad)

    def _on_camera_state(
        self,
        state: CameraLineCorrectionState,
    ) -> None:
        observation = state.observation
        with self._lock:
            follower_state = self._follower_state
        if (
            follower_state.running
            and not follower_state.completed
            and follower_state.segment is TrackSegment.DA
            and follower_state.progress_cm
            >= self.track.finish_progress_cm
            - self.config.mission_control.final_da_visual_window_cm
            and state.active
            and state.valid_frames >= 2
            and observation is not None
            and observation.detected
            and not observation.round_marker_detected
            and not observation.transverse_line_detected
            and math.isfinite(state.lateral_error_cm)
        ):
            with self._lock:
                self._final_da_visual_error_cm = state.lateral_error_cm
                self._final_da_visual_timestamp_s = state.timestamp_s
        if state.error and state.error != self._last_camera_error:
            LOG.warning(
                "camera correction degraded; radar remains authoritative: %s",
                state.error,
            )
        self._last_camera_error = state.error
        LOG.debug(
            "camera correction running=%s active=%s curve=%s recovery=%s "
            "valid_frames=%d large_frames=%d "
            "confidence=%.2f used_lateral_cm=%.2f raw_lateral_cm=%.2f "
            "heading_error_deg=%.2f curvature_per_cm=%.5f "
            "forward_heading_change_deg=%.2f correction_rad=%.4f "
            "detected=%s bands=%d rmse_cm=%.2f "
            "round=%s transverse=%s",
            state.running,
            state.active,
            state.curve_mode,
            state.recovery_mode,
            state.valid_frames,
            state.large_error_frames,
            state.confidence,
            state.lateral_error_cm,
            (
                0.0
                if state.observation is None
                else state.observation.near_lateral_error_cm
            ),
            (
                0.0
                if state.observation is None
                else math.degrees(state.observation.heading_error_rad)
            ),
            (
                0.0
                if state.observation is None
                else state.observation.curvature_per_cm
            ),
            (
                0.0
                if state.observation is None
                else math.degrees(
                    state.observation.forward_heading_change_rad
                )
            ),
            state.correction_rad,
            (
                False
                if state.observation is None
                else state.observation.detected
            ),
            (
                0
                if state.observation is None
                else state.observation.visible_band_count
            ),
            (
                0.0
                if state.observation is None
                else state.observation.fit_rmse_cm
            ),
            (
                False
                if state.observation is None
                else state.observation.round_marker_detected
            ),
            (
                False
                if state.observation is None
                else state.observation.transverse_line_detected
            ),
        )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._ready = False
            self._map_ready = False
            self._calibrating = False
        self._stop_event.set()
        with self._completion_alarm_lock:
            completion_thread = self._completion_alarm_thread
        if (
            completion_thread is not None
            and completion_thread is not threading.current_thread()
        ):
            completion_thread.join(
                timeout=self.config.completion_alarm_seconds + 0.5
            )
        with self._completion_alarm_lock:
            completion_alarm = self._completion_alarm_device
        if completion_alarm is not None:
            try:
                completion_alarm.off()
            except Exception:
                LOG.exception(
                    "failed to silence completion alarm during close"
                )
        if self.fleet_node is not None:
            self.fleet_node.close()
        if self.fleet_link is not None:
            self.fleet_link.close()
        if self._alarm is not None:
            try:
                self._alarm.off()
            except AlarmGPIOError as exc:
                LOG.warning("could not silence car alarm during shutdown: %s", exc)
        self.camera_corrector.close()
        self.follower.stop_mission()
        self.radar.set_motion_hint(False)
        self.radar.close()
        self.drive.close()
        LOG.info("application closed; hardware outputs are safe")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=None,
        help="path to the vehicle TOML profile (default: CAR_CONFIG env or "
        "configs/cooper_rock5a_l150.toml)",
    )
    parser.add_argument("--radar-port", default=None)
    parser.add_argument("--radar-x-cm", type=float, default=None)
    parser.add_argument("--radar-y-cm", type=float, default=None)
    parser.add_argument("--radar-yaw-cw-deg", type=float, default=None)
    parser.add_argument("--startup-scans", type=int, default=None)
    parser.add_argument("--calibration-timeout", type=float, default=None)
    parser.add_argument(
        "--radar-center-behind-a-cm",
        type=float,
        default=None,
        help="rear-axle distance ahead of A before the lap; overrides the "
        "profile and the runtime state file",
    )
    parser.add_argument("--ab-speed-cm-s", type=float, default=None)
    parser.add_argument("--bc-speed-cm-s", type=float, default=None)
    parser.add_argument("--cd-speed-cm-s", type=float, default=None)
    parser.add_argument("--cd-second-speed-cm-s", type=float, default=None)
    parser.add_argument("--da-speed-cm-s", type=float, default=None)
    parser.add_argument("--camera", type=_camera_source, default=None)
    parser.add_argument(
        "--no-camera-correction",
        action="store_true",
        help="run in radar-only mode (camera correction disabled)",
    )
    parser.add_argument("--fleet-link-port", default=None)
    parser.add_argument(
        "--no-fleet-position",
        action="store_true",
        help="disable read-only FleetBus relative-position reports",
    )
    parser.add_argument(
        "--fleet-position-only",
        action="store_true",
        help="calibrate and report pose without opening drive/camera outputs",
    )
    parser.add_argument(
        "--wait-for-fleet-start",
        action="store_true",
        default=None,
        help="remain stationary after calibration until CAR_START_MISSION",
    )
    parser.add_argument(
        "--fleet-mission-request-state",
        type=int,
        default=None,
        help="operation-state value reported while waiting for mission start",
    )
    parser.add_argument(
        "--completion-alarm-seconds",
        type=float,
        default=None,
        help="sound/light duration after a completed lap; zero disables it",
    )
    parser.add_argument(
        "--log-level",
        choices=("OFF", "DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    parser.add_argument("--log-dir", default=None)
    return parser


def build_main_config(
    car_config,
    mission: str | None = "task1",
    cli_args=None,
    config_path: str | None = None,
):
    """Compose the validated application config from the TOML profile + CLI.

    Explicit CLI values always win over the profile; the runtime radar-centre
    selection wins over the TOML default but loses to an explicit CLI value.
    ``mission`` selects the [missions.task1] / [missions.task2] section;
    ``None`` keeps the direct-entry behaviour (no FleetBus wait, no mission
    request state, no completion alarm).
    """
    from config.factory import (
        build_camera_correction_config,
        build_line_vision_config,
        build_steering_calibration,
        derive_steering_clamp_rad,
    )
    from config.runtime_state import load_runtime_radar_center_cm
    from config.loader import resolve_config_path as _resolve_path

    missions = car_config.missions
    common = missions.common
    devices = car_config.devices
    if mission == "task2":
        task = missions.task2
        cd_second = task.cd_speed_after_retakeoff_cm_s
        ab = task.ab_speed_cm_s
        bc = task.bc_speed_cm_s
        cd = task.cd_speed_before_retakeoff_cm_s
        da = task.da_speed_cm_s
    else:
        task = missions.task1
        cd_second = None
        ab = task.ab_speed_cm_s
        bc = task.bc_speed_cm_s
        cd = task.cd_speed_cm_s
        da = task.da_speed_cm_s

    if mission in ("task1", "task2"):
        wait_default = True
        alarm_default = task.completion_alarm_seconds
        state_default = task.fleet_mission_request_state
    else:
        # Direct entry keeps its historical behaviour: no FleetBus wait,
        # no mission request state, no completion alarm.
        wait_default = False
        alarm_default = 0.0
        state_default = None

    mount = car_config.sensors.radar.mount
    radar_center_cm = common.radar_center_behind_a_cm
    if cli_args is not None and cli_args.radar_center_behind_a_cm is not None:
        radar_center_cm = float(cli_args.radar_center_behind_a_cm)
    elif car_config.runtime.enabled:
        resolved = _resolve_path(config_path)
        base_directory = (
            None
            if Path(car_config.runtime.state_file).is_absolute()
            else resolved.resolve().parent.parent
        )
        radar_center_cm = load_runtime_radar_center_cm(
            car_config.runtime,
            radar_center_cm,
            base_directory=base_directory,
        )

    clamp_min, clamp_max = derive_steering_clamp_rad(car_config)
    return MainConfig(
        radar_port=(
            devices.radar.port
            if cli_args is None or cli_args.radar_port is None
            else cli_args.radar_port
        ),
        radar_mount=RadarMount(
            mount.x_forward_cm
            if cli_args is None or cli_args.radar_x_cm is None
            else cli_args.radar_x_cm,
            mount.y_left_cm
            if cli_args is None or cli_args.radar_y_cm is None
            else cli_args.radar_y_cm,
            mount.yaw_cw_deg
            if cli_args is None or cli_args.radar_yaw_cw_deg is None
            else cli_args.radar_yaw_cw_deg,
        ),
        startup_scan_count=(
            common.startup_scan_count
            if cli_args is None or cli_args.startup_scans is None
            else cli_args.startup_scans
        ),
        calibration_timeout_s=(
            common.calibration_timeout_s
            if cli_args is None or cli_args.calibration_timeout is None
            else cli_args.calibration_timeout
        ),
        radar_center_behind_a_cm=radar_center_cm,
        ab_speed_cm_s=ab if cli_args is None or cli_args.ab_speed_cm_s is None else cli_args.ab_speed_cm_s,
        bc_speed_cm_s=bc if cli_args is None or cli_args.bc_speed_cm_s is None else cli_args.bc_speed_cm_s,
        cd_speed_cm_s=cd if cli_args is None or cli_args.cd_speed_cm_s is None else cli_args.cd_speed_cm_s,
        cd_second_speed_cm_s=(
            cd_second
            if cli_args is None or cli_args.cd_second_speed_cm_s is None
            else cli_args.cd_second_speed_cm_s
        ),
        da_speed_cm_s=da if cli_args is None or cli_args.da_speed_cm_s is None else cli_args.da_speed_cm_s,
        camera_source=(
            devices.camera.source
            if cli_args is None or cli_args.camera is None
            else cli_args.camera
        ),
        camera_correction_enabled=(
            common.camera_correction_enabled
            and not (cli_args is not None and cli_args.no_camera_correction)
        ),
        camera_correction=build_camera_correction_config(car_config),
        vision_config=build_line_vision_config(car_config),
        fleet_position_reporting_enabled=(
            common.fleet_position_reporting_enabled
            and not (cli_args is not None and cli_args.no_fleet_position)
        ),
        fleet_link_port=(
            devices.hc14.port
            if cli_args is None or cli_args.fleet_link_port is None
            else cli_args.fleet_link_port
        ),
        fleet_position_only=(
            False if cli_args is None else cli_args.fleet_position_only
        ),
        fleet_wait_for_start=(
            wait_default
            if cli_args is None or cli_args.wait_for_fleet_start is None
            else cli_args.wait_for_fleet_start
        ),
        fleet_mission_request_state=(
            state_default
            if cli_args is None or cli_args.fleet_mission_request_state is None
            else cli_args.fleet_mission_request_state
        ),
        completion_alarm_seconds=(
            alarm_default
            if cli_args is None or cli_args.completion_alarm_seconds is None
            else cli_args.completion_alarm_seconds
        ),
        mission_control=missions.control,
        vehicle_steering_min_rad=clamp_min,
        vehicle_steering_max_rad=clamp_max,
        fleet_terminal_report_grace_s=(
            missions.control.fleet_terminal_report_grace_s
        ),
        fleet_trace_drain_timeout_s=(
            missions.control.fleet_trace_drain_timeout_s
        ),
        motor_device=devices.motor.port,
        wheelbase_mm=car_config.vehicle.geometry.wheelbase_mm,
        physical_track_width_mm=(
            car_config.vehicle.geometry.physical_track_width_mm
        ),
        firmware_track_width_mm=car_config.vehicle.drive.firmware_track_width_mm,
        min_turn_radius_mm=car_config.vehicle.drive.min_turn_radius_mm,
        allow_in_place_rotation=(
            car_config.vehicle.drive.allow_in_place_rotation
        ),
        steering_calibration=build_steering_calibration(car_config),
        hardware_lock_path=DEFAULT_HARDWARE_LOCK_PATH,
        alarm_sysfs_root=car_config.hardware.alarm_gpio.sysfs_root,
        alarm_bank_label=car_config.hardware.alarm_gpio.bank_label,
        alarm_line_offset=car_config.hardware.alarm_gpio.line_offset,
        alarm_active_low=car_config.hardware.alarm_gpio.active_low,
    )


def run_mission(
    mission: str | None = None,
    argv: list[str] | None = None,
) -> int:
    """Composition root: parse CLI, load the profile, run the application."""
    args = build_argument_parser().parse_args(argv)
    requested_log_dir = (
        default_log_dir() if args.log_dir is None else Path(args.log_dir)
    )
    try:
        configure_logging(requested_log_dir, args.log_level)
    except OSError as exc:
        print(
            f"cannot create detailed log in {requested_log_dir}: {exc}",
            file=sys.stderr,
        )
        return 2

    app: RadarCameraLineApplication | None = None
    try:
        car_config = load_car_config(args.config)
        main_config = build_main_config(
            car_config,
            mission=mission,
            cli_args=args,
            config_path=args.config,
        )

        def stop_handler(signum, _frame) -> None:
            LOG.info("received signal %s; stopping", signum)
            app.request_stop()

        app = RadarCameraLineApplication(main_config)
        signal.signal(signal.SIGINT, stop_handler)
        signal.signal(signal.SIGTERM, stop_handler)
        app.run()
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception:
        LOG.exception("radar+camera line main failed")
        return 1
    finally:
        if app is not None:
            app.close()
        shutdown_logging()


def main(argv: list[str] | None = None) -> int:
    return run_mission(mission=None, argv=argv)


if __name__ == "__main__":
    raise SystemExit(main())
