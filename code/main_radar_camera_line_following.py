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
import sys
import threading
import time

from components import (
    AckermannDrive,
    CompetitionTrack,
    CompetitionTrackFollower,
    CompetitionTrackSpeedProfile,
    DEFAULT_D500_PORT,
    DEFAULT_HC14_PORT,
    D500RadarComponent,
    LineVisionConfig,
    PerspectiveConfig,
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
    normalize_heading_deg,
    radar_yaw_to_navigation_heading,
    signed_heading_error_deg,
)


# Fixed-track segment speeds. Change these four values for the next real-car
# run; the stable steering/camera parameters below are not changed.
AB_TRACK_SPEED_CM_S = 30.0
BC_TRACK_SPEED_CM_S = 30.0
CD_TRACK_SPEED_CM_S = 30.0
DA_TRACK_SPEED_CM_S = 30.0

# FleetBus position reports are replies to the read-only ground-station POLL.
# Coordinates are centimetres relative to this run's radar-rebased start pose.
FLEET_POSITION_REPORTING_ENABLED = True
FLEET_POSITION_STALE_TIMEOUT_S = 0.5

# The rules place the physical front of the car on A.  With the measured
# 23.0 cm body and the rear axle 7.125 cm behind its centre, the rear axle is
# 18.625 cm behind that front reference.  After one geometric lap, continue by
# exactly this distance so the same physical front/rear-axle relationship is
# restored at A.
RADAR_CENTER_BEHIND_A_ALONG_AB_CM = 18.625

# Camera correction stays filtered and gated, but must be strong enough to
# overcome the repeatable radar bias once the line error is already large.
CAMERA_CORRECTION_ENABLED = True
CAMERA_LATERAL_DEADBAND_CM = 10.0
CAMERA_STEERING_GAIN_RAD_PER_CM = 0.010
CAMERA_MAX_STEERING_CORRECTION_RAD = 0.140

# Strong camera-heading alignment for an imperfect initial pose at A.  The
# start line/marker is explicitly rejected; once the longitudinal AB line is
# reliable, use its fitted heading to square the car before the first curve.
AB_START_ALIGNMENT_FULL_END_PROGRESS_CM = 90.0
AB_START_ALIGNMENT_FADE_END_PROGRESS_CM = 135.0
AB_START_HEADING_GAIN = 1.30
AB_START_MAX_HEADING_CORRECTION_RAD = 0.180
AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD = 0.220
AB_START_MIN_VALID_FRAMES = 2

# A small placement yaw error makes the radar's startup frame diverge from the
# painted AB direction.  During the first AB only, reliable straight-line
# vision estimates a bounded, slowly filtered static transform from that raw
# radar frame to the competition-track frame.  The transform then remains
# fixed for the rest of the lap: radar still supplies all motion and validity,
# while vision only calibrates the frame in which Pure Pursuit sees that pose.
AB_FRAME_LEARNING_END_PROGRESS_CM = 135.0
AB_FRAME_MIN_VALID_FRAMES = 3
AB_FRAME_HEADING_ADAPTATION_GAIN = 0.25
AB_FRAME_LATERAL_ADAPTATION_GAIN = 0.20
AB_FRAME_MAX_HEADING_OFFSET_DEG = 10.0
AB_FRAME_MAX_LATERAL_OFFSET_CM = 15.0
AB_FRAME_MAX_VISUAL_HEADING_DEG = 12.0
AB_FRAME_MAX_VISUAL_LATERAL_CM = 25.0

# The 60-degree camera mount makes the entrance of the first right semicircle
# look like a persistent right-side line offset.  Keep that ambiguous
# same-direction correction weak instead of adding it fully to radar steering.
BC_ENTRY_LIMIT_END_PROGRESS_CM = 210.0
BC_ENTRY_MIN_RIGHT_CORRECTION_RAD = -0.012

# Ground-station trajectories show a repeatable inside cut while approaching C
# and a short left-side offset after entering CD.  Apply only a small, smooth
# positive correction floor around that fixed part of the course.
C_VISIBLE_TRIM_START_PROGRESS_CM = 300.0
C_VISIBLE_TRIM_FULL_PROGRESS_CM = 330.0
C_VISIBLE_TRIM_FADE_START_PROGRESS_CM = 390.0
C_VISIBLE_TRIM_END_PROGRESS_CM = 430.0
C_VISIBLE_MIN_LEFT_CORRECTION_RAD = 0.035

# Fixed-course trim only where the ground-station trajectory shows the DA
# semicircle cutting inside the painted line.  Introduce a small outward trim
# after leaving D, hold it through the visibly displaced middle of the arc,
# then blend into the known-good stronger correction near A.
DA_VISIBLE_TRIM_START_PROGRESS_CM = 560.0
DA_VISIBLE_TRIM_FULL_PROGRESS_CM = 590.0
DA_VISIBLE_MIN_LEFT_CORRECTION_RAD = 0.045
# The camera already exceeds the floor over much of DA, so a floor alone cannot
# move the path farther outward.  Add this small course-specific amount on top,
# fading it out as the stronger final-A floor takes over.
DA_VISIBLE_EXTRA_LEFT_CORRECTION_RAD = 0.030
FINAL_DA_TRIM_START_PROGRESS_CM = 725.0
FINAL_DA_TRIM_FULL_PROGRESS_CM = 740.0
FINAL_DA_MIN_LEFT_CORRECTION_RAD = 0.100
# Near A the fixed trim is feed-forward, not a replacement for visual
# feedback.  Add a separately bounded residual from the last trustworthy
# longitudinal-line observation and hold it briefly while the A marker hides
# the line.  This distinguishes the accurate ~2 cm case from the observed
# 10-15 cm radar/camera disagreement without weakening the marker rejection.
FINAL_DA_VISUAL_DEADBAND_CM = 3.0
FINAL_DA_VISUAL_GAIN_RAD_PER_CM = 0.005
FINAL_DA_VISUAL_MAX_RESIDUAL_RAD = 0.040
FINAL_DA_VISUAL_HOLD_S = 0.65
FINAL_DA_MAX_TOTAL_LEFT_CORRECTION_RAD = 0.170
FINAL_A_MAX_CAMERA_ERROR_CM = 6.0
CAR_OPERATION_LOCALIZATION_LOST = 10
FLEET_TERMINAL_REPORT_GRACE_S = 3.0


LOG = logging.getLogger("radar-camera-line-main")
LOG_FILENAME = "car-main.log"
LOG_MAX_BYTES = 20 * 1024 * 1024
LOG_BACKUP_COUNT = 10
_LOG_LISTENER: QueueListener | None = None
MIN_VEHICLE_STEERING_RAD = -0.32
MAX_VEHICLE_STEERING_RAD = 0.336


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


def _default_correction_config() -> CameraLineCorrectionConfig:
    return CameraLineCorrectionConfig(
        lateral_deadband_cm=CAMERA_LATERAL_DEADBAND_CM,
        steering_gain_rad_per_cm=CAMERA_STEERING_GAIN_RAD_PER_CM,
        maximum_abs_correction_rad=CAMERA_MAX_STEERING_CORRECTION_RAD,
    )


@dataclass(frozen=True, slots=True)
class MainConfig:
    radar_port: str = DEFAULT_D500_PORT
    radar_mount: RadarMount = RadarMount()
    startup_scan_count: int = 3
    calibration_timeout_s: float = 30.0
    radar_center_behind_a_cm: float = (
        RADAR_CENTER_BEHIND_A_ALONG_AB_CM
    )
    ab_speed_cm_s: float = AB_TRACK_SPEED_CM_S
    bc_speed_cm_s: float = BC_TRACK_SPEED_CM_S
    cd_speed_cm_s: float = CD_TRACK_SPEED_CM_S
    da_speed_cm_s: float = DA_TRACK_SPEED_CM_S
    camera_source: int | str = 0
    camera_correction_enabled: bool = CAMERA_CORRECTION_ENABLED
    camera_correction: CameraLineCorrectionConfig = (
        CameraLineCorrectionConfig(
            lateral_deadband_cm=CAMERA_LATERAL_DEADBAND_CM,
            steering_gain_rad_per_cm=CAMERA_STEERING_GAIN_RAD_PER_CM,
            maximum_abs_correction_rad=(
                CAMERA_MAX_STEERING_CORRECTION_RAD
            ),
        )
    )
    fleet_position_reporting_enabled: bool = (
        FLEET_POSITION_REPORTING_ENABLED
    )
    fleet_link_port: str = DEFAULT_HC14_PORT
    fleet_position_only: bool = False
    fleet_wait_for_start: bool = False
    fleet_mission_request_state: int | None = None

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
    """Radar authority with bounded camera steering and frame correction."""

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
        self._final_da_visual_residual_rad = 0.0
        self._final_da_visual_timestamp_s: float | None = None
        self._terminal_camera_disagreement = False
        self._ab_frame_heading_offset_deg = 0.0
        self._ab_frame_lateral_offset_cm = 0.0
        self._ab_frame_learning_samples = 0
        self._ab_frame_last_camera_timestamp_s: float | None = None
        self._started_at = time.monotonic()
        self._calibrating = False
        self._alarm = None

        max_wheel_speed_mm_s = max(
            300.0,
            config.speed_profile.max_speed_cm_s * 12.0,
        )
        self.drive = AckermannDrive(
            max_wheel_speed_mm_s=max_wheel_speed_mm_s,
        )
        self.track = CompetitionTrack.build(
            reference_offset_cm=config.radar_center_behind_a_cm,
            finish_extension_cm=config.radar_center_behind_a_cm,
        )
        self._one_lap_progress_cm = self.track.point_at_index(
            self.track.wrap_start_index
        ).progress_cm
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

    @staticmethod
    def _front_camera_vision_config() -> LineVisionConfig:
        """Calibration for the current low-mounted, steep front camera."""

        return LineVisionConfig(
            perspective=PerspectiveConfig(
                source_points_norm=(
                    (0.02, 0.66),
                    (0.93, 0.66),
                    (0.68, 0.02),
                    (0.23, 0.02),
                ),
                output_width_px=320,
                output_height_px=400,
                ground_width_cm=80.0,
                ground_depth_cm=100.0,
            ),
            require_adaptive_confirmation=False,
            scan_near_cm=12.0,
            scan_far_cm=72.0,
            minimum_band_fill_ratio=0.20,
            use_expected_width_window=True,
            expected_line_width_cm=28.0,
            minimum_line_width_cm=10.0,
            maximum_line_width_cm=40.0,
            maximum_line_internal_gap_cm=8.0,
            maximum_center_jump_cm=18.0,
            morphology_close_size=9,
            polynomial_smoothing_alpha=0.32,
            transverse_stop_max_height_cm=8.0,
            round_marker_min_height_cm=12.0,
            continuity_weight=0.12,
        )

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
                "radar_center_behind_a_cm=%.1f "
                "post_lap_extension_cm=%.1f",
                self.config.ab_speed_cm_s,
                self.config.bc_speed_cm_s,
                self.config.cd_speed_cm_s,
                self.config.da_speed_cm_s,
                self.config.radar_center_behind_a_cm,
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
                    FLEET_TERMINAL_REPORT_GRACE_S,
                )
                self._stop_event.wait(FLEET_TERMINAL_REPORT_GRACE_S)
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

    def _fleet_set_alarm(self, active: bool) -> FleetCommandResult:
        try:
            if self._alarm is None:
                self._alarm = SoundLightAlarm()
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
                f"D500 UART {self.config.radar_port} could not be opened; "
                "verify UART6-M1, Pin 21 RX wiring and dialout permission"
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
            f"D500 UART {self.config.radar_port} is open but no complete "
            "scan arrived"
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
        control_pose: NavigationPose | None = None
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
                radar_pose = NavigationPose(
                    x_cm=update.global_pose.x_cm,
                    y_cm=update.global_pose.y_cm,
                    heading_deg=radar_yaw_to_navigation_heading(
                        update.global_pose.yaw_cw_deg
                    ),
                    timestamp_s=time.monotonic(),
                )
                self._localization_degraded = False
        if (
            ready
            and update.global_pose is not None
            and update.odometry.accepted
        ):
            control_pose = self._ab_aligned_control_pose(radar_pose)
            with self._lock:
                # FleetBus reports the same competition-frame pose that the
                # follower uses.  The radar map/ICP pose itself is untouched.
                self._latest_navigation_pose = control_pose
        try:
            self.follower.update_from_radar(
                update,
                control_pose_override=control_pose,
            )
        except BaseException:
            LOG.exception("track update failed; stopping")
            self.request_stop()

    def _ab_aligned_control_pose(
        self,
        radar_pose: NavigationPose,
        *,
        now_s: float | None = None,
    ) -> NavigationPose:
        """Apply the static AB vision calibration to a raw radar pose."""

        now = time.monotonic() if now_s is None else float(now_s)
        camera_state = self.camera_corrector.state
        observation = camera_state.observation
        with self._lock:
            follower_state = self._follower_state
            last_camera_timestamp_s = (
                self._ab_frame_last_camera_timestamp_s
            )
        learning_allowed = (
            self.config.camera_correction_enabled
            and follower_state.running
            and not follower_state.completed
            and follower_state.segment is TrackSegment.AB
            and follower_state.progress_cm
            < AB_FRAME_LEARNING_END_PROGRESS_CM
            and camera_state.active
            and camera_state.valid_frames >= AB_FRAME_MIN_VALID_FRAMES
            and (
                last_camera_timestamp_s is None
                or camera_state.timestamp_s > last_camera_timestamp_s
            )
            and now - camera_state.timestamp_s
            <= self.config.camera_correction.stale_timeout_s
            and observation is not None
            and observation.detected
            and observation.confidence
            >= self.config.camera_correction.minimum_confidence
            and observation.visible_band_count
            >= self.config.camera_correction.minimum_visible_bands
            and observation.fit_rmse_cm
            <= self.config.camera_correction.maximum_fit_rmse_cm
            and not observation.round_marker_detected
            and not observation.transverse_line_detected
            and math.isfinite(observation.heading_error_rad)
            and all(
                math.isfinite(value)
                for value in observation.polynomial_y_left_by_x
            )
        )
        if learning_allowed:
            visual_heading_deg = -math.degrees(
                observation.heading_error_rad
            )
            visual_lateral_cm = -float(
                observation.polynomial_y_left_by_x[2]
            )
            if (
                abs(visual_heading_deg)
                <= AB_FRAME_MAX_VISUAL_HEADING_DEG
                and abs(visual_lateral_cm)
                <= AB_FRAME_MAX_VISUAL_LATERAL_CM
            ):
                measured_heading_offset_deg = max(
                    -AB_FRAME_MAX_HEADING_OFFSET_DEG,
                    min(
                        AB_FRAME_MAX_HEADING_OFFSET_DEG,
                        signed_heading_error_deg(
                            visual_heading_deg,
                            radar_pose.heading_deg,
                        ),
                    ),
                )
                with self._lock:
                    previous_heading_offset_deg = (
                        self._ab_frame_heading_offset_deg
                    )
                heading_step_deg = signed_heading_error_deg(
                    measured_heading_offset_deg,
                    previous_heading_offset_deg,
                )
                heading_offset_deg = max(
                    -AB_FRAME_MAX_HEADING_OFFSET_DEG,
                    min(
                        AB_FRAME_MAX_HEADING_OFFSET_DEG,
                        previous_heading_offset_deg
                        + AB_FRAME_HEADING_ADAPTATION_GAIN
                        * heading_step_deg,
                    ),
                )
                heading_offset_rad = math.radians(heading_offset_deg)
                rotated_y_cm = (
                    math.sin(heading_offset_rad) * radar_pose.x_cm
                    + math.cos(heading_offset_rad) * radar_pose.y_cm
                )
                measured_lateral_offset_cm = max(
                    -AB_FRAME_MAX_LATERAL_OFFSET_CM,
                    min(
                        AB_FRAME_MAX_LATERAL_OFFSET_CM,
                        visual_lateral_cm - rotated_y_cm,
                    ),
                )
                with self._lock:
                    self._ab_frame_heading_offset_deg = heading_offset_deg
                    self._ab_frame_lateral_offset_cm += (
                        AB_FRAME_LATERAL_ADAPTATION_GAIN
                        * (
                            measured_lateral_offset_cm
                            - self._ab_frame_lateral_offset_cm
                        )
                    )
                    self._ab_frame_lateral_offset_cm = max(
                        -AB_FRAME_MAX_LATERAL_OFFSET_CM,
                        min(
                            AB_FRAME_MAX_LATERAL_OFFSET_CM,
                            self._ab_frame_lateral_offset_cm,
                        ),
                    )
                    self._ab_frame_learning_samples += 1
                    self._ab_frame_last_camera_timestamp_s = (
                        camera_state.timestamp_s
                    )

        with self._lock:
            heading_offset_deg = self._ab_frame_heading_offset_deg
            lateral_offset_cm = self._ab_frame_lateral_offset_cm
            learning_samples = self._ab_frame_learning_samples
        if learning_samples <= 0:
            return radar_pose
        heading_offset_rad = math.radians(heading_offset_deg)
        cos_offset = math.cos(heading_offset_rad)
        sin_offset = math.sin(heading_offset_rad)
        fused_pose = NavigationPose(
            x_cm=(
                cos_offset * radar_pose.x_cm
                - sin_offset * radar_pose.y_cm
            ),
            y_cm=(
                sin_offset * radar_pose.x_cm
                + cos_offset * radar_pose.y_cm
                + lateral_offset_cm
            ),
            heading_deg=normalize_heading_deg(
                radar_pose.heading_deg + heading_offset_deg
            ),
            timestamp_s=radar_pose.timestamp_s,
        )
        LOG.debug(
            "AB frame fusion active=%s samples=%d "
            "radar_x_cm=%.2f radar_y_cm=%.2f radar_heading_deg=%.2f "
            "heading_offset_deg=%.2f lateral_offset_cm=%.2f "
            "fused_x_cm=%.2f fused_y_cm=%.2f fused_heading_deg=%.2f",
            learning_allowed,
            learning_samples,
            radar_pose.x_cm,
            radar_pose.y_cm,
            radar_pose.heading_deg,
            heading_offset_deg,
            lateral_offset_cm,
            fused_pose.x_cm,
            fused_pose.y_cm,
            fused_pose.heading_deg,
        )
        return fused_pose

    def _on_follower_state(self, state: TrackFollowerState) -> None:
        self.camera_corrector.set_curve_mode(
            state.segment in (TrackSegment.BC, TrackSegment.DA)
        )
        with self._lock:
            self._follower_state = state
        if state.completed:
            now_s = time.monotonic()
            with self._lock:
                visual_error_cm = self._final_da_visual_error_cm
                visual_timestamp_s = self._final_da_visual_timestamp_s
                disagreement = (
                    visual_error_cm is not None
                    and visual_timestamp_s is not None
                    and now_s - visual_timestamp_s
                    <= FINAL_DA_VISUAL_HOLD_S
                    and abs(visual_error_cm)
                    > FINAL_A_MAX_CAMERA_ERROR_CM
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
                    "one lap plus %.1f cm complete; rear axle stopped at A",
                    self.config.radar_center_behind_a_cm,
                )

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
            now_s = time.monotonic()
            observed_correction = self.camera_corrector.correction_for_speed(
                speed_cm_s,
                now_s=now_s,
            )
            ab_start_alignment = self._ab_start_alignment_correction(
                now_s=now_s
            )
            camera_correction = max(
                -AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD,
                min(
                    AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD,
                    observed_correction + ab_start_alignment,
                ),
            )
            course_limited_correction = self._apply_course_camera_limit(
                camera_correction
            )
            c_point_trim = self._c_point_trim()
            position_limited_correction = (
                course_limited_correction
                if c_point_trim is None
                else max(course_limited_correction, c_point_trim)
            )
            final_da_trim = self._final_da_trim()
            da_visible_extra = self._da_visible_extra_trim()
            final_da_visual_residual = (
                self._final_da_visual_residual(now_s=now_s)
            )
            with self._lock:
                follower_state = self._follower_state
            terminal_da_active = (
                final_da_trim is not None
                and follower_state.running
                and not follower_state.completed
                and (
                    (
                        follower_state.segment is TrackSegment.DA
                        and follower_state.progress_cm
                        >= FINAL_DA_TRIM_START_PROGRESS_CM
                    )
                    or self._on_post_lap_extension(follower_state)
                )
            )
            if terminal_da_active:
                correction = max(
                    position_limited_correction,
                    final_da_trim + final_da_visual_residual,
                )
                if da_visible_extra is not None:
                    correction += da_visible_extra
                correction = max(
                    -FINAL_DA_MAX_TOTAL_LEFT_CORRECTION_RAD,
                    min(
                        FINAL_DA_MAX_TOTAL_LEFT_CORRECTION_RAD,
                        correction,
                    ),
                )
            else:
                correction = (
                    position_limited_correction
                    if final_da_trim is None
                    else max(position_limited_correction, final_da_trim)
                )
                if da_visible_extra is not None:
                    correction += da_visible_extra
            combined = float(radar_steering_rad) + correction
            adjusted = max(
                MIN_VEHICLE_STEERING_RAD,
                min(MAX_VEHICLE_STEERING_RAD, combined),
            )
            state = self.camera_corrector.state
            LOG.debug(
                "steering fusion radar_rad=%.4f camera_rad=%.4f "
                "final_rad=%.4f observed_camera_rad=%.4f "
                "ab_start_alignment_rad=%.4f "
                "course_limited_camera_rad=%.4f c_point_trim_rad=%.4f "
                "final_da_trim_rad=%.4f da_visible_extra_rad=%.4f "
                "final_da_visual_residual_rad=%.4f "
                "camera_active=%s "
                "camera_error_cm=%.2f camera_confidence=%.2f",
                radar_steering_rad,
                correction,
                adjusted,
                observed_correction,
                ab_start_alignment,
                course_limited_correction,
                0.0 if c_point_trim is None else c_point_trim,
                0.0 if final_da_trim is None else final_da_trim,
                (
                    0.0
                    if da_visible_extra is None
                    else da_visible_extra
                ),
                final_da_visual_residual,
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
            >= AB_START_ALIGNMENT_FADE_END_PROGRESS_CM
            or now - camera_state.timestamp_s
            > self.config.camera_correction.stale_timeout_s
            or camera_state.valid_frames < AB_START_MIN_VALID_FRAMES
            or observation is None
            or not observation.detected
            or not math.isfinite(observation.heading_error_rad)
            or observation.confidence
            < self.config.camera_correction.minimum_confidence
            or observation.visible_band_count
            < self.config.camera_correction.minimum_visible_bands
            or observation.fit_rmse_cm
            > self.config.camera_correction.maximum_fit_rmse_cm
            or observation.round_marker_detected
            or observation.transverse_line_detected
        ):
            return 0.0

        fade_span_cm = (
            AB_START_ALIGNMENT_FADE_END_PROGRESS_CM
            - AB_START_ALIGNMENT_FULL_END_PROGRESS_CM
        )
        fade_scale = (
            1.0
            if follower_state.progress_cm
            <= AB_START_ALIGNMENT_FULL_END_PROGRESS_CM
            else max(
                0.0,
                (
                    AB_START_ALIGNMENT_FADE_END_PROGRESS_CM
                    - follower_state.progress_cm
                )
                / fade_span_cm,
            )
        )
        requested = (
            AB_START_HEADING_GAIN
            * float(observation.heading_error_rad)
        )
        bounded = max(
            -AB_START_MAX_HEADING_CORRECTION_RAD,
            min(AB_START_MAX_HEADING_CORRECTION_RAD, requested),
        )
        return bounded * fade_scale

    def _apply_course_camera_limit(self, correction_rad: float) -> float:
        with self._lock:
            state = self._follower_state
        if (
            state.running
            and not state.completed
            and state.segment is TrackSegment.BC
            and state.progress_cm < BC_ENTRY_LIMIT_END_PROGRESS_CM
        ):
            return max(
                float(correction_rad),
                BC_ENTRY_MIN_RIGHT_CORRECTION_RAD,
            )
        return float(correction_rad)

    def _c_point_trim(self) -> float | None:
        with self._lock:
            state = self._follower_state
        if not state.running or state.completed:
            return None
        if state.segment is TrackSegment.BC:
            if state.progress_cm < C_VISIBLE_TRIM_START_PROGRESS_CM:
                return None
            if state.progress_cm >= C_VISIBLE_TRIM_FULL_PROGRESS_CM:
                return C_VISIBLE_MIN_LEFT_CORRECTION_RAD
            blend = (
                state.progress_cm - C_VISIBLE_TRIM_START_PROGRESS_CM
            ) / (
                C_VISIBLE_TRIM_FULL_PROGRESS_CM
                - C_VISIBLE_TRIM_START_PROGRESS_CM
            )
            return C_VISIBLE_MIN_LEFT_CORRECTION_RAD * blend
        if state.segment is not TrackSegment.CD:
            return None
        if state.progress_cm >= C_VISIBLE_TRIM_END_PROGRESS_CM:
            return None
        if state.progress_cm <= C_VISIBLE_TRIM_FADE_START_PROGRESS_CM:
            return C_VISIBLE_MIN_LEFT_CORRECTION_RAD
        blend = (
            C_VISIBLE_TRIM_END_PROGRESS_CM - state.progress_cm
        ) / (
            C_VISIBLE_TRIM_END_PROGRESS_CM
            - C_VISIBLE_TRIM_FADE_START_PROGRESS_CM
        )
        return C_VISIBLE_MIN_LEFT_CORRECTION_RAD * blend

    def _final_da_trim(self) -> float | None:
        with self._lock:
            state = self._follower_state
        if self._on_post_lap_extension(state):
            return FINAL_DA_MIN_LEFT_CORRECTION_RAD
        if (
            not state.running
            or state.completed
            or state.segment is not TrackSegment.DA
            or state.progress_cm < DA_VISIBLE_TRIM_START_PROGRESS_CM
        ):
            return None
        if state.progress_cm < DA_VISIBLE_TRIM_FULL_PROGRESS_CM:
            visible_span_cm = (
                DA_VISIBLE_TRIM_FULL_PROGRESS_CM
                - DA_VISIBLE_TRIM_START_PROGRESS_CM
            )
            visible_blend = (
                state.progress_cm - DA_VISIBLE_TRIM_START_PROGRESS_CM
            ) / visible_span_cm
            return DA_VISIBLE_MIN_LEFT_CORRECTION_RAD * visible_blend
        if state.progress_cm < FINAL_DA_TRIM_START_PROGRESS_CM:
            return DA_VISIBLE_MIN_LEFT_CORRECTION_RAD
        final_span_cm = (
            FINAL_DA_TRIM_FULL_PROGRESS_CM
            - FINAL_DA_TRIM_START_PROGRESS_CM
        )
        final_blend = min(
            1.0,
            max(
                0.0,
                (
                    state.progress_cm
                    - FINAL_DA_TRIM_START_PROGRESS_CM
                )
                / final_span_cm,
            ),
        )
        return DA_VISIBLE_MIN_LEFT_CORRECTION_RAD + (
            FINAL_DA_MIN_LEFT_CORRECTION_RAD
            - DA_VISIBLE_MIN_LEFT_CORRECTION_RAD
        ) * final_blend

    def _da_visible_extra_trim(self) -> float | None:
        with self._lock:
            state = self._follower_state
        if (
            not state.running
            or state.completed
            or state.segment is not TrackSegment.DA
            or state.progress_cm < DA_VISIBLE_TRIM_START_PROGRESS_CM
            or state.progress_cm >= FINAL_DA_TRIM_FULL_PROGRESS_CM
        ):
            return None
        if state.progress_cm < DA_VISIBLE_TRIM_FULL_PROGRESS_CM:
            blend = (
                state.progress_cm - DA_VISIBLE_TRIM_START_PROGRESS_CM
            ) / (
                DA_VISIBLE_TRIM_FULL_PROGRESS_CM
                - DA_VISIBLE_TRIM_START_PROGRESS_CM
            )
            return DA_VISIBLE_EXTRA_LEFT_CORRECTION_RAD * blend
        if state.progress_cm <= FINAL_DA_TRIM_START_PROGRESS_CM:
            return DA_VISIBLE_EXTRA_LEFT_CORRECTION_RAD
        fade = (
            FINAL_DA_TRIM_FULL_PROGRESS_CM - state.progress_cm
        ) / (
            FINAL_DA_TRIM_FULL_PROGRESS_CM
            - FINAL_DA_TRIM_START_PROGRESS_CM
        )
        return DA_VISIBLE_EXTRA_LEFT_CORRECTION_RAD * fade

    def _final_da_visual_residual(
        self,
        *,
        now_s: float | None = None,
    ) -> float:
        now = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            follower_state = self._follower_state
            timestamp_s = self._final_da_visual_timestamp_s
            residual_rad = self._final_da_visual_residual_rad
        if (
            not follower_state.running
            or follower_state.completed
            or not (
                (
                    follower_state.segment is TrackSegment.DA
                    and follower_state.progress_cm
                    >= FINAL_DA_TRIM_START_PROGRESS_CM
                )
                or self._on_post_lap_extension(follower_state)
            )
            or timestamp_s is None
            or now - timestamp_s > FINAL_DA_VISUAL_HOLD_S
        ):
            return 0.0
        return residual_rad

    def _on_post_lap_extension(
        self,
        state: TrackFollowerState,
    ) -> bool:
        return (
            state.running
            and not state.completed
            and state.segment is TrackSegment.AB
            and state.progress_cm >= self._one_lap_progress_cm
        )

    def _update_final_da_visual_feedback(
        self,
        state: CameraLineCorrectionState,
    ) -> None:
        observation = state.observation
        with self._lock:
            follower_state = self._follower_state
        on_post_lap_extension = self._on_post_lap_extension(follower_state)
        if (
            not follower_state.running
            or follower_state.completed
            or not (
                (
                    follower_state.segment is TrackSegment.DA
                    and follower_state.progress_cm
                    >= FINAL_DA_TRIM_START_PROGRESS_CM
                )
                or on_post_lap_extension
            )
            or not state.active
            or state.valid_frames < 2
            or observation is None
            or not observation.detected
            or observation.round_marker_detected
            or observation.transverse_line_detected
            or not math.isfinite(state.lateral_error_cm)
        ):
            return
        magnitude = max(
            0.0,
            abs(state.lateral_error_cm) - FINAL_DA_VISUAL_DEADBAND_CM,
        )
        residual_rad = math.copysign(
            min(
                FINAL_DA_VISUAL_MAX_RESIDUAL_RAD,
                FINAL_DA_VISUAL_GAIN_RAD_PER_CM * magnitude,
            ),
            state.lateral_error_cm,
        )
        with self._lock:
            self._final_da_visual_error_cm = state.lateral_error_cm
            self._final_da_visual_residual_rad = residual_rad
            self._final_da_visual_timestamp_s = state.timestamp_s

    def _on_camera_state(
        self,
        state: CameraLineCorrectionState,
    ) -> None:
        self._update_final_da_visual_feedback(state)
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
            "correction_rad=%.4f "
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
    parser.add_argument("--radar-port", default=DEFAULT_D500_PORT)
    parser.add_argument("--radar-x-cm", type=float, default=0.0)
    parser.add_argument("--radar-y-cm", type=float, default=0.0)
    parser.add_argument("--radar-yaw-cw-deg", type=float, default=0.0)
    parser.add_argument("--startup-scans", type=int, default=3)
    parser.add_argument("--calibration-timeout", type=float, default=30.0)
    parser.add_argument(
        "--radar-center-behind-a-cm",
        type=float,
        default=RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    )
    parser.add_argument("--ab-speed-cm-s", type=float, default=AB_TRACK_SPEED_CM_S)
    parser.add_argument("--bc-speed-cm-s", type=float, default=BC_TRACK_SPEED_CM_S)
    parser.add_argument("--cd-speed-cm-s", type=float, default=CD_TRACK_SPEED_CM_S)
    parser.add_argument("--da-speed-cm-s", type=float, default=DA_TRACK_SPEED_CM_S)
    parser.add_argument("--camera", type=_camera_source, default=0)
    parser.add_argument(
        "--no-camera-correction",
        action="store_true",
        help="run the copied fixed-track program in radar-only mode",
    )
    parser.add_argument("--fleet-link-port", default=DEFAULT_HC14_PORT)
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
        help="remain stationary after calibration until CAR_START_MISSION",
    )
    parser.add_argument(
        "--fleet-mission-request-state",
        type=int,
        default=None,
        help="operation-state value reported while waiting for mission start",
    )
    parser.add_argument(
        "--log-level",
        choices=("OFF", "DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    parser.add_argument("--log-dir", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
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
        app = RadarCameraLineApplication(
            MainConfig(
                radar_port=args.radar_port,
                radar_mount=RadarMount(
                    args.radar_x_cm,
                    args.radar_y_cm,
                    args.radar_yaw_cw_deg,
                ),
                startup_scan_count=args.startup_scans,
                calibration_timeout_s=args.calibration_timeout,
                radar_center_behind_a_cm=args.radar_center_behind_a_cm,
                ab_speed_cm_s=args.ab_speed_cm_s,
                bc_speed_cm_s=args.bc_speed_cm_s,
                cd_speed_cm_s=args.cd_speed_cm_s,
                da_speed_cm_s=args.da_speed_cm_s,
                camera_source=args.camera,
                camera_correction_enabled=(
                    not args.no_camera_correction
                ),
                camera_correction=_default_correction_config(),
                fleet_position_reporting_enabled=(
                    not args.no_fleet_position
                ),
                fleet_link_port=args.fleet_link_port,
                fleet_position_only=args.fleet_position_only,
                fleet_wait_for_start=args.wait_for_fleet_start,
                fleet_mission_request_state=args.fleet_mission_request_state,
            )
        )

        def stop_handler(signum, _frame) -> None:
            LOG.info("received signal %s; stopping", signum)
            app.request_stop()

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


if __name__ == "__main__":
    raise SystemExit(main())
