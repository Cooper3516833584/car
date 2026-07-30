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


# Fixed-track segment speeds. Change these four values for the next real-car
# run; the stable steering/camera parameters below are not changed.
AB_TRACK_SPEED_CM_S = 12.0
BC_TRACK_SPEED_CM_S = 15.0
CD_TRACK_SPEED_CM_S = 30.0
DA_TRACK_SPEED_CM_S = 15.0

# FleetBus position reports are replies to the read-only ground-station POLL.
# Coordinates are centimetres relative to this run's radar-rebased start pose.
FLEET_POSITION_REPORTING_ENABLED = True
FLEET_POSITION_STALE_TIMEOUT_S = 0.5

# At startup the car faces AB with its front reference at A. The radar centre
# is this far behind A; after driving forward this distance, it passes A.
RADAR_CENTER_BEHIND_A_ALONG_AB_CM = 24.0

# Camera correction stays filtered and gated, but must be strong enough to
# overcome the repeatable radar bias once the line error is already large.
CAMERA_CORRECTION_ENABLED = True
CAMERA_LATERAL_DEADBAND_CM = 10.0
CAMERA_STEERING_GAIN_RAD_PER_CM = 0.010
CAMERA_MAX_STEERING_CORRECTION_RAD = 0.140

# Strong camera-heading alignment for an imperfect initial pose at A.  The
# start line/marker is explicitly rejected; once the longitudinal AB line is
# reliable, use its fitted heading to square the car before the first curve.
AB_START_ALIGNMENT_FULL_END_PROGRESS_CM = 30.0
AB_START_ALIGNMENT_FADE_END_PROGRESS_CM = 80.0
AB_START_HEADING_GAIN = 1.30
AB_START_MAX_HEADING_CORRECTION_RAD = 0.180
AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD = 0.220
AB_START_MIN_VALID_FRAMES = 2

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
C_VISIBLE_MIN_LEFT_CORRECTION_RAD = 0.025

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
DA_VISIBLE_EXTRA_LEFT_CORRECTION_RAD = 0.018
FINAL_DA_TRIM_START_PROGRESS_CM = 725.0
FINAL_DA_TRIM_FULL_PROGRESS_CM = 740.0
FINAL_DA_MIN_LEFT_CORRECTION_RAD = 0.100


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
    """Radar authority with a bounded camera steering correction."""

    def __init__(self, config: MainConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._completed_event = threading.Event()
        self._scan_event = threading.Event()
        self._startup_scans: list[RadarScan] = []
        self._ready = False
        self._map_ready = False
        self._latest_navigation_pose = None
        self._localization_degraded = False
        self._fleet_error_code = 0
        self._closed = False
        self._last_camera_error: str | None = None
        self._started_at = time.monotonic()
        self._calibrating = False

        max_wheel_speed_mm_s = max(
            300.0,
            config.speed_profile.max_speed_cm_s * 12.0,
        )
        self.drive = AckermannDrive(
            max_wheel_speed_mm_s=max_wheel_speed_mm_s,
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
                localization_degraded=self._localization_degraded,
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
                "radar_center_behind_a_cm=%.1f",
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
            degraded = self._localization_degraded
            pose = self._latest_navigation_pose
            follower_state = self._follower_state
            error_code = self._fleet_error_code
        pose_fresh = (
            pose is not None
            and now - pose.timestamp_s
            <= FLEET_POSITION_STALE_TIMEOUT_S
        )
        pose_valid = ready and pose_fresh
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
        if follower_state.completed:
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
            pose_quality=4 if pose_valid else (2 if pose is not None else 0),
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
            "calibration complete; rear axle rebased to A=(0,0,0deg) "
            "bounds=x[%.1f,%.1f] y[%.1f,%.1f] fitted_lines=%d",
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
                self._latest_navigation_pose = NavigationPose(
                    x_cm=update.global_pose.x_cm,
                    y_cm=update.global_pose.y_cm,
                    heading_deg=radar_yaw_to_navigation_heading(
                        update.global_pose.yaw_cw_deg
                    ),
                    timestamp_s=time.monotonic(),
                )
                self._localization_degraded = False
        try:
            self.follower.update_from_radar(update)
        except BaseException:
            LOG.exception("track update failed; stopping")
            self.request_stop()

    def _on_follower_state(self, state: TrackFollowerState) -> None:
        self.camera_corrector.set_curve_mode(
            state.segment in (TrackSegment.BC, TrackSegment.DA)
        )
        with self._lock:
            self._follower_state = state
        if state.completed:
            self.radar.set_motion_hint(False)
            self._completed_event.set()
            LOG.info("one lap complete; vehicle stopped at A")

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
            correction = (
                position_limited_correction
                if final_da_trim is None
                else max(position_limited_correction, final_da_trim)
            )
            da_visible_extra = self._da_visible_extra_trim()
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

    def _on_camera_state(
        self,
        state: CameraLineCorrectionState,
    ) -> None:
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
