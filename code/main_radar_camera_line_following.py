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
    BlackLineDetector,
    CompetitionTrack,
    CompetitionTrackFollower,
    DEFAULT_D500_PORT,
    D500RadarComponent,
    LineObservation,
    LineVisionConfig,
    PerspectiveConfig,
    Pose2D,
    RadarLocalizationUpdate,
    RadarMount,
    RadarScan,
    RectangleFieldCalibrator,
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
from components.navigation import (
    NavigationPose,
    NavigationState,
    radar_yaw_to_navigation_heading,
)


# Radar fixed-track speed. Change this one value for the next real-car run.
TRACK_SPEED_CM_S = 30.0

# At startup the car faces AB with its front reference at A. The radar centre
# is this far behind A; after driving forward this distance, it passes A.
RADAR_CENTER_BEHIND_A_ALONG_AB_CM = 20.0

# Camera correction stays filtered and gated, but must be strong enough to
# overcome the repeatable radar bias once the line error is already large.
CAMERA_CORRECTION_ENABLED = True
CAMERA_LATERAL_DEADBAND_CM = 10.0
CAMERA_STEERING_GAIN_RAD_PER_CM = 0.010
CAMERA_MAX_STEERING_CORRECTION_RAD = 0.140

# The camera now looks 30 degrees below horizontal.  A second, inexpensive
# perspective profile uses the farther visible path only on the two straights.
# It contributes heading only (never lateral position), waits for a stable
# longitudinal line, and is reset before either semicircle.
STRAIGHT_FAR_MARGIN_CM = 25.0
STRAIGHT_FAR_MIN_CONFIDENCE = 0.78
STRAIGHT_FAR_MIN_VISIBLE_BANDS = 8
STRAIGHT_FAR_MAX_RMSE_CM = 1.20
STRAIGHT_FAR_MAX_HEADING_CHANGE_RAD = 0.14
STRAIGHT_FAR_REQUIRED_FRAMES = 5
STRAIGHT_FAR_MAX_FRAME_STEP_RAD = 0.035
STRAIGHT_FAR_HEADING_DEADBAND_RAD = 0.018
STRAIGHT_FAR_HEADING_GAIN = 0.90
STRAIGHT_FAR_MAX_CORRECTION_RAD = 0.055
STRAIGHT_FAR_FILTER_TIME_CONSTANT_S = 0.30
STRAIGHT_FAR_MAX_CORRECTION_RATE_RAD_S = 0.18
STRAIGHT_FAR_NEAR_PROBE_CM = 35.0
STRAIGHT_FAR_PROBE_CM = 95.0
TRACK_STRAIGHT_LENGTH_CM = 150.0
TRACK_SEMICIRCLE_LENGTH_CM = math.pi * 75.0

# Strong camera-heading alignment for an imperfect initial pose at A.  The
# start line/marker is explicitly rejected; once the longitudinal AB line is
# reliable, use its fitted heading to square the car before the first curve.
AB_START_ALIGNMENT_FULL_END_PROGRESS_CM = 30.0
AB_START_ALIGNMENT_FADE_END_PROGRESS_CM = 80.0
AB_START_HEADING_GAIN = 1.30
AB_START_MAX_HEADING_CORRECTION_RAD = 0.180
AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD = 0.220
AB_START_MIN_VALID_FRAMES = 2

# The near-field camera profile can make the entrance of the first right
# semicircle look like a persistent right-side line offset. Keep that ambiguous
# same-direction correction weak instead of adding it fully to radar steering.
BC_ENTRY_LIMIT_END_PROGRESS_CM = 210.0
BC_ENTRY_MIN_RIGHT_CORRECTION_RAD = -0.012

# Fixed-course trim for the final DA approach.  The end marker progressively
# hides the longitudinal line while radar tends to command full right lock, so
# preserve a known-good left correction over only the last part of the lap.
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
    speed_cm_s: float = TRACK_SPEED_CM_S
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

    def __post_init__(self) -> None:
        if self.startup_scan_count <= 0:
            raise ValueError("startup_scan_count must be positive")
        if self.calibration_timeout_s <= 0.0:
            raise ValueError("calibration_timeout_s must be positive")
        if self.radar_center_behind_a_cm < 0.0:
            raise ValueError("radar_center_behind_a_cm cannot be negative")
        if self.speed_cm_s <= 0.0:
            raise ValueError("speed_cm_s must be positive")
        if isinstance(self.camera_source, int) and self.camera_source < 0:
            raise ValueError("camera_source cannot be negative")
        if isinstance(self.camera_source, str) and not self.camera_source:
            raise ValueError("camera_source cannot be empty")

@dataclass(frozen=True, slots=True)
class CarRuntimeSnapshot:
    """Atomic read-only FleetBus view of the local navigation runtime."""

    ready: bool
    map_ready: bool
    pose: NavigationPose | None
    navigation_state: NavigationState
    localization_degraded: bool
    error_code: int
    localization_timeout_s: float = 0.5


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
        self._straight_far_valid_frames = 0
        self._straight_far_candidate_rad: float | None = None
        self._straight_far_last_observation_s: float | None = None
        self._straight_far_last_filter_s: float | None = None
        self._straight_far_filtered_rad = 0.0
        self._straight_far_heading_error_rad = 0.0

        max_wheel_speed_mm_s = max(300.0, config.speed_cm_s * 12.0)
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
            supplemental_detector=BlackLineDetector(
                self._straight_far_vision_config()
            ),
            supplemental_in_curve_mode=False,
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
            speed_cm_s=config.speed_cm_s,
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
        """Retain the proven near-field profile used by curve correction."""

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

    @staticmethod
    def _straight_far_vision_config() -> LineVisionConfig:
        """Far-path profile calibrated from the current 30-degree camera."""

        return LineVisionConfig(
            perspective=PerspectiveConfig(
                source_points_norm=(
                    (0.100, 0.950),
                    (0.900, 0.950),
                    (0.556, 0.100),
                    (0.413, 0.100),
                ),
                output_width_px=200,
                output_height_px=250,
                ground_width_cm=80.0,
                ground_depth_cm=120.0,
            ),
            require_adaptive_confirmation=False,
            scan_near_cm=18.0,
            scan_far_cm=105.0,
            minimum_band_fill_ratio=0.20,
            use_expected_width_window=True,
            expected_line_width_cm=20.0,
            minimum_line_width_cm=7.0,
            maximum_line_width_cm=34.0,
            maximum_line_internal_gap_cm=8.0,
            maximum_center_jump_cm=18.0,
            morphology_close_size=9,
            polynomial_smoothing_alpha=0.32,
            transverse_stop_max_forward_cm=105.0,
            transverse_stop_max_height_cm=8.0,
            round_marker_min_height_cm=12.0,
            continuity_weight=0.12,
        )

    def run(self) -> None:
        LOG.info(
            "vehicle must remain stationary at A during D500 calibration"
        )
        try:
            self._calibrate_radar()
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
            with self._lock:
                self._ready = True
            self.radar.start()
            LOG.info(
                "one-lap radar+camera tracking started speed_cm_s=%.1f "
                "radar_center_behind_a_cm=%.1f",
                self.config.speed_cm_s,
                self.config.radar_center_behind_a_cm,
            )
            while (
                not self._stop_event.is_set()
                and not self._completed_event.wait(0.5)
            ):
                pass
        finally:
            self.close()

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
            straight_far_heading = self._straight_far_heading_correction(
                now_s=now_s
            )
            camera_correction = max(
                -AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD,
                min(
                    AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD,
                    (
                        observed_correction
                        + ab_start_alignment
                        + straight_far_heading
                    ),
                ),
            )
            course_limited_correction = self._apply_course_camera_limit(
                camera_correction
            )
            final_da_trim = self._final_da_trim()
            correction = (
                course_limited_correction
                if final_da_trim is None
                else max(course_limited_correction, final_da_trim)
            )
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
                "straight_far_heading_rad=%.4f "
                "straight_far_error_deg=%.2f "
                "course_limited_camera_rad=%.4f final_da_trim_rad=%.4f "
                "camera_active=%s "
                "camera_error_cm=%.2f camera_confidence=%.2f",
                radar_steering_rad,
                correction,
                adjusted,
                observed_correction,
                ab_start_alignment,
                straight_far_heading,
                math.degrees(self._straight_far_heading_error_rad),
                course_limited_correction,
                0.0 if final_da_trim is None else final_da_trim,
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

    def _straight_far_heading_correction(
        self,
        *,
        now_s: float | None = None,
        observation: LineObservation | None = None,
    ) -> float:
        """Return a slow heading-only correction on the middle of AB/CD."""

        now = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            follower_state = self._follower_state
        if not self._straight_far_window_active(follower_state):
            self._reset_straight_far_heading()
            return 0.0

        current = (
            self.camera_corrector.supplemental_observation
            if observation is None
            else observation
        )
        if current is None or now - current.timestamp_s > (
            self.config.camera_correction.stale_timeout_s
        ):
            self._reset_straight_far_heading()
            return 0.0
        if current.timestamp_s == self._straight_far_last_observation_s:
            return self._straight_far_filtered_rad
        self._straight_far_last_observation_s = current.timestamp_s

        heading_error = self._far_path_heading_error(current)
        usable = (
            current.detected
            and math.isfinite(heading_error)
            and current.confidence >= STRAIGHT_FAR_MIN_CONFIDENCE
            and current.visible_band_count >= STRAIGHT_FAR_MIN_VISIBLE_BANDS
            and current.fit_rmse_cm <= STRAIGHT_FAR_MAX_RMSE_CM
            and abs(current.forward_heading_change_rad)
            <= STRAIGHT_FAR_MAX_HEADING_CHANGE_RAD
            and not current.round_marker_detected
            and not current.transverse_line_detected
        )
        if not usable:
            self._reset_straight_far_candidate()
            return self._filter_straight_far_heading(0.0, current.timestamp_s)

        previous = self._straight_far_candidate_rad
        stable = (
            previous is not None
            and abs(heading_error - previous)
            <= STRAIGHT_FAR_MAX_FRAME_STEP_RAD
        )
        self._straight_far_valid_frames = (
            self._straight_far_valid_frames + 1 if stable else 1
        )
        self._straight_far_candidate_rad = heading_error
        self._straight_far_heading_error_rad = heading_error
        if self._straight_far_valid_frames < STRAIGHT_FAR_REQUIRED_FRAMES:
            return self._filter_straight_far_heading(
                0.0,
                current.timestamp_s,
            )

        magnitude = max(
            0.0,
            abs(heading_error) - STRAIGHT_FAR_HEADING_DEADBAND_RAD,
        )
        requested = math.copysign(
            STRAIGHT_FAR_HEADING_GAIN * magnitude,
            heading_error,
        )
        requested = max(
            -STRAIGHT_FAR_MAX_CORRECTION_RAD,
            min(STRAIGHT_FAR_MAX_CORRECTION_RAD, requested),
        )
        return self._filter_straight_far_heading(
            requested,
            current.timestamp_s,
        )

    @staticmethod
    def _straight_far_window_active(state: TrackFollowerState) -> bool:
        if not state.running or state.completed:
            return False
        if state.segment is TrackSegment.AB:
            local_progress = state.progress_cm
        elif state.segment is TrackSegment.CD:
            local_progress = (
                state.progress_cm
                - TRACK_STRAIGHT_LENGTH_CM
                - TRACK_SEMICIRCLE_LENGTH_CM
            )
        else:
            return False
        return (
            STRAIGHT_FAR_MARGIN_CM
            <= local_progress
            <= TRACK_STRAIGHT_LENGTH_CM - STRAIGHT_FAR_MARGIN_CM
        )

    @staticmethod
    def _far_path_heading_error(observation: LineObservation) -> float:
        polynomial = observation.polynomial_y_left_by_x
        if polynomial is None:
            return math.nan
        near_y = (
            polynomial[0] * STRAIGHT_FAR_NEAR_PROBE_CM**2
            + polynomial[1] * STRAIGHT_FAR_NEAR_PROBE_CM
            + polynomial[2]
        )
        far_y = (
            polynomial[0] * STRAIGHT_FAR_PROBE_CM**2
            + polynomial[1] * STRAIGHT_FAR_PROBE_CM
            + polynomial[2]
        )
        return math.atan2(
            far_y - near_y,
            STRAIGHT_FAR_PROBE_CM - STRAIGHT_FAR_NEAR_PROBE_CM,
        )

    def _filter_straight_far_heading(
        self,
        requested_rad: float,
        timestamp_s: float,
    ) -> float:
        previous_time = self._straight_far_last_filter_s
        dt = (
            1.0 / 30.0
            if previous_time is None
            else max(1e-3, min(0.25, timestamp_s - previous_time))
        )
        self._straight_far_last_filter_s = timestamp_s
        alpha = 1.0 - math.exp(
            -dt / STRAIGHT_FAR_FILTER_TIME_CONSTANT_S
        )
        low_passed = self._straight_far_filtered_rad + alpha * (
            requested_rad - self._straight_far_filtered_rad
        )
        maximum_delta = STRAIGHT_FAR_MAX_CORRECTION_RATE_RAD_S * dt
        self._straight_far_filtered_rad = max(
            self._straight_far_filtered_rad - maximum_delta,
            min(
                self._straight_far_filtered_rad + maximum_delta,
                low_passed,
            ),
        )
        return self._straight_far_filtered_rad

    def _reset_straight_far_candidate(self) -> None:
        self._straight_far_valid_frames = 0
        self._straight_far_candidate_rad = None
        self._straight_far_heading_error_rad = 0.0

    def _reset_straight_far_heading(self) -> None:
        self._reset_straight_far_candidate()
        self._straight_far_last_observation_s = None
        self._straight_far_last_filter_s = None
        self._straight_far_filtered_rad = 0.0

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

    def _final_da_trim(self) -> float | None:
        with self._lock:
            state = self._follower_state
        if (
            not state.running
            or state.completed
            or state.segment is not TrackSegment.DA
            or state.progress_cm < FINAL_DA_TRIM_START_PROGRESS_CM
        ):
            return None
        span_cm = (
            FINAL_DA_TRIM_FULL_PROGRESS_CM
            - FINAL_DA_TRIM_START_PROGRESS_CM
        )
        blend = min(
            1.0,
            max(
                0.0,
                (
                    state.progress_cm
                    - FINAL_DA_TRIM_START_PROGRESS_CM
                )
                / span_cm,
            ),
        )
        return FINAL_DA_MIN_LEFT_CORRECTION_RAD * blend

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
    parser.add_argument(
        "--speed-cm-s",
        type=float,
        default=TRACK_SPEED_CM_S,
    )
    parser.add_argument("--camera", type=_camera_source, default=0)
    parser.add_argument(
        "--no-camera-correction",
        action="store_true",
        help="run the copied fixed-track program in radar-only mode",
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
                speed_cm_s=args.speed_cm_s,
                camera_source=args.camera,
                camera_correction_enabled=(
                    not args.no_camera_correction
                ),
                camera_correction=_default_correction_config(),
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
