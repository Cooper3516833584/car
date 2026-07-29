#!/usr/bin/env python3
"""Camera-only black-line follower for an Ackermann vehicle.

The component owns camera capture, black-line extraction, local path fitting and
lateral control. It has no dependency on radar, global positioning or a fixed
competition-map path.

Coordinate/sign conventions:
- vehicle +X: forward
- vehicle +Y: left
- steering > 0: turn left
- image u grows to the right, so a line left of image centre produces +Y

The injected ``drive`` object is expected to expose the same public methods as
``components.ackermann_drive.AckermannDrive``. The component never starts or
closes that object; the application remains responsible for its lifecycle.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
import logging
import math
import threading
import time
from typing import Callable, Final, Protocol, Sequence

import numpy as np

try:
    import cv2  # type: ignore
    # Some development images expose an empty ``cv2`` namespace package even
    # though the compiled OpenCV bindings are absent.  Treat that exactly as
    # an unavailable OpenCV installation rather than failing only at runtime.
    if not all(
        hasattr(cv2, name)
        for name in ("VideoCapture", "getPerspectiveTransform", "warpPerspective")
    ):
        cv2 = None  # type: ignore
except ImportError:  # Import-safe on development machines without OpenCV.
    cv2 = None  # type: ignore

try:
    from .rear_motor import MotorDirection
except ImportError:  # Allows isolated tests outside the package.
    class MotorDirection(Enum):
        FORWARD = 1
        REVERSE = -1


LOG = logging.getLogger(__name__)


class AckermannDriveLike(Protocol):
    @property
    def is_running(self) -> bool: ...

    def set_motion(
        self,
        speed_mm_s: float,
        steering_angle_rad: float,
        *,
        direction: MotorDirection,
        rear_differential_linked: bool,
    ): ...

    def stop(self, *, center_steering: bool = True) -> None: ...


class LineFollowerStatus(Enum):
    IDLE = "idle"
    STARTING = "starting"
    TRACKING = "tracking"
    DEGRADED = "degraded"
    LOST = "lost"
    FINISHED = "finished"
    CAMERA_ERROR = "camera_error"
    STOPPED = "stopped"
    CLOSED = "closed"


@dataclass(frozen=True, slots=True)
class PerspectiveConfig:
    """Ground-plane perspective transform.

    ``source_points_norm`` are image-normalized coordinates in this order:
    bottom-left, bottom-right, top-right, top-left. They must outline a ground
    trapezoid. The destination is a rectangle whose bottom edge is nearest the
    car and whose top edge is farthest away.

    The camera's nominal 60-degree mounting pitch is *not* sufficient to infer
    this transform reliably: camera height, intrinsics, lens distortion and the
    exact mount angle also matter. Calibrate these four points on the real car.
    """

    source_points_norm: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ] = (
        (0.06, 0.98),
        (0.94, 0.98),
        (0.64, 0.40),
        (0.36, 0.40),
    )
    output_width_px: int = 320
    output_height_px: int = 400
    ground_width_cm: float = 80.0
    ground_depth_cm: float = 100.0

    def __post_init__(self) -> None:
        if self.output_width_px < 64 or self.output_height_px < 64:
            raise ValueError("perspective output dimensions are too small")
        if self.ground_width_cm <= 0.0 or self.ground_depth_cm <= 0.0:
            raise ValueError("ground dimensions must be positive")
        if len(self.source_points_norm) != 4:
            raise ValueError("exactly four source points are required")
        for x, y in self.source_points_norm:
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
                raise ValueError("normalized perspective points must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class LineVisionConfig:
    frame_width: int = 640
    frame_height: int = 360
    camera_fps: float = 30.0
    capture_backend_v4l2: bool = True
    fourcc: str = "MJPG"
    warmup_frames: int = 12
    max_consecutive_capture_failures: int = 5

    perspective: PerspectiveConfig = field(default_factory=PerspectiveConfig)

    # Optional calibrated pinhole parameters. Leave both None to skip undistort.
    camera_matrix: tuple[tuple[float, float, float], ...] | None = None
    distortion_coefficients: tuple[float, ...] | None = None

    clahe_clip_limit: float = 2.0
    clahe_tile_grid: tuple[int, int] = (8, 8)
    adaptive_block_size: int = 31
    adaptive_c: float = 7.0
    require_adaptive_confirmation: bool = True
    dark_percentile: float = 34.0
    dark_percentile_margin: float = 10.0
    maximum_dark_threshold: int = 175
    morphology_open_size: int = 3
    morphology_close_size: int = 5

    scan_count: int = 13
    scan_near_cm: float = 8.0
    scan_far_cm: float = 78.0
    scan_band_height_px: int = 7
    minimum_band_fill_ratio: float = 0.30
    use_expected_width_window: bool = False
    expected_line_width_cm: float = 5.0
    minimum_line_width_cm: float = 1.5
    maximum_line_width_cm: float = 16.0
    maximum_line_internal_gap_cm: float = 0.0
    maximum_center_jump_cm: float = 17.0
    minimum_fit_points: int = 5
    fit_outlier_floor_cm: float = 1.5
    fit_outlier_sigma: float = 2.8
    maximum_fit_rmse_cm: float = 5.0
    polynomial_smoothing_alpha: float = 0.35

    # A finish marker is a black line running across the track, perpendicular
    # to the longitudinal guide line.  In the bird's-eye image it therefore
    # appears as a wide horizontal connected component.
    transverse_stop_min_forward_cm: float = 10.0
    transverse_stop_max_forward_cm: float = 78.0
    transverse_stop_min_width_cm: float = 45.0
    transverse_stop_min_height_cm: float = 1.2
    transverse_stop_max_height_cm: float = 10.0
    round_marker_min_height_cm: float = 12.0

    # Candidate association weights. Larger continuity weight makes temporal
    # tracking stronger and rejects unrelated dark objects/shadows.
    continuity_weight: float = 1.0
    width_weight: float = 0.45
    darkness_weight: float = 0.15

    def __post_init__(self) -> None:
        if self.frame_width <= 0 or self.frame_height <= 0:
            raise ValueError("frame dimensions must be positive")
        if self.camera_fps <= 0.0:
            raise ValueError("camera_fps must be positive")
        if len(self.fourcc) != 4:
            raise ValueError("fourcc must contain exactly four characters")
        if self.adaptive_block_size < 3 or self.adaptive_block_size % 2 == 0:
            raise ValueError("adaptive_block_size must be odd and >= 3")
        if not 0.0 < self.dark_percentile < 100.0:
            raise ValueError("dark_percentile must be in (0, 100)")
        if self.scan_count < 5:
            raise ValueError("scan_count must be at least 5")
        if not 0.0 <= self.scan_near_cm < self.scan_far_cm:
            raise ValueError("scan range is invalid")
        if self.scan_far_cm > self.perspective.ground_depth_cm:
            raise ValueError("scan_far_cm exceeds perspective ground depth")
        if not (
            0.0 < self.minimum_line_width_cm
            <= self.expected_line_width_cm
            <= self.maximum_line_width_cm
        ):
            raise ValueError("line width limits are invalid")
        if self.maximum_line_internal_gap_cm < 0.0:
            raise ValueError("maximum_line_internal_gap_cm cannot be negative")
        if self.minimum_fit_points > self.scan_count:
            raise ValueError("minimum_fit_points exceeds scan_count")
        if not 0.0 < self.polynomial_smoothing_alpha <= 1.0:
            raise ValueError("polynomial_smoothing_alpha must be in (0, 1]")
        if not (
            0.0 <= self.transverse_stop_min_forward_cm
            < self.transverse_stop_max_forward_cm
            <= self.perspective.ground_depth_cm
        ):
            raise ValueError("transverse stop range is invalid")
        if (
            self.transverse_stop_min_width_cm <= 0.0
            or self.transverse_stop_min_width_cm > self.perspective.ground_width_cm
            or self.transverse_stop_min_height_cm <= 0.0
            or self.transverse_stop_max_height_cm
            <= self.transverse_stop_min_height_cm
            or self.round_marker_min_height_cm
            <= self.transverse_stop_max_height_cm
        ):
            raise ValueError("transverse stop dimensions are invalid")


@dataclass(frozen=True, slots=True)
class LineControlConfig:
    wheelbase_cm: float = 14.25
    cruise_speed_mm_s: float = 100.0
    degraded_speed_mm_s: float = 55.0
    short_loss_speed_mm_s: float = 35.0
    minimum_tracking_speed_mm_s: float = 45.0

    minimum_lookahead_cm: float = 25.0
    maximum_lookahead_cm: float = 48.0
    lookahead_speed_gain_s: float = 0.09
    lateral_gain: float = 0.35
    heading_gain: float = 0.30
    curvature_feedforward_gain: float = 1.0
    maximum_abs_steering_rad: float = 0.28
    maximum_steering_rate_rad_s: float = 1.05
    steering_low_pass_time_constant_s: float = 0.10
    steering_deadband_rad: float = 0.006
    curvature_speed_gain: float = 1.8

    tracking_confidence: float = 0.58
    degraded_confidence: float = 0.36
    recovery_good_frames: int = 3
    short_loss_frames: int = 3
    stale_observation_timeout_s: float = 0.20

    # The car may start with the finish/start line in view.  Do not arm the
    # finish marker until it has disappeared for several valid camera frames.
    finish_line_enabled: bool = True
    finish_line_startup_grace_s: float = 1.0
    finish_line_clear_frames_to_arm: int = 3
    finish_line_confirm_frames: int = 2
    minimum_markers_before_finish: int = 3
    round_marker_clear_frames_to_arm: int = 3
    round_marker_confirm_frames: int = 2

    def __post_init__(self) -> None:
        if self.wheelbase_cm <= 0.0:
            raise ValueError("wheelbase_cm must be positive")
        if min(
            self.cruise_speed_mm_s,
            self.degraded_speed_mm_s,
            self.short_loss_speed_mm_s,
            self.minimum_tracking_speed_mm_s,
        ) < 0.0:
            raise ValueError("speeds cannot be negative")
        if not 0.0 < self.minimum_lookahead_cm <= self.maximum_lookahead_cm:
            raise ValueError("lookahead limits are invalid")
        if not 0.0 < self.degraded_confidence < self.tracking_confidence <= 1.0:
            raise ValueError("confidence thresholds are invalid")
        if self.recovery_good_frames <= 0 or self.short_loss_frames < 0:
            raise ValueError("frame counters are invalid")
        if self.finish_line_startup_grace_s < 0.0:
            raise ValueError("finish_line_startup_grace_s cannot be negative")
        if (
            self.finish_line_clear_frames_to_arm <= 0
            or self.finish_line_confirm_frames <= 0
            or self.round_marker_clear_frames_to_arm <= 0
            or self.round_marker_confirm_frames <= 0
        ):
            raise ValueError("track-marker frame counters must be positive")
        if self.minimum_markers_before_finish < 0:
            raise ValueError("minimum_markers_before_finish cannot be negative")


@dataclass(frozen=True, slots=True)
class LineObservation:
    timestamp_s: float
    detected: bool
    confidence: float
    lookahead_x_cm: float
    lookahead_y_left_cm: float
    near_lateral_error_cm: float
    heading_error_rad: float
    curvature_per_cm: float
    fit_rmse_cm: float
    visible_band_count: int
    total_band_count: int
    median_line_width_cm: float
    polynomial_y_left_by_x: tuple[float, float, float] | None
    dark_threshold: float
    transverse_line_detected: bool = False
    round_marker_detected: bool = False


@dataclass(frozen=True, slots=True)
class LineFollowerState:
    status: LineFollowerStatus = LineFollowerStatus.IDLE
    running: bool = False
    frame_index: int = 0
    timestamp_s: float = 0.0
    confidence: float = 0.0
    speed_mm_s: float = 0.0
    steering_angle_rad: float = 0.0
    lost_frames: int = 0
    good_frames: int = 0
    capture_failures: int = 0
    observation: LineObservation | None = None
    error: str | None = None
    finish_line_armed: bool = False
    marker_count: int = 0


@dataclass(frozen=True, slots=True)
class _ControlCommand:
    moving: bool
    speed_mm_s: float
    steering_angle_rad: float
    status: LineFollowerStatus


class BlackLineDetector:
    """Extract a local black-line path from one camera frame."""

    def __init__(self, config: LineVisionConfig = LineVisionConfig()) -> None:
        self.config = config
        self._homography: np.ndarray | None = None
        self._homography_frame_size: tuple[int, int] | None = None
        self._previous_polynomial: np.ndarray | None = None
        self._previous_confidence = 0.0
        self._clahe = None
        self._camera_matrix = (
            None
            if config.camera_matrix is None
            else np.asarray(config.camera_matrix, dtype=np.float64)
        )
        self._distortion = (
            None
            if config.distortion_coefficients is None
            else np.asarray(config.distortion_coefficients, dtype=np.float64)
        )

    def reset(self) -> None:
        self._previous_polynomial = None
        self._previous_confidence = 0.0

    def process(
        self,
        frame_bgr: np.ndarray,
        *,
        timestamp_s: float | None = None,
        return_debug: bool = False,
    ) -> LineObservation | tuple[LineObservation, np.ndarray]:
        self._require_opencv()
        if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 BGR image")
        if frame_bgr.dtype != np.uint8:
            raise ValueError("frame_bgr must have dtype uint8")

        timestamp = time.monotonic() if timestamp_s is None else float(timestamp_s)
        frame = frame_bgr
        if self._camera_matrix is not None and self._distortion is not None:
            frame = cv2.undistort(frame, self._camera_matrix, self._distortion)

        homography = self._get_homography(frame.shape[1], frame.shape[0])
        perspective = self.config.perspective
        bird = cv2.warpPerspective(
            frame,
            homography,
            (perspective.output_width_px, perspective.output_height_px),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )
        mask, enhanced_lightness, dark_threshold = self._segment_black_line(bird)
        points, widths = self._extract_band_centres(mask, enhanced_lightness)
        observation = self._fit_observation(
            points,
            widths,
            timestamp_s=timestamp,
            dark_threshold=dark_threshold,
        )
        transverse, round_marker = self._classify_track_markers(mask)
        observation = replace(
            observation,
            transverse_line_detected=transverse,
            round_marker_detected=round_marker,
        )

        if observation.detected and observation.polynomial_y_left_by_x is not None:
            self._previous_polynomial = np.asarray(
                observation.polynomial_y_left_by_x,
                dtype=np.float64,
            )
            self._previous_confidence = observation.confidence
        elif self._previous_confidence > 0.0:
            self._previous_confidence *= 0.75
            if self._previous_confidence < 0.15:
                self._previous_polynomial = None

        if not return_debug:
            return observation
        return observation, self._render_debug(bird, mask, points, observation)

    @staticmethod
    def _require_opencv() -> None:
        if cv2 is None:
            raise RuntimeError(
                "OpenCV is required; install python3-opencv or opencv-python"
            )

    def _get_homography(self, frame_width: int, frame_height: int) -> np.ndarray:
        size = (frame_width, frame_height)
        if self._homography is not None and self._homography_frame_size == size:
            return self._homography
        src = np.asarray(
            [
                (x * (frame_width - 1), y * (frame_height - 1))
                for x, y in self.config.perspective.source_points_norm
            ],
            dtype=np.float32,
        )
        out_w = self.config.perspective.output_width_px
        out_h = self.config.perspective.output_height_px
        dst = np.asarray(
            [
                (0.0, out_h - 1.0),
                (out_w - 1.0, out_h - 1.0),
                (out_w - 1.0, 0.0),
                (0.0, 0.0),
            ],
            dtype=np.float32,
        )
        self._homography = cv2.getPerspectiveTransform(src, dst)
        self._homography_frame_size = size
        return self._homography

    def _segment_black_line(
        self,
        bird_bgr: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        lab = cv2.cvtColor(bird_bgr, cv2.COLOR_BGR2LAB)
        lightness = lab[:, :, 0]
        if self._clahe is None:
            self._clahe = cv2.createCLAHE(
                clipLimit=self.config.clahe_clip_limit,
                tileGridSize=self.config.clahe_tile_grid,
            )
        enhanced = self._clahe.apply(lightness)
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)

        local = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.config.adaptive_block_size,
            self.config.adaptive_c,
        )
        percentile = float(
            np.percentile(blurred, self.config.dark_percentile)
        )
        otsu_threshold, _ = cv2.threshold(
            blurred,
            0,
            255,
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU,
        )
        dark_threshold = min(
            float(self.config.maximum_dark_threshold),
            max(18.0, min(float(otsu_threshold) + 10.0, percentile + self.config.dark_percentile_margin)),
        )
        global_dark = np.where(blurred <= dark_threshold, 255, 0).astype(np.uint8)
        mask = (
            cv2.bitwise_and(local, global_dark)
            if self.config.require_adaptive_confirmation
            else global_dark
        )

        if self.config.morphology_open_size > 1:
            k = self.config.morphology_open_size
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if self.config.morphology_close_size > 1:
            k = self.config.morphology_close_size
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask, enhanced, dark_threshold

    def _extract_band_centres(
        self,
        mask: np.ndarray,
        enhanced_lightness: np.ndarray,
    ) -> tuple[list[tuple[float, float]], list[float]]:
        perspective = self.config.perspective
        h, w = mask.shape
        cm_per_px_x = perspective.ground_width_cm / w
        cm_per_px_y = perspective.ground_depth_cm / h
        min_width_px = max(2, round(self.config.minimum_line_width_cm / cm_per_px_x))
        max_width_px = max(min_width_px + 1, round(self.config.maximum_line_width_cm / cm_per_px_x))
        expected_width_px = self.config.expected_line_width_cm / cm_per_px_x
        max_internal_gap_px = round(
            self.config.maximum_line_internal_gap_cm / cm_per_px_x
        )
        max_jump_px = self.config.maximum_center_jump_cm / cm_per_px_x

        forward_positions = np.linspace(
            self.config.scan_near_cm,
            self.config.scan_far_cm,
            self.config.scan_count,
        )
        points: list[tuple[float, float]] = []
        widths_cm: list[float] = []
        last_selected_u: float | None = None

        for forward_cm in forward_positions:
            y = int(round((h - 1) - forward_cm / cm_per_px_y))
            half = self.config.scan_band_height_px // 2
            y0 = max(0, y - half)
            y1 = min(h, y + half + 1)
            if y0 >= y1:
                continue
            fill = np.mean(mask[y0:y1] > 0, axis=0)
            expected_u = self._expected_u(forward_cm, w)
            if expected_u is None:
                expected_u = (
                    last_selected_u
                    if last_selected_u is not None
                    else (w - 1) / 2.0
                )
            if self.config.use_expected_width_window:
                centre = self._expected_width_window_center(
                    fill,
                    expected_u=expected_u,
                    expected_width_px=max(3, round(expected_width_px)),
                    maximum_jump_px=max_jump_px,
                )
                if centre is None:
                    continue
                last_selected_u = centre
                y_left_cm = ((w - 1) / 2.0 - centre) * cm_per_px_x
                points.append((float(forward_cm), float(y_left_cm)))
                widths_cm.append(self.config.expected_line_width_cm)
                continue

            occupied = fill >= self.config.minimum_band_fill_ratio
            runs = self._true_runs(occupied)
            if max_internal_gap_px > 0:
                runs = self._merge_nearby_runs(
                    runs,
                    maximum_gap_px=max_internal_gap_px,
                    maximum_width_px=max_width_px,
                )
            candidates: list[tuple[float, int, int, float]] = []
            for start, end in runs:
                width = end - start
                if width < min_width_px or width > max_width_px:
                    continue
                centre = 0.5 * (start + end - 1)
                continuity = abs(centre - expected_u) / max(1.0, max_jump_px)
                if continuity > 1.6:
                    continue
                width_error = abs(width - expected_width_px) / max(1.0, expected_width_px)
                strip = enhanced_lightness[y0:y1, start:end]
                darkness = float(np.mean(strip)) / 255.0 if strip.size else 1.0
                score = (
                    self.config.continuity_weight * continuity
                    + self.config.width_weight * width_error
                    + self.config.darkness_weight * darkness
                )
                candidates.append((score, start, end, centre))

            if not candidates:
                continue
            _, start, end, centre = min(candidates, key=lambda item: item[0])
            last_selected_u = centre
            y_left_cm = ((w - 1) / 2.0 - centre) * cm_per_px_x
            points.append((float(forward_cm), float(y_left_cm)))
            widths_cm.append((end - start) * cm_per_px_x)

        return points, widths_cm

    def _expected_width_window_center(
        self,
        fill: np.ndarray,
        *,
        expected_u: float,
        expected_width_px: int,
        maximum_jump_px: float,
    ) -> float | None:
        """Find the darkest full-width lane window, not one textured edge."""

        width = len(fill)
        window = min(width - 2, max(3, expected_width_px))
        density = np.convolve(
            fill,
            np.ones(window, dtype=np.float64) / window,
            mode="same",
        )
        centres = np.arange(width, dtype=np.float64)
        half = window / 2.0
        valid = (
            (centres >= half)
            & (centres < width - half)
            & (np.abs(centres - expected_u) <= 1.6 * maximum_jump_px)
            & (density >= self.config.minimum_band_fill_ratio)
        )
        if not np.any(valid):
            return None
        continuity = np.abs(centres - expected_u) / max(1.0, maximum_jump_px)
        score = (1.0 - density) + self.config.continuity_weight * continuity
        score[~valid] = math.inf
        return float(np.argmin(score))

    def _classify_track_markers(self, mask: np.ndarray) -> tuple[bool, bool]:
        """Distinguish a thin finish stripe from the large round segment dots."""

        perspective = self.config.perspective
        height_px, width_px = mask.shape
        cm_per_px_x = perspective.ground_width_cm / width_px
        cm_per_px_y = perspective.ground_depth_cm / height_px
        min_width_px = math.ceil(
            self.config.transverse_stop_min_width_cm / cm_per_px_x
        )
        min_height_px = max(
            1,
            math.ceil(self.config.transverse_stop_min_height_cm / cm_per_px_y),
        )
        max_height_px = max(
            min_height_px,
            math.floor(self.config.transverse_stop_max_height_cm / cm_per_px_y),
        )
        round_height_px = max(
            max_height_px + 1,
            math.ceil(self.config.round_marker_min_height_cm / cm_per_px_y),
        )
        transverse_row_start = max(
            0,
            math.floor(
                height_px
                - self.config.transverse_stop_max_forward_cm / cm_per_px_y
            ),
        )
        transverse_row_end = min(
            height_px,
            math.ceil(
                height_px
                - self.config.transverse_stop_min_forward_cm / cm_per_px_y
            ),
        )
        if transverse_row_end <= transverse_row_start:
            return False, False

        # A round point can extend into the near 0..10 cm region.  Classify it
        # over the complete bird view; only the finish stripe uses the narrower
        # forward range that excludes the vehicle nose.
        wide_rows = np.zeros(height_px, dtype=bool)
        for index, row in enumerate(mask):
            for start, end in self._true_runs(row > 0):
                if end - start >= min_width_px:
                    wide_rows[index] = True
                    break
        full_spans = tuple(self._true_runs(wide_rows))
        round_spans = tuple(
            (start, end)
            for start, end in full_spans
            if end - start >= round_height_px
        )
        transverse_spans = self._true_runs(
            wide_rows[transverse_row_start:transverse_row_end]
        )
        transverse = False
        for local_start, local_end in transverse_spans:
            start = local_start + transverse_row_start
            end = local_end + transverse_row_start
            height = end - start
            overlaps_round = any(
                start < round_end and end > round_start
                for round_start, round_end in round_spans
            )
            if (
                min_height_px <= height <= max_height_px
                and not overlaps_round
            ):
                transverse = True
                break
        round_marker = bool(round_spans)
        return transverse, round_marker

    @staticmethod
    def _true_runs(values: np.ndarray) -> list[tuple[int, int]]:
        padded = np.pad(values.astype(np.int8), (1, 1))
        transitions = np.diff(padded)
        starts = np.flatnonzero(transitions == 1)
        ends = np.flatnonzero(transitions == -1)
        return [(int(a), int(b)) for a, b in zip(starts, ends)]

    @staticmethod
    def _merge_nearby_runs(
        runs: Sequence[tuple[int, int]],
        *,
        maximum_gap_px: int,
        maximum_width_px: int,
    ) -> list[tuple[int, int]]:
        if not runs:
            return []
        merged: list[tuple[int, int]] = []
        start, end = runs[0]
        for next_start, next_end in runs[1:]:
            if (
                next_start - end <= maximum_gap_px
                and next_end - start <= maximum_width_px
            ):
                end = next_end
            else:
                merged.append((start, end))
                start, end = next_start, next_end
        merged.append((start, end))
        return merged

    def _expected_u(self, forward_cm: float, width_px: int) -> float | None:
        if self._previous_polynomial is None:
            return None
        y_left_cm = float(np.polyval(self._previous_polynomial, forward_cm))
        cm_per_px = self.config.perspective.ground_width_cm / width_px
        return (width_px - 1) / 2.0 - y_left_cm / cm_per_px

    def _fit_observation(
        self,
        points: Sequence[tuple[float, float]],
        widths_cm: Sequence[float],
        *,
        timestamp_s: float,
        dark_threshold: float,
    ) -> LineObservation:
        total = self.config.scan_count
        if len(points) < self.config.minimum_fit_points:
            return self._empty_observation(
                timestamp_s,
                visible_band_count=len(points),
                total_band_count=total,
                dark_threshold=dark_threshold,
            )

        array = np.asarray(points, dtype=np.float64)
        x = array[:, 0]
        y = array[:, 1]
        keep = np.ones(len(array), dtype=bool)
        polynomial: np.ndarray | None = None
        for _ in range(3):
            if np.count_nonzero(keep) < self.config.minimum_fit_points:
                break
            # Near points have more control relevance, but far points still
            # contribute to curvature prediction.
            weights = 1.35 - 0.55 * (
                (x[keep] - self.config.scan_near_cm)
                / max(1e-6, self.config.scan_far_cm - self.config.scan_near_cm)
            )
            polynomial = np.polyfit(x[keep], y[keep], deg=2, w=weights)
            residual = y - np.polyval(polynomial, x)
            kept_residual = residual[keep]
            median = float(np.median(kept_residual))
            mad = float(np.median(np.abs(kept_residual - median)))
            robust_sigma = 1.4826 * mad
            threshold = max(
                self.config.fit_outlier_floor_cm,
                self.config.fit_outlier_sigma * robust_sigma,
            )
            new_keep = np.abs(residual - median) <= threshold
            if np.array_equal(new_keep, keep):
                break
            keep = new_keep

        if polynomial is None or np.count_nonzero(keep) < self.config.minimum_fit_points:
            return self._empty_observation(
                timestamp_s,
                visible_band_count=int(np.count_nonzero(keep)),
                total_band_count=total,
                dark_threshold=dark_threshold,
            )

        if self._previous_polynomial is not None:
            alpha = self.config.polynomial_smoothing_alpha
            polynomial = (
                alpha * polynomial
                + (1.0 - alpha) * self._previous_polynomial
            )
        x_kept = x[keep]
        y_kept = y[keep]
        residual = y_kept - np.polyval(polynomial, x_kept)
        rmse = float(math.sqrt(float(np.mean(np.square(residual)))))
        visible_count = int(np.count_nonzero(keep))
        coverage = visible_count / total
        rmse_score = math.exp(-rmse / max(0.5, self.config.maximum_fit_rmse_cm * 0.45))
        median_width = float(np.median(widths_cm)) if widths_cm else 0.0
        width_score = math.exp(
            -abs(median_width - self.config.expected_line_width_cm)
            / max(1.0, self.config.expected_line_width_cm)
        )
        near_visible = float(np.min(x_kept)) <= self.config.scan_near_cm + (
            self.config.scan_far_cm - self.config.scan_near_cm
        ) / max(1, self.config.scan_count - 1) * 2.1
        temporal_score = self._temporal_fit_score(polynomial)
        confidence = float(
            np.clip(
                0.42 * coverage
                + 0.25 * rmse_score
                + 0.13 * width_score
                + 0.12 * (1.0 if near_visible else 0.0)
                + 0.08 * temporal_score,
                0.0,
                1.0,
            )
        )

        lookahead_x = min(
            self.config.scan_far_cm,
            max(self.config.scan_near_cm, 35.0),
        )
        lookahead_y = float(np.polyval(polynomial, lookahead_x))
        near_x = max(self.config.scan_near_cm, 10.0)
        near_error = float(np.polyval(polynomial, near_x))
        derivative = float(2.0 * polynomial[0] * lookahead_x + polynomial[1])
        heading = math.atan(derivative)
        second = float(2.0 * polynomial[0])
        curvature = second / max(1e-9, (1.0 + derivative * derivative) ** 1.5)
        detected = (
            visible_count >= self.config.minimum_fit_points
            and rmse <= self.config.maximum_fit_rmse_cm
        )
        if not detected:
            confidence *= 0.5

        return LineObservation(
            timestamp_s=timestamp_s,
            detected=detected,
            confidence=confidence,
            lookahead_x_cm=lookahead_x,
            lookahead_y_left_cm=lookahead_y,
            near_lateral_error_cm=near_error,
            heading_error_rad=heading,
            curvature_per_cm=curvature,
            fit_rmse_cm=rmse,
            visible_band_count=visible_count,
            total_band_count=total,
            median_line_width_cm=median_width,
            polynomial_y_left_by_x=(
                float(polynomial[0]),
                float(polynomial[1]),
                float(polynomial[2]),
            ),
            dark_threshold=dark_threshold,
        )

    def _temporal_fit_score(self, polynomial: np.ndarray) -> float:
        if self._previous_polynomial is None:
            return 0.75
        sample_x = np.asarray((12.0, 30.0, 55.0), dtype=np.float64)
        delta = np.abs(
            np.polyval(polynomial, sample_x)
            - np.polyval(self._previous_polynomial, sample_x)
        )
        return math.exp(-float(np.mean(delta)) / 8.0)

    @staticmethod
    def _empty_observation(
        timestamp_s: float,
        *,
        visible_band_count: int,
        total_band_count: int,
        dark_threshold: float,
    ) -> LineObservation:
        return LineObservation(
            timestamp_s=timestamp_s,
            detected=False,
            confidence=0.0,
            lookahead_x_cm=0.0,
            lookahead_y_left_cm=0.0,
            near_lateral_error_cm=0.0,
            heading_error_rad=0.0,
            curvature_per_cm=0.0,
            fit_rmse_cm=math.inf,
            visible_band_count=visible_band_count,
            total_band_count=total_band_count,
            median_line_width_cm=0.0,
            polynomial_y_left_by_x=None,
            dark_threshold=dark_threshold,
        )

    def _render_debug(
        self,
        bird_bgr: np.ndarray,
        mask: np.ndarray,
        points: Sequence[tuple[float, float]],
        observation: LineObservation,
    ) -> np.ndarray:
        overlay = bird_bgr.copy()
        h, w = mask.shape
        cm_per_px_x = self.config.perspective.ground_width_cm / w
        cm_per_px_y = self.config.perspective.ground_depth_cm / h
        mask_color = np.zeros_like(overlay)
        mask_color[:, :, 1] = mask
        overlay = cv2.addWeighted(overlay, 0.78, mask_color, 0.22, 0.0)
        for forward_cm, y_left_cm in points:
            u = int(round((w - 1) / 2.0 - y_left_cm / cm_per_px_x))
            v = int(round((h - 1) - forward_cm / cm_per_px_y))
            cv2.circle(overlay, (u, v), 4, (0, 255, 255), -1)
        if observation.polynomial_y_left_by_x is not None:
            poly = np.asarray(observation.polynomial_y_left_by_x)
            curve = []
            for forward_cm in np.linspace(
                self.config.scan_near_cm,
                self.config.scan_far_cm,
                100,
            ):
                y_left_cm = float(np.polyval(poly, forward_cm))
                u = int(round((w - 1) / 2.0 - y_left_cm / cm_per_px_x))
                v = int(round((h - 1) - forward_cm / cm_per_px_y))
                if 0 <= u < w and 0 <= v < h:
                    curve.append((u, v))
            if len(curve) >= 2:
                cv2.polylines(
                    overlay,
                    [np.asarray(curve, dtype=np.int32)],
                    False,
                    (0, 0, 255),
                    2,
                )
        cv2.putText(
            overlay,
            f"conf={observation.confidence:.2f} rmse={observation.fit_rmse_cm:.1f}cm bands={observation.visible_band_count}/{observation.total_band_count}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return overlay


class CameraLineFollower:
    """Capture camera frames and command an injected Ackermann drive."""

    def __init__(
        self,
        *,
        drive: AckermannDriveLike,
        camera_index: int | str = 0,
        vision_config: LineVisionConfig = LineVisionConfig(),
        control_config: LineControlConfig = LineControlConfig(),
        detector: BlackLineDetector | None = None,
        on_state_changed: Callable[[LineFollowerState], None] | None = None,
        on_marker_passed: Callable[[int], None] | None = None,
        on_debug_frame: Callable[[np.ndarray, LineObservation], None] | None = None,
        debug_frame_interval: int = 3,
    ) -> None:
        if debug_frame_interval <= 0:
            raise ValueError("debug_frame_interval must be positive")
        self.drive = drive
        self.camera_index = camera_index
        self.vision_config = vision_config
        self.control_config = control_config
        self.detector = detector or BlackLineDetector(vision_config)
        self._on_state_changed = on_state_changed
        self._on_marker_passed = on_marker_passed
        self._on_debug_frame = on_debug_frame
        self._debug_frame_interval = debug_frame_interval

        self._lock = threading.Lock()
        self._state = LineFollowerState()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._capture = None
        self._last_steering = 0.0
        self._last_control_time: float | None = None
        self._lost_frames = 0
        self._good_frames = 0
        self._drive_stopped = True
        self._run_started_at_s: float | None = None
        self._finish_line_armed = False
        self._finish_line_clear_frames = 0
        self._finish_line_confirm_frames = 0
        self._round_marker_armed = False
        self._round_marker_clear_frames = 0
        self._round_marker_confirm_frames = 0
        self._marker_count = 0
        self._active_cruise_speed_mm_s = control_config.cruise_speed_mm_s

    @property
    def state(self) -> LineFollowerState:
        with self._lock:
            return self._state

    @property
    def is_running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def set_cruise_speed_mm_s(self, speed_mm_s: float) -> None:
        speed = float(speed_mm_s)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("speed_mm_s must be positive and finite")
        self._active_cruise_speed_mm_s = speed

    def start(self) -> "CameraLineFollower":
        self.detector._require_opencv()
        with self._lock:
            if self._state.status is LineFollowerStatus.CLOSED:
                raise RuntimeError("camera line follower is closed")
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("camera line follower is already running")
            if not self.drive.is_running:
                raise RuntimeError("AckermannDrive must be started first")
            self._stop_event.clear()
            self._run_started_at_s = time.monotonic()
            self._finish_line_armed = False
            self._finish_line_clear_frames = 0
            self._finish_line_confirm_frames = 0
            self._round_marker_armed = False
            self._round_marker_clear_frames = 0
            self._round_marker_confirm_frames = 0
            self._marker_count = 0
            self._active_cruise_speed_mm_s = (
                self.control_config.cruise_speed_mm_s
            )
            self._state = replace(
                self._state,
                status=LineFollowerStatus.STARTING,
                running=True,
                error=None,
            )
            self._thread = threading.Thread(
                target=self._run,
                name="camera-line-follower",
                daemon=True,
            )
            self._thread.start()
            state = self._state
        self._notify(state)
        return self

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._safe_stop_drive()
        with self._lock:
            if self._state.status is not LineFollowerStatus.CLOSED:
                self._state = replace(
                    self._state,
                    status=LineFollowerStatus.STOPPED,
                    running=False,
                    speed_mm_s=0.0,
                )
                state = self._state
            else:
                state = None
            self._thread = None
        if state is not None:
            self._notify(state)

    def close(self) -> None:
        self.stop()
        with self._lock:
            self._state = replace(
                self._state,
                status=LineFollowerStatus.CLOSED,
                running=False,
                speed_mm_s=0.0,
            )
            state = self._state
        self._notify(state)

    def __enter__(self) -> "CameraLineFollower":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def compute_command(
        self,
        observation: LineObservation,
        *,
        now_s: float | None = None,
    ) -> _ControlCommand:
        """Pure control step, exposed for deterministic tests and tuning."""
        now = time.monotonic() if now_s is None else float(now_s)
        dt = (
            1.0 / self.vision_config.camera_fps
            if self._last_control_time is None
            else max(1e-3, min(0.25, now - self._last_control_time))
        )
        self._last_control_time = now

        fresh = now - observation.timestamp_s <= self.control_config.stale_observation_timeout_s
        good = (
            fresh
            and observation.detected
            and observation.confidence >= self.control_config.tracking_confidence
        )
        degraded = (
            fresh
            and observation.detected
            and observation.confidence >= self.control_config.degraded_confidence
        )

        if good:
            self._good_frames += 1
            self._lost_frames = 0
        elif degraded:
            self._good_frames = 0
            self._lost_frames += 1
        else:
            self._good_frames = 0
            self._lost_frames += 1

        if good or degraded:
            raw_steering = self._steering_from_observation(observation)
            steering = self._filter_steering(raw_steering, dt)
            if good and self._good_frames >= self.control_config.recovery_good_frames:
                speed = self._tracking_speed(observation, steering)
                return _ControlCommand(True, speed, steering, LineFollowerStatus.TRACKING)
            if degraded or self._good_frames < self.control_config.recovery_good_frames:
                return _ControlCommand(
                    True,
                    self._scaled_speed(
                        self.control_config.degraded_speed_mm_s
                    ),
                    steering,
                    LineFollowerStatus.DEGRADED,
                )

        if self._lost_frames <= self.control_config.short_loss_frames:
            steering = self._filter_steering(self._last_steering * 0.82, dt)
            return _ControlCommand(
                True,
                self._scaled_speed(
                    self.control_config.short_loss_speed_mm_s
                ),
                steering,
                LineFollowerStatus.DEGRADED,
            )
        self._last_steering = 0.0
        return _ControlCommand(False, 0.0, 0.0, LineFollowerStatus.LOST)

    def _steering_from_observation(self, observation: LineObservation) -> float:
        speed = max(
            self._scaled_speed(
                self.control_config.minimum_tracking_speed_mm_s
            ),
            self._active_cruise_speed_mm_s,
        )
        lookahead = float(
            np.clip(
                self.control_config.minimum_lookahead_cm
                + self.control_config.lookahead_speed_gain_s * speed,
                self.control_config.minimum_lookahead_cm,
                self.control_config.maximum_lookahead_cm,
            )
        )
        curvature_feedforward = (
            self.control_config.curvature_feedforward_gain
            * math.atan(
                self.control_config.wheelbase_cm
                * observation.curvature_per_cm
            )
        )
        lateral_feedback = self.control_config.lateral_gain * math.atan2(
            observation.near_lateral_error_cm,
            max(1.0, lookahead),
        )
        heading_feedback = (
            self.control_config.heading_gain
            * observation.heading_error_rad
        )
        raw = curvature_feedforward + lateral_feedback + heading_feedback
        raw = float(
            np.clip(
                raw,
                -self.control_config.maximum_abs_steering_rad,
                self.control_config.maximum_abs_steering_rad,
            )
        )
        if abs(raw) < self.control_config.steering_deadband_rad:
            return 0.0
        return raw

    def _tracking_speed(
        self,
        observation: LineObservation,
        steering_rad: float,
    ) -> float:
        steering_scale = 1.0 / (
            1.0
            + self.control_config.curvature_speed_gain
            * abs(steering_rad)
            / max(1e-6, self.control_config.maximum_abs_steering_rad)
        )
        confidence_scale = 0.70 + 0.30 * observation.confidence
        minimum_speed = self._scaled_speed(
            self.control_config.minimum_tracking_speed_mm_s
        )
        return float(
            np.clip(
                self._active_cruise_speed_mm_s
                * steering_scale
                * confidence_scale,
                minimum_speed,
                self._active_cruise_speed_mm_s,
            )
        )

    def _scaled_speed(self, configured_speed_mm_s: float) -> float:
        base = max(1e-6, self.control_config.cruise_speed_mm_s)
        return (
            float(configured_speed_mm_s)
            * self._active_cruise_speed_mm_s
            / base
        )

    def _filter_steering(self, requested: float, dt: float) -> float:
        tau = self.control_config.steering_low_pass_time_constant_s
        alpha = 1.0 if tau <= 0.0 else dt / (tau + dt)
        low_passed = self._last_steering + alpha * (requested - self._last_steering)
        maximum_delta = self.control_config.maximum_steering_rate_rad_s * dt
        filtered = float(
            np.clip(
                low_passed,
                self._last_steering - maximum_delta,
                self._last_steering + maximum_delta,
            )
        )
        filtered = float(
            np.clip(
                filtered,
                -self.control_config.maximum_abs_steering_rad,
                self.control_config.maximum_abs_steering_rad,
            )
        )
        self._last_steering = filtered
        return filtered

    def _run(self) -> None:
        capture_failures = 0
        frame_index = 0
        try:
            self._capture = self._open_capture()
            for _ in range(self.vision_config.warmup_frames):
                if self._stop_event.is_set():
                    return
                self._capture.grab()
            while not self._stop_event.is_set():
                ok, frame = self._capture.read()
                now = time.monotonic()
                if not ok or frame is None:
                    capture_failures += 1
                    self._safe_stop_drive()
                    self._publish_state(
                        status=LineFollowerStatus.CAMERA_ERROR,
                        running=True,
                        frame_index=frame_index,
                        timestamp_s=now,
                        speed_mm_s=0.0,
                        capture_failures=capture_failures,
                        error="camera frame capture failed",
                    )
                    if capture_failures >= self.vision_config.max_consecutive_capture_failures:
                        raise RuntimeError("camera repeatedly failed to return frames")
                    time.sleep(0.03)
                    continue

                capture_failures = 0
                frame_index += 1
                wants_debug = (
                    self._on_debug_frame is not None
                    and frame_index % self._debug_frame_interval == 0
                )
                result = self.detector.process(
                    frame,
                    timestamp_s=now,
                    return_debug=wants_debug,
                )
                if wants_debug:
                    observation, debug_frame = result  # type: ignore[misc]
                    try:
                        self._on_debug_frame(debug_frame, observation)  # type: ignore[misc]
                    except Exception:
                        LOG.exception("debug frame callback failed")
                else:
                    observation = result  # type: ignore[assignment]

                self._update_round_marker(observation, now)
                if self._finish_line_reached(observation, now):
                    self._safe_stop_drive()
                    self._publish_state(
                        status=LineFollowerStatus.FINISHED,
                        running=False,
                        frame_index=frame_index,
                        timestamp_s=now,
                        confidence=observation.confidence,
                        speed_mm_s=0.0,
                        steering_angle_rad=0.0,
                        lost_frames=self._lost_frames,
                        good_frames=self._good_frames,
                        capture_failures=0,
                        observation=observation,
                        finish_line_armed=True,
                        marker_count=self._marker_count,
                        error=None,
                    )
                    LOG.info("transverse finish line confirmed; vehicle stopped")
                    return

                command = self.compute_command(observation, now_s=now)
                if command.moving:
                    self.drive.set_motion(
                        command.speed_mm_s,
                        command.steering_angle_rad,
                        direction=MotorDirection.FORWARD,
                        rear_differential_linked=True,
                    )
                    self._drive_stopped = False
                else:
                    self._safe_stop_drive()
                self._publish_state(
                    status=command.status,
                    running=True,
                    frame_index=frame_index,
                    timestamp_s=now,
                    confidence=observation.confidence,
                    speed_mm_s=command.speed_mm_s if command.moving else 0.0,
                    steering_angle_rad=command.steering_angle_rad,
                    lost_frames=self._lost_frames,
                    good_frames=self._good_frames,
                    capture_failures=0,
                    observation=observation,
                    finish_line_armed=self._finish_line_armed,
                    marker_count=self._marker_count,
                    error=None,
                )
        except BaseException as exc:
            self._safe_stop_drive()
            LOG.exception("camera line follower failed")
            self._publish_state(
                status=LineFollowerStatus.CAMERA_ERROR,
                running=False,
                speed_mm_s=0.0,
                error=str(exc),
            )
        finally:
            capture, self._capture = self._capture, None
            if capture is not None:
                capture.release()
            self._safe_stop_drive()
            with self._lock:
                if self._state.status not in (
                    LineFollowerStatus.CLOSED,
                    LineFollowerStatus.CAMERA_ERROR,
                    LineFollowerStatus.FINISHED,
                ):
                    self._state = replace(
                        self._state,
                        status=LineFollowerStatus.STOPPED,
                        running=False,
                        speed_mm_s=0.0,
                    )

    def _open_capture(self):
        backend = (
            cv2.CAP_V4L2
            if self.vision_config.capture_backend_v4l2 and hasattr(cv2, "CAP_V4L2")
            else cv2.CAP_ANY
        )
        capture = cv2.VideoCapture(self.camera_index, backend)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open camera {self.camera_index!r}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.vision_config.frame_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.vision_config.frame_height)
        capture.set(cv2.CAP_PROP_FPS, self.vision_config.camera_fps)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*self.vision_config.fourcc),
        )
        return capture

    def _finish_line_reached(
        self,
        observation: LineObservation,
        now_s: float,
    ) -> bool:
        """Arm only after the startup line is gone, then confirm its return."""

        config = self.control_config
        if not config.finish_line_enabled:
            return False
        started_at = self._run_started_at_s
        if (
            started_at is None
            or now_s - started_at < config.finish_line_startup_grace_s
        ):
            return False
        visible = observation.transverse_line_detected
        if not self._finish_line_armed:
            if visible:
                self._finish_line_clear_frames = 0
            else:
                self._finish_line_clear_frames += 1
                if (
                    self._finish_line_clear_frames
                    >= config.finish_line_clear_frames_to_arm
                ):
                    self._finish_line_armed = True
                    LOG.info(
                        "finish line armed after the startup marker cleared"
                    )
            return False

        # After B, C, and D have been counted, the next debounced round marker
        # is necessarily A on this one-lap closed track.  This is a fallback
        # when the extra A transverse stripe is partly hidden by the low camera.
        if self._marker_count >= 4:
            return True
        if self._marker_count < config.minimum_markers_before_finish:
            self._finish_line_confirm_frames = 0
            return False
        if visible:
            self._finish_line_confirm_frames += 1
        else:
            self._finish_line_confirm_frames = 0
        return (
            self._finish_line_confirm_frames
            >= config.finish_line_confirm_frames
        )

    def _update_round_marker(
        self,
        observation: LineObservation,
        now_s: float,
    ) -> None:
        """Count each large endpoint dot once, ignoring the startup A dot."""

        config = self.control_config
        started_at = self._run_started_at_s
        if (
            started_at is None
            or now_s - started_at < config.finish_line_startup_grace_s
        ):
            return

        visible = observation.round_marker_detected
        if not self._round_marker_armed:
            if visible:
                self._round_marker_clear_frames = 0
            else:
                self._round_marker_clear_frames += 1
                if (
                    self._round_marker_clear_frames
                    >= config.round_marker_clear_frames_to_arm
                ):
                    self._round_marker_armed = True
                    LOG.info("round track-marker detector armed")
            return

        if not visible:
            self._round_marker_confirm_frames = 0
            return
        self._round_marker_confirm_frames += 1
        if (
            self._round_marker_confirm_frames
            < config.round_marker_confirm_frames
        ):
            return

        self._marker_count += 1
        marker_count = self._marker_count
        self._round_marker_armed = False
        self._round_marker_clear_frames = 0
        self._round_marker_confirm_frames = 0
        LOG.info("round track marker passed count=%d", marker_count)
        if self._on_marker_passed is not None:
            try:
                self._on_marker_passed(marker_count)
            except Exception:
                LOG.exception("track-marker callback failed")

    def _safe_stop_drive(self) -> None:
        if self._drive_stopped:
            return
        try:
            self.drive.stop(center_steering=True)
        except Exception:
            LOG.exception("failed to stop Ackermann drive")
        finally:
            self._drive_stopped = True

    def _publish_state(self, **changes) -> None:
        with self._lock:
            self._state = replace(self._state, **changes)
            state = self._state
        self._notify(state)

    def _notify(self, state: LineFollowerState) -> None:
        if self._on_state_changed is None:
            return
        try:
            self._on_state_changed(state)
        except Exception:
            LOG.exception("line follower state callback failed")


__all__: Final = [
    "BlackLineDetector",
    "CameraLineFollower",
    "LineControlConfig",
    "LineFollowerState",
    "LineFollowerStatus",
    "LineObservation",
    "LineVisionConfig",
    "PerspectiveConfig",
]
