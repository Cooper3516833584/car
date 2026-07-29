#!/usr/bin/env python3
"""Camera-derived soft steering correction for radar track following.

This component never opens the motor driver or steering servo.  It observes
the already-calibrated black line and publishes a small, rate-limited steering
increment.  Radar localization and the fixed-track Pure Pursuit controller
remain authoritative.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import logging
import math
import threading
import time
from typing import Callable

import numpy as np

from . import camera_line_follower as line_module
from .camera_line_follower import (
    BlackLineDetector,
    LineObservation,
    LineVisionConfig,
)


LOG = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class CameraLineCorrectionConfig:
    """Conservative gates and gains for camera-to-steering correction."""

    minimum_confidence: float = 0.72
    minimum_visible_bands: int = 7
    maximum_fit_rmse_cm: float = 2.5
    required_consecutive_frames: int = 4
    large_error_fast_activate_cm: float = 18.0
    large_error_required_frames: int = 2
    large_error_max_step_cm: float = 6.0
    curve_round_marker_minimum_confidence: float = 0.45
    curve_round_marker_minimum_visible_bands: int = 3
    curve_round_marker_maximum_fit_rmse_cm: float = 1.25
    curve_round_marker_required_frames: int = 5
    curve_round_marker_maximum_abs_correction_rad: float = 0.030
    curve_invalid_grace_frames: int = 4
    lateral_deadband_cm: float = 10.0
    steering_gain_rad_per_cm: float = 0.006
    maximum_abs_correction_rad: float = 0.055
    correction_filter_time_constant_s: float = 0.20
    maximum_correction_rate_rad_s: float = 0.20
    stale_timeout_s: float = 0.25
    stale_fade_out_s: float = 0.35
    full_correction_speed_cm_s: float = 25.0
    minimum_high_speed_scale: float = 0.50

    def __post_init__(self) -> None:
        if not 0.0 < self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in (0, 1]")
        if self.minimum_visible_bands <= 0:
            raise ValueError("minimum_visible_bands must be positive")
        if self.maximum_fit_rmse_cm <= 0.0:
            raise ValueError("maximum_fit_rmse_cm must be positive")
        if self.required_consecutive_frames <= 0:
            raise ValueError("required_consecutive_frames must be positive")
        if self.large_error_required_frames <= 0:
            raise ValueError("large_error_required_frames must be positive")
        if self.curve_round_marker_minimum_visible_bands <= 0:
            raise ValueError(
                "curve_round_marker_minimum_visible_bands must be positive"
            )
        if self.curve_round_marker_required_frames <= 0:
            raise ValueError(
                "curve_round_marker_required_frames must be positive"
            )
        if self.curve_invalid_grace_frames < 0:
            raise ValueError("curve_invalid_grace_frames cannot be negative")
        if not 0.0 < self.curve_round_marker_minimum_confidence <= 1.0:
            raise ValueError(
                "curve_round_marker_minimum_confidence must be in (0, 1]"
            )
        if (
            self.lateral_deadband_cm < 0.0
            or (
                self.large_error_fast_activate_cm
                <= self.lateral_deadband_cm
            )
            or self.large_error_max_step_cm <= 0.0
            or self.curve_round_marker_maximum_fit_rmse_cm <= 0.0
            or self.curve_round_marker_maximum_abs_correction_rad <= 0.0
            or (
                self.curve_round_marker_maximum_abs_correction_rad
                > self.maximum_abs_correction_rad
            )
            or self.steering_gain_rad_per_cm < 0.0
            or self.maximum_abs_correction_rad <= 0.0
            or self.correction_filter_time_constant_s < 0.0
            or self.maximum_correction_rate_rad_s <= 0.0
            or self.stale_timeout_s <= 0.0
            or self.stale_fade_out_s <= 0.0
            or self.full_correction_speed_cm_s <= 0.0
        ):
            raise ValueError("camera correction parameters are invalid")
        if not 0.0 < self.minimum_high_speed_scale <= 1.0:
            raise ValueError("minimum_high_speed_scale must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class CameraLineCorrectionState:
    running: bool = False
    active: bool = False
    timestamp_s: float = 0.0
    confidence: float = 0.0
    lateral_error_cm: float = 0.0
    correction_rad: float = 0.0
    valid_frames: int = 0
    large_error_frames: int = 0
    curve_mode: bool = False
    recovery_mode: bool = False
    observation: LineObservation | None = None
    error: str | None = None


class CameraLineSteeringCorrector:
    """Capture line observations and expose a bounded steering increment."""

    def __init__(
        self,
        *,
        camera_index: int | str = 0,
        vision_config: LineVisionConfig = LineVisionConfig(),
        correction_config: CameraLineCorrectionConfig = (
            CameraLineCorrectionConfig()
        ),
        detector: BlackLineDetector | None = None,
        on_state_changed: (
            Callable[[CameraLineCorrectionState], None] | None
        ) = None,
    ) -> None:
        self.camera_index = camera_index
        self.vision_config = vision_config
        self.correction_config = correction_config
        self.detector = detector or BlackLineDetector(vision_config)
        self._on_state_changed = on_state_changed

        self._lock = threading.Lock()
        self._state = CameraLineCorrectionState()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._capture = None
        self._valid_frames = 0
        self._large_error_frames = 0
        self._candidate_error_cm: float | None = None
        self._last_accepted_error_cm = 0.0
        self._invalid_grace_frames = 0
        self._curve_mode = False
        self._last_update_s: float | None = None
        self._filtered_correction_rad = 0.0

    @property
    def state(self) -> CameraLineCorrectionState:
        with self._lock:
            return self._state

    @property
    def is_running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def start(self) -> "CameraLineSteeringCorrector":
        self.detector._require_opencv()
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("camera line corrector is already running")
            self._stop_event.clear()
            self._valid_frames = 0
            self._large_error_frames = 0
            self._candidate_error_cm = None
            self._last_accepted_error_cm = 0.0
            self._invalid_grace_frames = 0
            self._last_update_s = None
            self._filtered_correction_rad = 0.0
            self.detector.reset()
            self._state = CameraLineCorrectionState(
                running=True,
                curve_mode=self._curve_mode,
            )
            self._thread = threading.Thread(
                target=self._run,
                name="camera-line-corrector",
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
        with self._lock:
            self._thread = None
            self._valid_frames = 0
            self._large_error_frames = 0
            self._candidate_error_cm = None
            self._last_accepted_error_cm = 0.0
            self._invalid_grace_frames = 0
            self._last_update_s = None
            self._filtered_correction_rad = 0.0
            self._state = replace(
                self._state,
                running=False,
                active=False,
                correction_rad=0.0,
                valid_frames=0,
                large_error_frames=0,
                recovery_mode=False,
            )
            state = self._state
        self._notify(state)

    close = stop

    def set_curve_mode(self, enabled: bool) -> None:
        """Tell the corrector whether radar is currently on a semicircle."""

        with self._lock:
            self._curve_mode = bool(enabled)
            self._state = replace(
                self._state,
                curve_mode=self._curve_mode,
            )

    def update_from_observation(
        self,
        observation: LineObservation,
        *,
        now_s: float | None = None,
    ) -> CameraLineCorrectionState:
        """Update the soft correction without commanding any hardware."""

        now = time.monotonic() if now_s is None else float(now_s)
        dt = (
            1.0 / self.vision_config.camera_fps
            if self._last_update_s is None
            else max(1e-3, min(0.25, now - self._last_update_s))
        )
        self._last_update_s = now
        with self._lock:
            curve_mode = self._curve_mode
            previous_active = self._state.active
        usable, marker_recovery = self._classify_observation(
            observation,
            curve_mode=curve_mode,
        )
        active = False
        recovery_mode = False
        lateral_error = 0.0
        target = 0.0
        if usable:
            lateral_error = float(observation.near_lateral_error_cm)
            previous_error = self._candidate_error_cm
            stable = self._candidate_is_stable(lateral_error)
            self._valid_frames = self._valid_frames + 1 if stable else 1
            large_error = (
                abs(lateral_error)
                >= self.correction_config.large_error_fast_activate_cm
            )
            previous_was_large = (
                previous_error is not None
                and abs(previous_error)
                >= self.correction_config.large_error_fast_activate_cm
            )
            self._large_error_frames = (
                self._large_error_frames + 1
                if large_error and stable and previous_was_large
                else (1 if large_error else 0)
            )
            self._candidate_error_cm = lateral_error
            self._last_accepted_error_cm = lateral_error
            self._invalid_grace_frames = 0
            required_frames = (
                self.correction_config.curve_round_marker_required_frames
                if marker_recovery
                else (
                    self.correction_config.large_error_required_frames
                    if large_error
                    else self.correction_config.required_consecutive_frames
                )
            )
            qualifying_frames = (
                self._large_error_frames
                if large_error
                else self._valid_frames
            )
            active = qualifying_frames >= required_frames
            recovery_mode = active and (large_error or marker_recovery)
            if active:
                target = self._target_correction(
                    lateral_error,
                    maximum_abs_correction_rad=(
                        self.correction_config
                        .curve_round_marker_maximum_abs_correction_rad
                        if marker_recovery
                        else None
                    ),
                )
        else:
            # A real end-line can fill the camera near A.  It must never
            # create a correction, but it may briefly preserve an already
            # established large-error curve correction from the last
            # trustworthy longitudinal-line observation.
            can_hold_large_curve_recovery = (
                curve_mode
                and previous_active
                and abs(self._last_accepted_error_cm)
                >= self.correction_config.large_error_fast_activate_cm
                and self._invalid_grace_frames
                < self.correction_config.curve_invalid_grace_frames
            )
            if can_hold_large_curve_recovery:
                self._invalid_grace_frames += 1
                active = True
                recovery_mode = True
                lateral_error = self._last_accepted_error_cm
                # Keep the last established soft correction steady while the
                # marker/end-line temporarily hides the longitudinal line.
                # Do not let an untrusted frame ramp it toward a larger limit.
                target = self._filtered_correction_rad
            else:
                self._candidate_error_cm = None
                self._valid_frames = 0
                self._large_error_frames = 0
                self._invalid_grace_frames = 0
        self._filtered_correction_rad = self._filter_correction(
            target,
            dt,
        )
        with self._lock:
            self._state = CameraLineCorrectionState(
                running=self._state.running,
                active=active,
                timestamp_s=now,
                confidence=float(observation.confidence),
                lateral_error_cm=lateral_error,
                correction_rad=self._filtered_correction_rad,
                valid_frames=self._valid_frames,
                large_error_frames=self._large_error_frames,
                curve_mode=curve_mode,
                recovery_mode=recovery_mode,
                observation=observation,
                error=None,
            )
            state = self._state
        self._notify(state)
        return state

    def correction_for_speed(
        self,
        speed_cm_s: float,
        *,
        now_s: float | None = None,
    ) -> float:
        """Return the current correction, reduced at high speed and when stale."""

        now = time.monotonic() if now_s is None else float(now_s)
        state = self.state
        age = max(0.0, now - state.timestamp_s)
        correction = float(state.correction_rad)
        if age > self.correction_config.stale_timeout_s:
            fade_age = age - self.correction_config.stale_timeout_s
            stale_scale = max(
                0.0,
                1.0
                - fade_age / self.correction_config.stale_fade_out_s,
            )
            correction *= stale_scale
        speed = max(0.0, float(speed_cm_s))
        if speed > self.correction_config.full_correction_speed_cm_s:
            speed_scale = max(
                self.correction_config.minimum_high_speed_scale,
                self.correction_config.full_correction_speed_cm_s
                / speed,
            )
            correction *= speed_scale
        return float(
            np.clip(
                correction,
                -self.correction_config.maximum_abs_correction_rad,
                self.correction_config.maximum_abs_correction_rad,
            )
        )

    def _classify_observation(
        self,
        observation: LineObservation,
        *,
        curve_mode: bool,
    ) -> tuple[bool, bool]:
        config = self.correction_config
        structurally_valid = (
            observation.detected
            and math.isfinite(observation.near_lateral_error_cm)
            and not observation.transverse_line_detected
        )
        if not structurally_valid:
            return False, False
        basic_quality = (
            observation.confidence >= config.minimum_confidence
            and observation.visible_band_count
            >= config.minimum_visible_bands
            and observation.fit_rmse_cm <= config.maximum_fit_rmse_cm
        )
        if not observation.round_marker_detected:
            return basic_quality, False
        marker_recovery = (
            curve_mode
            and abs(observation.near_lateral_error_cm)
            >= config.large_error_fast_activate_cm
            and observation.confidence
            >= config.curve_round_marker_minimum_confidence
            and observation.visible_band_count
            >= config.curve_round_marker_minimum_visible_bands
            and observation.fit_rmse_cm
            <= config.curve_round_marker_maximum_fit_rmse_cm
        )
        return marker_recovery, marker_recovery

    def _candidate_is_stable(self, lateral_error_cm: float) -> bool:
        previous = self._candidate_error_cm
        if previous is None:
            return False
        same_side = (
            previous * lateral_error_cm > 0.0
            or (
                abs(previous) <= self.correction_config.lateral_deadband_cm
                and abs(lateral_error_cm)
                <= self.correction_config.lateral_deadband_cm
            )
        )
        small_step = (
            abs(lateral_error_cm - previous)
            <= self.correction_config.large_error_max_step_cm
        )
        return same_side and small_step

    def _target_correction(
        self,
        lateral_error_cm: float,
        *,
        maximum_abs_correction_rad: float | None = None,
    ) -> float:
        magnitude = max(
            0.0,
            abs(float(lateral_error_cm))
            - self.correction_config.lateral_deadband_cm,
        )
        requested = math.copysign(
            self.correction_config.steering_gain_rad_per_cm * magnitude,
            lateral_error_cm,
        )
        limit = (
            self.correction_config.maximum_abs_correction_rad
            if maximum_abs_correction_rad is None
            else min(
                self.correction_config.maximum_abs_correction_rad,
                float(maximum_abs_correction_rad),
            )
        )
        return float(
            np.clip(
                requested,
                -limit,
                limit,
            )
        )

    def _filter_correction(self, requested: float, dt: float) -> float:
        tau = self.correction_config.correction_filter_time_constant_s
        alpha = (
            1.0
            if tau <= 0.0
            else 1.0 - math.exp(-max(0.0, dt) / tau)
        )
        low_passed = self._filtered_correction_rad + alpha * (
            requested - self._filtered_correction_rad
        )
        maximum_delta = (
            self.correction_config.maximum_correction_rate_rad_s
            * max(0.0, dt)
        )
        return float(
            np.clip(
                low_passed,
                self._filtered_correction_rad - maximum_delta,
                self._filtered_correction_rad + maximum_delta,
            )
        )

    def _run(self) -> None:
        capture_failures = 0
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
                    self._decay_with_error(
                        now,
                        "camera frame capture failed",
                    )
                    if (
                        capture_failures
                        >= self.vision_config.max_consecutive_capture_failures
                    ):
                        raise RuntimeError(
                            "camera repeatedly failed to return frames"
                        )
                    time.sleep(0.03)
                    continue
                capture_failures = 0
                observation = self.detector.process(
                    frame,
                    timestamp_s=now,
                )
                self.update_from_observation(observation, now_s=now)
        except BaseException as exc:
            LOG.exception("camera line correction stopped")
            self._decay_with_error(time.monotonic(), str(exc))
        finally:
            capture, self._capture = self._capture, None
            if capture is not None:
                capture.release()
            with self._lock:
                self._state = replace(
                    self._state,
                    running=False,
                    active=False,
                    recovery_mode=False,
                )
                state = self._state
            self._notify(state)

    def _decay_with_error(self, now_s: float, error: str) -> None:
        dt = (
            1.0 / self.vision_config.camera_fps
            if self._last_update_s is None
            else max(1e-3, min(0.25, now_s - self._last_update_s))
        )
        self._last_update_s = now_s
        self._valid_frames = 0
        self._large_error_frames = 0
        self._candidate_error_cm = None
        self._invalid_grace_frames = 0
        self._filtered_correction_rad = self._filter_correction(0.0, dt)
        with self._lock:
            self._state = replace(
                self._state,
                active=False,
                timestamp_s=now_s,
                correction_rad=self._filtered_correction_rad,
                valid_frames=0,
                large_error_frames=0,
                recovery_mode=False,
                error=error,
            )
            state = self._state
        self._notify(state)

    def _open_capture(self):
        cv2 = line_module.cv2
        if cv2 is None:
            raise RuntimeError("OpenCV is required for camera correction")
        backend = (
            cv2.CAP_V4L2
            if self.vision_config.capture_backend_v4l2
            and hasattr(cv2, "CAP_V4L2")
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

    def _notify(self, state: CameraLineCorrectionState) -> None:
        if self._on_state_changed is None:
            return
        try:
            self._on_state_changed(state)
        except Exception:
            LOG.exception("camera correction state callback failed")


__all__ = [
    "CameraLineCorrectionConfig",
    "CameraLineCorrectionState",
    "CameraLineSteeringCorrector",
]
