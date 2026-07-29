#!/usr/bin/env python3
"""Run the front-mounted camera black-line follower on the real car.

The camera is mounted at an approximately 60-degree downward pitch.  Before a
real run, calibrate the perspective trapezoid in ``LineVisionConfig`` against
the actual camera position and line surface.  This entry point deliberately
does not open D500 radar or HC-14: it is a camera-only fixed-track test.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import logging
from pathlib import Path
import signal
import sys
import threading
import time

from components import (
    AckermannDrive,
    CameraLineFollower,
    LineControlConfig,
    LineFollowerState,
    LineFollowerStatus,
    LineVisionConfig,
    PerspectiveConfig,
)
from main_fixed_track_test import (
    configure_logging,
    default_log_dir,
    shutdown_logging,
)


# Per-segment fixed-track speeds.  AB/CD are the 150 cm straights; BC/DA are
# the two 75 cm-radius semicircles.
TRACK_AB_SPEED_CM_S = 20.0
TRACK_BC_SPEED_CM_S = 20.0
TRACK_CD_SPEED_CM_S = 20.0
TRACK_DA_SPEED_CM_S = 20.0

# Backward-compatible uniform-speed name and CLI override.
TRACK_SPEED_CM_S = TRACK_AB_SPEED_CM_S

# At startup the car faces AB with its front reference at A. The radar centre
# is this far behind A; after driving forward this distance, it passes A.
RADAR_CENTER_BEHIND_A_ALONG_AB_CM = 0.0


LOG = logging.getLogger("camera-line-main")


def _camera_source(value: str) -> int | str:
    """Accept either a numeric camera index or a V4L2 device path."""

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
class CameraLineMainConfig:
    camera_source: int | str = 0
    ab_speed_cm_s: float = TRACK_AB_SPEED_CM_S
    bc_speed_cm_s: float = TRACK_BC_SPEED_CM_S
    cd_speed_cm_s: float = TRACK_CD_SPEED_CM_S
    da_speed_cm_s: float = TRACK_DA_SPEED_CM_S
    uniform_speed_cm_s: float | None = None
    radar_center_behind_a_cm: float = RADAR_CENTER_BEHIND_A_ALONG_AB_CM

    def __post_init__(self) -> None:
        for name, value in zip(
            ("ab", "bc", "cd", "da"),
            self.segment_speeds_cm_s,
        ):
            if not 0.0 < value <= 100.0:
                raise ValueError(f"{name}_speed_cm_s must be in (0, 100]")
        if self.radar_center_behind_a_cm < 0.0:
            raise ValueError("radar_center_behind_a_cm cannot be negative")
        if isinstance(self.camera_source, int) and self.camera_source < 0:
            raise ValueError("camera_source cannot be negative")
        if isinstance(self.camera_source, str) and not self.camera_source:
            raise ValueError("camera_source cannot be empty")

    @property
    def segment_speeds_cm_s(self) -> tuple[float, float, float, float]:
        if self.uniform_speed_cm_s is not None:
            speed = float(self.uniform_speed_cm_s)
            return (speed, speed, speed, speed)
        return (
            float(self.ab_speed_cm_s),
            float(self.bc_speed_cm_s),
            float(self.cd_speed_cm_s),
            float(self.da_speed_cm_s),
        )


class CameraLineApplication:
    """Own the camera follower and guarantee a safe output on every exit."""

    def __init__(self, config: CameraLineMainConfig) -> None:
        self.config = config
        self._stop_event = threading.Event()
        self._closed = False
        self._last_state = LineFollowerState()
        self._segment_names = ("AB", "BC", "CD", "DA")
        self._segment_speeds_cm_s = config.segment_speeds_cm_s
        self._segment_index = 0

        # Match the main-program vehicle limit convention: leave 20% room for
        # the outside rear wheel during an Ackermann turn, with a 30 cm/s floor.
        max_wheel_speed_mm_s = max(
            300.0,
            max(self._segment_speeds_cm_s) * 12.0,
        )
        self.drive = AckermannDrive(max_wheel_speed_mm_s=max_wheel_speed_mm_s)
        self.follower = CameraLineFollower(
            drive=self.drive,
            camera_index=config.camera_source,
            vision_config=self._front_camera_vision_config(),
            control_config=self._control_config_for_speed(
                self._segment_speeds_cm_s[0]
            ),
            on_state_changed=self._on_follower_state,
            on_marker_passed=self._on_marker_passed,
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

    @staticmethod
    def _control_config_for_speed(speed_cm_s: float) -> LineControlConfig:
        """Scale the loss/recovery speeds with the one editable cruise speed."""

        speed_mm_s = float(speed_cm_s) * 10.0
        defaults = LineControlConfig()
        scale = speed_mm_s / defaults.cruise_speed_mm_s
        return replace(
            defaults,
            cruise_speed_mm_s=speed_mm_s,
            degraded_speed_mm_s=defaults.degraded_speed_mm_s * scale,
            short_loss_speed_mm_s=defaults.short_loss_speed_mm_s * scale,
            minimum_tracking_speed_mm_s=defaults.minimum_tracking_speed_mm_s * scale,
            minimum_lookahead_cm=38.0,
            maximum_lookahead_cm=68.0,
            lookahead_speed_gain_s=0.40,
            latency_compensation_s=0.14,
            pure_pursuit_gain=1.0,
            lateral_gain=0.04,
            heading_gain=0.0,
            curvature_feedforward_gain=0.0,
            maximum_abs_steering_rad=0.26,
            maximum_steering_rate_rad_s=0.85,
            steering_low_pass_time_constant_s=0.12,
            steering_deadband_rad=0.010,
            target_filter_time_constant_s=0.10,
            target_filter_max_rate_cm_s=140.0,
            heading_filter_time_constant_s=0.14,
            heading_filter_max_rate_rad_s=2.2,
            curvature_slowdown_start_rad=0.105,
            curvature_full_slowdown_rad=0.49,
            curvature_speed_gain=1.0,
            maximum_acceleration_mm_s2=450.0,
            maximum_deceleration_mm_s2=1000.0,
            recovery_good_frames=3,
            short_loss_frames=1,
            finish_line_confirm_frames=3,
            minimum_markers_before_finish=3,
            round_marker_clear_frames_to_arm=5,
            round_marker_confirm_frames=3,
        )

    @property
    def state(self) -> LineFollowerState:
        return self._last_state

    def run(self) -> None:
        LOG.info(
            "camera line test starting source=%r speeds_cm_s=%s "
            "radar_center_behind_a_cm=%.1f",
            self.config.camera_source,
            self._segment_speeds_cm_s,
            self.config.radar_center_behind_a_cm,
        )
        LOG.info(
            "camera-only mode: D500 and HC-14 are not opened; keep the car "
            "stationary until a valid black line is visible"
        )
        try:
            self.drive.start()
            self.follower.start()
            while not self._stop_event.wait(0.10):
                state = self.follower.state
                if (
                    state.status is LineFollowerStatus.CAMERA_ERROR
                    and not state.running
                ):
                    raise RuntimeError(state.error or "camera line follower failed")
                if state.status is LineFollowerStatus.FINISHED:
                    LOG.info("finish line reached; camera-only test complete")
                    return
        finally:
            self.close()

    def request_stop(self) -> None:
        self._stop_event.set()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        LOG.info("closing camera follower")
        try:
            self.follower.close()
        finally:
            LOG.info("closing Ackermann drive")
            self.drive.close()
        LOG.info("camera line application closed; vehicle stopped and centred")

    def _on_follower_state(self, state: LineFollowerState) -> None:
        previous_status = self._last_state.status
        self._last_state = state
        if (
            state.status is LineFollowerStatus.LOST
            and previous_status is not LineFollowerStatus.LOST
        ):
            LOG.warning("line lost; drive stopped until the line is reacquired")
        elif state.status is LineFollowerStatus.FINISHED:
            LOG.info("transverse finish line confirmed; vehicle stopped")
        elif state.status is LineFollowerStatus.CAMERA_ERROR:
            LOG.error("camera follower error: %s", state.error)
        else:
            observation = state.observation
            LOG.debug(
                "line state=%s segment=%s marker_count=%d confidence=%.2f "
                "speed_mm_s=%.1f steering_rad=%.3f lateral_cm=%.2f "
                "heading_rad=%.3f curvature_cm_inv=%.5f "
                "forward_heading_change_rad=%.3f round=%s transverse=%s",
                state.status.value,
                self._segment_names[self._segment_index],
                state.marker_count,
                state.confidence,
                state.speed_mm_s,
                state.steering_angle_rad,
                0.0 if observation is None else observation.near_lateral_error_cm,
                0.0 if observation is None else observation.heading_error_rad,
                0.0 if observation is None else observation.curvature_per_cm,
                (
                    0.0
                    if observation is None
                    else observation.forward_heading_change_rad
                ),
                False if observation is None else observation.round_marker_detected,
                False if observation is None else observation.transverse_line_detected,
            )

    def _on_marker_passed(self, marker_count: int) -> None:
        if 1 <= marker_count <= 3:
            self._segment_index = marker_count
            speed_cm_s = self._segment_speeds_cm_s[self._segment_index]
            self.follower.set_cruise_speed_mm_s(speed_cm_s * 10.0)
            LOG.info(
                "track segment changed to %s speed_cm_s=%.1f marker_count=%d",
                self._segment_names[self._segment_index],
                speed_cm_s,
                marker_count,
            )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=_camera_source, default=0)
    parser.add_argument(
        "--speed-cm-s",
        type=float,
        default=None,
        help="optional uniform override for all four segment speeds",
    )
    parser.add_argument("--ab-speed-cm-s", type=float, default=TRACK_AB_SPEED_CM_S)
    parser.add_argument("--bc-speed-cm-s", type=float, default=TRACK_BC_SPEED_CM_S)
    parser.add_argument("--cd-speed-cm-s", type=float, default=TRACK_CD_SPEED_CM_S)
    parser.add_argument("--da-speed-cm-s", type=float, default=TRACK_DA_SPEED_CM_S)
    parser.add_argument(
        "--radar-center-behind-a-cm",
        type=float,
        default=RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
        help="startup reference metadata only; camera-only control does not use D500",
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
    requested_log_dir = default_log_dir() if args.log_dir is None else Path(args.log_dir)
    try:
        configure_logging(requested_log_dir, args.log_level)
    except OSError as exc:
        print(f"cannot create detailed log in {requested_log_dir}: {exc}", file=sys.stderr)
        return 2

    app: CameraLineApplication | None = None
    try:
        app = CameraLineApplication(
            CameraLineMainConfig(
                camera_source=args.camera,
                ab_speed_cm_s=args.ab_speed_cm_s,
                bc_speed_cm_s=args.bc_speed_cm_s,
                cd_speed_cm_s=args.cd_speed_cm_s,
                da_speed_cm_s=args.da_speed_cm_s,
                uniform_speed_cm_s=args.speed_cm_s,
                radar_center_behind_a_cm=args.radar_center_behind_a_cm,
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
        LOG.exception("camera line main failed")
        return 1
    finally:
        if app is not None:
            app.close()
        shutdown_logging()


if __name__ == "__main__":
    raise SystemExit(main())
