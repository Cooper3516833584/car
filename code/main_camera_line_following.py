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
)
from main_fixed_track_test import (
    configure_logging,
    default_log_dir,
    shutdown_logging,
)


# Fixed-track test speed. Change this one value for the next real-car run.
TRACK_SPEED_CM_S = 40.0

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
    speed_cm_s: float = TRACK_SPEED_CM_S
    radar_center_behind_a_cm: float = RADAR_CENTER_BEHIND_A_ALONG_AB_CM

    def __post_init__(self) -> None:
        if not 0.0 < self.speed_cm_s <= 100.0:
            raise ValueError("speed_cm_s must be in (0, 100]")
        if self.radar_center_behind_a_cm < 0.0:
            raise ValueError("radar_center_behind_a_cm cannot be negative")
        if isinstance(self.camera_source, int) and self.camera_source < 0:
            raise ValueError("camera_source cannot be negative")
        if isinstance(self.camera_source, str) and not self.camera_source:
            raise ValueError("camera_source cannot be empty")


class CameraLineApplication:
    """Own the camera follower and guarantee a safe output on every exit."""

    def __init__(self, config: CameraLineMainConfig) -> None:
        self.config = config
        self._stop_event = threading.Event()
        self._closed = False
        self._last_state = LineFollowerState()

        # Match the main-program vehicle limit convention: leave 20% room for
        # the outside rear wheel during an Ackermann turn, with a 30 cm/s floor.
        max_wheel_speed_mm_s = max(300.0, config.speed_cm_s * 12.0)
        self.drive = AckermannDrive(max_wheel_speed_mm_s=max_wheel_speed_mm_s)
        self.follower = CameraLineFollower(
            drive=self.drive,
            camera_index=config.camera_source,
            control_config=self._control_config_for_speed(config.speed_cm_s),
            on_state_changed=self._on_follower_state,
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
        )

    @property
    def state(self) -> LineFollowerState:
        return self._last_state

    def run(self) -> None:
        LOG.info(
            "camera line test starting source=%r speed_cm_s=%.1f "
            "radar_center_behind_a_cm=%.1f",
            self.config.camera_source,
            self.config.speed_cm_s,
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
        try:
            self.follower.close()
        finally:
            self.drive.close()
        LOG.info("camera line application closed; vehicle stopped and centred")

    def _on_follower_state(self, state: LineFollowerState) -> None:
        self._last_state = state
        if state.status is LineFollowerStatus.LOST:
            LOG.warning("line lost; drive stopped until the line is reacquired")
        elif state.status is LineFollowerStatus.FINISHED:
            LOG.info("transverse finish line confirmed; vehicle stopped")
        elif state.status is LineFollowerStatus.CAMERA_ERROR:
            LOG.error("camera follower error: %s", state.error)
        else:
            LOG.debug(
                "line state=%s confidence=%.2f speed_mm_s=%.1f steering_rad=%.3f",
                state.status.value,
                state.confidence,
                state.speed_mm_s,
                state.steering_angle_rad,
            )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=_camera_source, default=0)
    parser.add_argument("--speed-cm-s", type=float, default=TRACK_SPEED_CM_S)
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
                speed_cm_s=args.speed_cm_s,
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
