#!/usr/bin/env python3
"""Use D500 localization to follow the fixed track from A for one lap."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
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
    DEFAULT_D500_PORT,
    D500RadarComponent,
    Pose2D,
    RadarLocalizationUpdate,
    RadarMount,
    RadarScan,
    RectangleFieldCalibrator,
    TrackFollowerState,
    WallFusionConfig,
    WallLineConfig,
    rebase_calibration_to_start_pose,
)
from components.navigation import (
    NavigationPose,
    NavigationState,
    radar_yaw_to_navigation_heading,
)
from components.steering_servo import (
    DEFAULT_STEERING_CALIBRATION,
    FrontSteeringServo,
    SteeringCalibration,
)
from components.vehicle_defaults import (
    DEFAULT_FIRMWARE_TRACK_WIDTH_MM,
    DEFAULT_MIN_TURN_RADIUS_MM,
    DEFAULT_PHYSICAL_TRACK_WIDTH_MM,
    DEFAULT_WHEELBASE_MM,
)
from config.factory import build_steering_calibration, build_steering_servo
from config.loader import load_car_config


# Fixed-track test speed. Change this one value for the next real-car run.
TRACK_SPEED_CM_S = 30.0

# At startup the car faces AB with its front reference at A. The radar centre
# is this far behind A; after driving forward this distance, it passes A.
RADAR_CENTER_BEHIND_A_ALONG_AB_CM = 20.0

LOG = logging.getLogger("car-main")
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


@dataclass(frozen=True, slots=True)
class MainConfig:
    radar_port: str = DEFAULT_D500_PORT
    radar_mount: RadarMount = RadarMount()
    startup_scan_count: int = 3
    calibration_timeout_s: float = 30.0
    radar_center_behind_a_cm: float = RADAR_CENTER_BEHIND_A_ALONG_AB_CM
    speed_cm_s: float = TRACK_SPEED_CM_S
    # Drive construction data; the rollback entry fills these from the TOML
    # profile so the car keeps working without hardcoded devices.
    motor_device: str = ""
    wheelbase_mm: float = DEFAULT_WHEELBASE_MM
    physical_track_width_mm: float = DEFAULT_PHYSICAL_TRACK_WIDTH_MM
    firmware_track_width_mm: float = DEFAULT_FIRMWARE_TRACK_WIDTH_MM
    min_turn_radius_mm: float = DEFAULT_MIN_TURN_RADIUS_MM
    allow_in_place_rotation: bool = False
    steering_calibration: SteeringCalibration = DEFAULT_STEERING_CALIBRATION
    # Ready-made steering servo carrying the configured PWM HAL; the rollback
    # entry fills it from the TOML profile so drive.start() works on hardware.
    steering_servo: FrontSteeringServo | None = None

    def __post_init__(self) -> None:
        if self.startup_scan_count <= 0:
            raise ValueError("startup_scan_count must be positive")
        if self.calibration_timeout_s <= 0.0:
            raise ValueError("calibration_timeout_s must be positive")
        if self.radar_center_behind_a_cm < 0.0:
            raise ValueError("radar_center_behind_a_cm cannot be negative")
        if self.speed_cm_s <= 0.0:
            raise ValueError("speed_cm_s must be positive")


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


class CompetitionCarApplication:
    """Calibrate once, run one radar-driven lap, then stop."""

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

        max_wheel_speed_mm_s = max(300.0, config.speed_cm_s * 12.0)
        self.drive = AckermannDrive.from_config(
            device=config.motor_device or None,
            wheelbase_mm=config.wheelbase_mm,
            track_width_mm=config.physical_track_width_mm,
            firmware_track_width_mm=config.firmware_track_width_mm,
            max_wheel_speed_mm_s=max_wheel_speed_mm_s,
            min_turn_radius_mm=config.min_turn_radius_mm,
            allow_in_place_rotation=config.allow_in_place_rotation,
            steering_calibration=config.steering_calibration,
            steering=config.steering_servo,
        )
        self.track = CompetitionTrack.build(
            reference_offset_cm=config.radar_center_behind_a_cm,
        )
        self.follower = CompetitionTrackFollower(
            drive=self.drive,
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

    def run(self) -> None:
        LOG.info(
            "vehicle must remain stationary at A during D500 calibration"
        )
        try:
            self._calibrate_radar()
            self.drive.start()
            self.follower.start_mission()
            self.radar.set_motion_hint(True)
            with self._lock:
                self._ready = True
            self.radar.start()
            LOG.info(
                "one-lap tracking started speed_cm_s=%.1f "
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
            "calibration complete; startup rear axle=(0,0,0deg), "
            "A is %.1f cm ahead; "
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
        with self._lock:
            self._follower_state = state
        if state.completed:
            self.radar.set_motion_hint(False)
            self._completed_event.set()
            LOG.info("one lap complete; vehicle stopped at A")

    def request_stop(self) -> None:
        self._stop_event.set()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._ready = False
            self._map_ready = False
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
        help="vehicle TOML profile (default: CAR_CONFIG env or the repository "
        "default profile); used for the motor device and drive geometry",
    )
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

    app: CompetitionCarApplication | None = None
    try:
        car_config = load_car_config(args.config)
        app = CompetitionCarApplication(
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
                motor_device=car_config.devices.motor.port,
                wheelbase_mm=car_config.vehicle.geometry.wheelbase_mm,
                physical_track_width_mm=(
                    car_config.vehicle.geometry.physical_track_width_mm
                ),
                firmware_track_width_mm=(
                    car_config.vehicle.drive.firmware_track_width_mm
                ),
                min_turn_radius_mm=(
                    car_config.vehicle.drive.min_turn_radius_mm
                ),
                allow_in_place_rotation=(
                    car_config.vehicle.drive.allow_in_place_rotation
                ),
                steering_calibration=build_steering_calibration(car_config),
                steering_servo=build_steering_servo(car_config),
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
        LOG.exception("car main failed")
        return 1
    finally:
        if app is not None:
            app.close()
        shutdown_logging()


if __name__ == "__main__":
    raise SystemExit(main())
