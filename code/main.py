#!/usr/bin/env python3
"""FleetBus-triggered point-and-return mission for the road-patrol task."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import Enum
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import signal
import threading
import time
from typing import Callable

from components import (
    DEFAULT_D500_PORT,
    DEFAULT_HC14_PORT,
    CoordinateNavigation,
    CoordinateNavigationConfig,
    NavigationState,
    RadarMount,
    SerialCommunicationDriver,
)
from components.fleet_car_node import FleetCarNode
from components.fleet_models import (
    AckReason,
    AckStatus,
    CarFleetState,
    CommandResult,
    NodeFlags,
)
from fleet_bus.models import CarOperationState


# Competition-site tuning: edit this one point when the final coordinate is known.
TARGET_X_CM = 100.0
TARGET_Y_CM = 0.0
TARGET_HOLD_SECONDS = 3.0
RETURN_X_CM = 0.0
RETURN_Y_CM = 0.0

NAVIGATION_CRUISE_SPEED_CM_S = 30.0
NAVIGATION_REVERSE_SPEED_CM_S = 15.0
MISSION_LEG_TIMEOUT_S = 120.0
FLEET_POSITION_STALE_TIMEOUT_S = 0.5
LOG_FILENAME = "car-main.log"
LOG_MAX_BYTES = 20 * 1024 * 1024
LOG_BACKUP_COUNT = 10

LOG = logging.getLogger("road-patrol-car")


class MissionPhase(Enum):
    STARTING = "starting"
    CALIBRATING = "calibrating"
    READY = "ready"
    OUTBOUND = "outbound"
    HOLDING = "holding"
    RETURNING = "returning"
    COMPLETED = "completed"
    FAILED = "failed"
    CLOSED = "closed"


@dataclass(frozen=True, slots=True)
class MainConfig:
    radar_port: str = DEFAULT_D500_PORT
    fleet_port: str = DEFAULT_HC14_PORT
    radar_mount: RadarMount = RadarMount()
    startup_scan_count: int = 20
    calibration_timeout_s: float = 30.0
    target_x_cm: float = TARGET_X_CM
    target_y_cm: float = TARGET_Y_CM
    hold_seconds: float = TARGET_HOLD_SECONDS
    leg_timeout_s: float = MISSION_LEG_TIMEOUT_S

    def __post_init__(self) -> None:
        if not self.radar_port or not self.fleet_port:
            raise ValueError("radar and FleetBus ports cannot be empty")
        if self.startup_scan_count <= 0 or self.calibration_timeout_s <= 0.0:
            raise ValueError("startup scan count and calibration timeout must be positive")
        if self.hold_seconds < 0.0 or self.leg_timeout_s <= 0.0:
            raise ValueError("mission timing is invalid")


def execute_point_return_mission(
    navigate_leg: Callable[[float, float, MissionPhase], bool],
    wait: Callable[[float], bool],
    *,
    target_x_cm: float = TARGET_X_CM,
    target_y_cm: float = TARGET_Y_CM,
    hold_seconds: float = TARGET_HOLD_SECONDS,
) -> bool:
    """Run the task route; ``True`` means both coordinate legs arrived."""

    if not navigate_leg(target_x_cm, target_y_cm, MissionPhase.OUTBOUND):
        return False
    if wait(hold_seconds):
        return False
    return navigate_leg(RETURN_X_CM, RETURN_Y_CM, MissionPhase.RETURNING)


class RoadPatrolCarApplication:
    """Own calibration, FleetBus, navigation and safe shutdown."""

    def __init__(self, config: MainConfig) -> None:
        self.config = config
        self._lock = threading.RLock()
        self._navigation_condition = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._mission_start_event = threading.Event()
        self._started_s = time.monotonic()
        self._phase = MissionPhase.STARTING
        self._navigation_state = NavigationState.IDLE
        self._navigation_reason = "not started"
        self._terminal_generation = 0
        self._ready = False
        self._closed = False
        self._error_code = 0

        self.navigation = CoordinateNavigation(
            CoordinateNavigationConfig(
                radar_port=config.radar_port,
                radar_mount=config.radar_mount,
                startup_scan_count=config.startup_scan_count,
                calibration_timeout_s=config.calibration_timeout_s,
                allow_reverse=True,
                cruise_speed_cm_s=NAVIGATION_CRUISE_SPEED_CM_S,
                reverse_speed_cm_s=NAVIGATION_REVERSE_SPEED_CM_S,
            ),
            on_state_changed=self._on_navigation_state,
        )
        self.fleet_link = SerialCommunicationDriver(
            port=config.fleet_port,
            on_bytes=self._on_fleet_frame,
            on_connected=lambda: LOG.info(
                "FleetBus HC-14 connected on %s", config.fleet_port
            ),
            on_disconnected=lambda error: LOG.warning(
                "FleetBus HC-14 disconnected: %s", error
            ),
            on_callback_error=lambda error: LOG.error(
                "FleetBus callback failed: %s", error
            ),
        )
        self.fleet_node = FleetCarNode(
            writer=self._send_fleet_frame,
            state_provider=self._fleet_state,
            on_set_coordinate_frame=self._fleet_unsupported,
            on_navigate=self._fleet_unsupported,
            on_stop=self._fleet_stop,
            on_start_mission=self._fleet_start_mission,
        )

    def run(self) -> bool:
        LOG.info("vehicle must remain stationary during D500 calibration")
        self.fleet_node.start()
        self.fleet_link.start()
        try:
            self._set_phase(MissionPhase.CALIBRATING)
            self.navigation.start()
            with self._lock:
                self._ready = True
            self._set_phase(MissionPhase.READY)
            LOG.info(
                "ready; waiting for FleetBus CAR_START_MISSION target=(%.1f, %.1f)cm",
                self.config.target_x_cm,
                self.config.target_y_cm,
            )
            while (
                not self._stop_event.is_set()
                and not self._mission_start_event.wait(0.2)
            ):
                pass
            if self._stop_event.is_set():
                return False

            succeeded = execute_point_return_mission(
                self._navigate_leg,
                self._stop_event.wait,
                target_x_cm=self.config.target_x_cm,
                target_y_cm=self.config.target_y_cm,
                hold_seconds=self.config.hold_seconds,
            )
            if succeeded:
                self._set_phase(MissionPhase.COMPLETED)
                LOG.info(
                    "mission complete; returned to (0,0) without heading constraint"
                )
            elif not self._stop_event.is_set():
                self._set_failed("point-and-return mission did not complete")

            while not self._stop_event.wait(0.5):
                pass
            return succeeded
        finally:
            self.close()

    def request_stop(self) -> None:
        self._stop_event.set()
        self._mission_start_event.set()
        self.navigation.request_stop()
        with self._navigation_condition:
            self._navigation_condition.notify_all()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._ready = False
            self._phase = MissionPhase.CLOSED
        self._stop_event.set()
        try:
            self.navigation.cancel(reason="road-patrol car closing")
        except Exception:
            LOG.exception("failed to cancel active navigation during close")
        self.fleet_node.close()
        self.fleet_link.close()
        self.navigation.close()
        LOG.info("application closed; drive outputs are stopped and steering is centred")

    def _navigate_leg(self, x_cm: float, y_cm: float, phase: MissionPhase) -> bool:
        self._set_phase(phase)
        with self._navigation_condition:
            generation = self._terminal_generation
        try:
            self.navigation.navigate_to(x_cm, y_cm, None)
        except Exception as exc:
            self._set_failed(f"goal ({x_cm:.1f},{y_cm:.1f}) rejected: {exc}")
            return False

        deadline = time.monotonic() + self.config.leg_timeout_s
        with self._navigation_condition:
            while self._terminal_generation == generation:
                if self._stop_event.is_set():
                    self.navigation.cancel(reason="mission stop requested")
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self.navigation.cancel(reason="mission leg timeout")
                    self._set_failed(
                        f"goal ({x_cm:.1f},{y_cm:.1f}) timed out"
                    )
                    return False
                self._navigation_condition.wait(min(0.2, remaining))
            arrived = self._navigation_state is NavigationState.ARRIVED

        if arrived and phase is MissionPhase.OUTBOUND:
            self._set_phase(MissionPhase.HOLDING)
            LOG.info(
                "target reached; holding stopped for %.1f seconds",
                self.config.hold_seconds,
            )
        elif not arrived:
            self._set_failed(
                f"navigation ended in {self._navigation_state.value}: "
                f"{self._navigation_reason}"
            )
        return arrived

    def _on_navigation_state(self, state: NavigationState, reason: str) -> None:
        terminal = state in {
            NavigationState.ARRIVED,
            NavigationState.BLOCKED,
            NavigationState.FAILED,
            NavigationState.LOCALIZATION_LOST,
            NavigationState.CLOSED,
        }
        with self._navigation_condition:
            self._navigation_state = state
            self._navigation_reason = reason
            if terminal:
                self._terminal_generation += 1
                self._navigation_condition.notify_all()

    def _set_phase(self, phase: MissionPhase) -> None:
        with self._lock:
            self._phase = phase
        LOG.info("mission phase=%s", phase.value)

    def _set_failed(self, reason: str) -> None:
        with self._lock:
            self._phase = MissionPhase.FAILED
            self._error_code = int(AckReason.INTERNAL_ERROR)
        LOG.error("mission failed: %s", reason)

    def _on_fleet_frame(self, frame: bytes) -> None:
        self.fleet_node.feed_frame(frame)

    def _send_fleet_frame(self, frame: bytes) -> None:
        try:
            self.fleet_link.write(frame)
        except Exception as exc:
            LOG.warning("FleetBus reply could not be sent: %s", exc)

    def _fleet_start_mission(self) -> CommandResult:
        with self._lock:
            if not self._ready:
                return CommandResult(
                    AckStatus.REJECTED,
                    AckReason.NOT_READY,
                    "D500 calibration is not complete",
                )
            if self._phase is MissionPhase.FAILED:
                return CommandResult(
                    AckStatus.FAILED,
                    AckReason.INTERNAL_ERROR,
                    "car mission is in failed state",
                )
            self._mission_start_event.set()
        return CommandResult(AckStatus.COMPLETED)

    def _fleet_stop(self) -> CommandResult:
        self.request_stop()
        return CommandResult(AckStatus.COMPLETED)

    @staticmethod
    def _fleet_unsupported(*_args) -> CommandResult:
        return CommandResult(AckStatus.REJECTED, AckReason.UNSUPPORTED)

    def _fleet_state(self) -> CarFleetState:
        now = time.monotonic()
        with self._lock:
            phase = self._phase
            ready = self._ready
            navigation_state = self._navigation_state
            error_code = self._error_code
        pose = self.navigation.pose
        pose_valid = bool(
            ready
            and pose is not None
            and now - pose.timestamp_s <= FLEET_POSITION_STALE_TIMEOUT_S
        )
        flags = 0
        if pose_valid:
            flags |= int(NodeFlags.POSE_VALID)
        if ready:
            flags |= int(NodeFlags.READY | NodeFlags.MAP_READY)
        if phase in (MissionPhase.OUTBOUND, MissionPhase.RETURNING):
            flags |= int(NodeFlags.BUSY | NodeFlags.ARMED_OR_MOTOR_ACTIVE)
        if ready and not pose_valid:
            flags |= int(NodeFlags.LOCALIZATION_DEGRADED)

        x_cm = round(pose.x_cm) if pose is not None else 0
        y_cm = round(pose.y_cm) if pose is not None else 0
        heading_cdeg = (
            round(pose.heading_deg * 100.0) % 36000 if pose is not None else 0
        )
        return CarFleetState(
            node_flags=flags,
            uptime_ms=round((now - self._started_s) * 1000.0) & 0xFFFFFFFF,
            x_cm=x_cm,
            y_cm=y_cm,
            heading_cdeg=heading_cdeg,
            operation_state=int(_car_operation_state(phase, navigation_state)),
            pose_quality=4 if pose_valid else (2 if pose is not None else 0),
            error_code=error_code,
            map_revision=self.navigation.navigation.map_revision,
        )


def _car_operation_state(
    phase: MissionPhase,
    navigation_state: NavigationState,
) -> CarOperationState:
    if phase is MissionPhase.CALIBRATING:
        return CarOperationState.CALIBRATING
    if phase is MissionPhase.READY:
        return CarOperationState.READY
    if phase in (MissionPhase.HOLDING, MissionPhase.COMPLETED):
        return CarOperationState.ARRIVED
    if phase is MissionPhase.FAILED:
        if navigation_state is NavigationState.BLOCKED:
            return CarOperationState.BLOCKED
        if navigation_state is NavigationState.LOCALIZATION_LOST:
            return CarOperationState.LOCALIZATION_LOST
        return CarOperationState.FAILED
    if phase is MissionPhase.CLOSED:
        return CarOperationState.CLOSED
    if phase in (MissionPhase.OUTBOUND, MissionPhase.RETURNING):
        mapping = {
            NavigationState.PLANNING: CarOperationState.PLANNING,
            NavigationState.FINAL_APPROACH: CarOperationState.FINAL_APPROACH,
            NavigationState.BLOCKED: CarOperationState.BLOCKED,
            NavigationState.LOCALIZATION_LOST: CarOperationState.LOCALIZATION_LOST,
            NavigationState.FAILED: CarOperationState.FAILED,
        }
        return mapping.get(navigation_state, CarOperationState.FOLLOWING)
    return CarOperationState.STARTING


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radar-port", default=DEFAULT_D500_PORT)
    parser.add_argument("--fleet-port", default=DEFAULT_HC14_PORT)
    parser.add_argument("--radar-x-cm", type=float, default=0.0)
    parser.add_argument("--radar-y-cm", type=float, default=0.0)
    parser.add_argument("--radar-yaw-cw-deg", type=float, default=0.0)
    parser.add_argument("--startup-scans", type=int, default=20)
    parser.add_argument("--calibration-timeout", type=float, default=30.0)
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument("--log-dir", default=None)
    return parser


def default_log_dir() -> Path:
    configured = os.environ.get("CAR_LOG_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path(__file__).resolve().parent / "logs"


def configure_logging(level: str, log_dir: str | os.PathLike[str] | None = None) -> Path:
    directory = default_log_dir() if log_dir is None else Path(log_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    log_path = directory / LOG_FILENAME
    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)s %(threadName)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    file_handler = RotatingFileHandler(
        log_path,
        maxBytes=LOG_MAX_BYTES,
        backupCount=LOG_BACKUP_COUNT,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        handlers=(console, file_handler),
        force=True,
    )
    return log_path


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    configure_logging(args.log_level, args.log_dir)
    app = RoadPatrolCarApplication(
        MainConfig(
            radar_port=args.radar_port,
            fleet_port=args.fleet_port,
            radar_mount=RadarMount(
                args.radar_x_cm,
                args.radar_y_cm,
                args.radar_yaw_cw_deg,
            ),
            startup_scan_count=args.startup_scans,
            calibration_timeout_s=args.calibration_timeout,
        )
    )

    def stop_handler(signum, _frame) -> None:
        LOG.info("received signal %s; stopping", signum)
        app.request_stop()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        return 0 if app.run() else 1
    except KeyboardInterrupt:
        app.request_stop()
        return 130
    except Exception:
        LOG.exception("road-patrol car mission failed")
        return 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
