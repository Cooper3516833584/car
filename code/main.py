#!/usr/bin/env python3
"""ROCK 5A production entry point for radar-localized car navigation."""

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
    DEFAULT_D500_PORT,
    DEFAULT_HC14_PORT,
    AckStatus,
    D500RadarComponent,
    DroneGlobalAlignment,
    DroneGlobalPointMap,
    GroundNavigationProtocol,
    Navigation,
    NavigationCommandReceipt,
    NavigationCommandRejected,
    NavigationConfig,
    NavigationError,
    NavigationGoal,
    NavigationPose,
    NavigationProtocolError,
    NavigationState,
    OccupancyGrid,
    Pose2D,
    PurePursuitConfig,
    PurePursuitController,
    RadarLocalizationUpdate,
    RadarMount,
    RadarScan,
    RectangularWallReference,
    RectangleFieldCalibration,
    RectangleFieldCalibrator,
    RejectReason,
    SerialCommunicationDriver,
    WallFusionConfig,
    VehicleCollisionChecker,
    load_navigation_hmac_key,
    scan_points_in_drone_global,
)


# 自主导航巡航速度，单位 cm/s；50 cm/s = 0.5 m/s。
# 允许范围为 0～100 cm/s。主程序会自动为阿克曼弯道外侧轮预留 20% 速度余量，
# 以后只需修改这一处即可调整正常行驶速度。
NAVIGATION_CRUISE_SPEED_CM_S = 50.0
# Reverse cruise speed, in cm/s.  Keep this independent from the forward
# cruise setting so it can be tuned safely at the top of this file.
NAVIGATION_REVERSE_SPEED_CM_S = 10.0
# 自主导航倒车开关；True 允许规划倒车和前进/倒车换挡，False 只允许前进。
NAVIGATION_ALLOW_REVERSE = True
_MAX_NAVIGATION_CRUISE_SPEED_CM_S = 100.0
_WHEEL_SPEED_HEADROOM = 1.20

LOG = logging.getLogger("car-main")
LOG_FILENAME = "car-main.log"
LOG_MAX_BYTES = 20 * 1024 * 1024
LOG_BACKUP_COUNT = 10
_LOG_LISTENER: QueueListener | None = None


def default_log_dir() -> Path:
    """Return ``logs`` beside this main program unless explicitly overridden."""

    configured = os.environ.get("CAR_LOG_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path(__file__).resolve().parent / "logs"


def configure_logging(log_dir: str | os.PathLike[str], console_level: str) -> Path:
    """Install asynchronous console plus detailed rotating UTF-8 file logging."""

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
    console = logging.StreamHandler()
    console.setLevel(getattr(logging, console_level))
    console.setFormatter(formatter)
    detailed_file = RotatingFileHandler(
        log_path,
        maxBytes=LOG_MAX_BYTES,
        backupCount=LOG_BACKUP_COUNT,
        encoding="utf-8",
    )
    detailed_file.setLevel(logging.DEBUG)
    detailed_file.setFormatter(formatter)

    log_queue: queue.SimpleQueue[logging.LogRecord] = queue.SimpleQueue()
    queued = QueueHandler(log_queue)
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.addHandler(queued)
    _LOG_LISTENER = QueueListener(
        log_queue,
        console,
        detailed_file,
        respect_handler_level=True,
    )
    _LOG_LISTENER.start()
    logging.captureWarnings(True)
    LOG.info(
        "detailed logging enabled file=%s max_bytes=%d backups=%d console_level=%s",
        log_path,
        LOG_MAX_BYTES,
        LOG_BACKUP_COUNT,
        console_level,
    )
    return log_path


def shutdown_logging() -> None:
    """Flush the asynchronous queue and close every handler installed here."""

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
class ConsoleCommand:
    action: str
    goal: NavigationGoal | None = None


def parse_console_command(text: str) -> ConsoleCommand:
    """Parse ``x y [heading]`` or one of the SSH console control words."""

    tokens = text.replace(",", " ").split()
    if not tokens:
        return ConsoleCommand("empty")
    action = tokens[0].lower()
    aliases = {
        "help": "help",
        "?": "help",
        "status": "status",
        "stop": "stop",
        "quit": "quit",
        "exit": "quit",
    }
    if action in aliases:
        if len(tokens) != 1:
            raise ValueError(f"{action} 命令后不能带参数")
        return ConsoleCommand(aliases[action])
    if len(tokens) not in (2, 3):
        raise ValueError("请输入：x_cm y_cm [heading_deg]")
    try:
        x_cm = float(tokens[0])
        y_cm = float(tokens[1])
    except ValueError as exc:
        raise ValueError("x、y 必须是厘米数值") from exc
    heading: int | None = None
    if len(tokens) == 3:
        try:
            heading = int(tokens[2])
        except ValueError as exc:
            raise ValueError("角度必须是 0～359 的整数") from exc
        if not 0 <= heading <= 359:
            raise ValueError("角度必须是 0～359 的整数")
    return ConsoleCommand("navigate", NavigationGoal(x_cm, y_cm, heading))


def _compose_alignment(
    first: DroneGlobalAlignment,
    second: DroneGlobalAlignment,
) -> DroneGlobalAlignment:
    """Compose local->middle ``first`` with middle->global ``second``."""

    x_cm, y_cm = second.point_to_global(first.point_to_global((0.0, 0.0)))
    return DroneGlobalAlignment(
        x_cm,
        y_cm,
        first.yaw_offset_cw_deg + second.yaw_offset_cw_deg,
    )


def rebase_calibration_to_start_pose(
    calibration: RectangleFieldCalibration,
) -> RectangleFieldCalibration:
    """Rebase an edge-aligned rectangle so startup rear axle/heading is 0/0/0."""

    old_global_to_start = DroneGlobalAlignment.from_reference(
        calibration.initial_global_pose,
        Pose2D(),
    )
    local_to_start = _compose_alignment(
        calibration.local_to_global,
        old_global_to_start,
    )
    wall_to_start = _compose_alignment(
        calibration.wall_reference.wall_to_global,
        old_global_to_start,
    )
    corners = tuple(
        old_global_to_start.point_to_global(point)
        for point in calibration.field_polygon_cm
    )
    min_x = min(point[0] for point in corners)
    max_x = max(point[0] for point in corners)
    min_y = min(point[1] for point in corners)
    max_y = max(point[1] for point in corners)
    return RectangleFieldCalibration(
        local_to_start,
        RectangularWallReference(
            wall_to_start,
            calibration.wall_reference.back_wall_x_cm,
            calibration.wall_reference.right_wall_y_cm,
        ),
        Pose2D(),
        min_x,
        max_x,
        min_y,
        max_y,
        calibration.selected_edge_ccw_from_car_deg,
        calibration.fitted_lines,
        corners,
    )


@dataclass(frozen=True, slots=True)
class MainConfig:
    radar_port: str = DEFAULT_D500_PORT
    link_port: str = DEFAULT_HC14_PORT
    radar_mount: RadarMount = RadarMount()
    startup_scan_count: int = 3
    calibration_timeout_s: float = 30.0
    allow_reverse: bool = NAVIGATION_ALLOW_REVERSE
    map_resolution_cm: float = 5.0
    map_margin_cm: float = 15.0
    map_update_interval_s: float = 2.0
    map_min_hits: int = 2
    trusted_max_pose_step_cm: float = 25.0
    trusted_max_yaw_step_deg: float = 15.0
    trusted_max_icp_error_cm: float = 10.0
    footprint_clearance_cm: float = 2.0
    console_enabled: bool = True

    def __post_init__(self) -> None:
        if self.startup_scan_count <= 0 or self.calibration_timeout_s <= 0:
            raise ValueError("startup scan count and timeout must be positive")
        if self.map_resolution_cm <= 0 or self.map_margin_cm < 0:
            raise ValueError("invalid map geometry")
        if self.map_update_interval_s <= 0 or self.map_min_hits <= 0:
            raise ValueError("invalid map update configuration")
        if min(
            self.trusted_max_pose_step_cm,
            self.trusted_max_yaw_step_deg,
            self.trusted_max_icp_error_cm,
        ) <= 0 or self.footprint_clearance_cm < 0:
            raise ValueError("invalid trusted localization configuration")


class CarMainApplication:
    """Own all long-lived components and their safe startup/shutdown order."""

    def __init__(self, config: MainConfig, *, hmac_key: bytes | None) -> None:
        self.config = config
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._scan_event = threading.Event()
        self._startup_scans: list[RadarScan] = []
        self._calibration: RectangleFieldCalibration | None = None
        self._grid: OccupancyGrid | None = None
        self._trusted_map = DroneGlobalPointMap(resolution_cm=config.map_resolution_cm)
        self._last_trusted_pose: Pose2D | None = None
        self._last_trusted_pose_time = 0.0
        self._last_trusted_rejection: str | None = None
        self._ready = False
        self._last_map_update = 0.0
        self._active_receipt: NavigationCommandReceipt | None = None
        self._post_command_acks: list[bytes] = []
        self._handling_link_frame = False
        self._console_mission_active = False
        self._console_thread: threading.Thread | None = None

        LOG.debug(
            "application config radar_port=%s link_port=%s radar_mount=(%.2f,%.2f,%.2f) "
            "startup_scans=%d calibration_timeout_s=%.1f allow_reverse=%s "
            "map_resolution_cm=%.1f map_margin_cm=%.1f map_update_interval_s=%.1f "
            "map_min_hits=%d trusted_gates=(%.1fcm,%.1fdeg,%.1fcm_icp) "
            "footprint_clearance_cm=%.1f console_enabled=%s hmac_enabled=%s",
            config.radar_port,
            config.link_port,
            config.radar_mount.x_forward_cm,
            config.radar_mount.y_left_cm,
            config.radar_mount.yaw_cw_deg,
            config.startup_scan_count,
            config.calibration_timeout_s,
            config.allow_reverse,
            config.map_resolution_cm,
            config.map_margin_cm,
            config.map_update_interval_s,
            config.map_min_hits,
            config.trusted_max_pose_step_cm,
            config.trusted_max_yaw_step_deg,
            config.trusted_max_icp_error_cm,
            config.footprint_clearance_cm,
            config.console_enabled,
            hmac_key is not None,
        )

        self.calibrator = RectangleFieldCalibrator(mount=config.radar_mount)
        self.radar = D500RadarComponent(
            port=config.radar_port,
            mount=config.radar_mount,
            on_update=self._on_radar_update,
            on_connected=lambda: LOG.info("D500 connected on %s", config.radar_port),
            on_disconnected=lambda error: LOG.warning("D500 disconnected: %s", error),
        )
        if not 0.0 < NAVIGATION_CRUISE_SPEED_CM_S <= _MAX_NAVIGATION_CRUISE_SPEED_CM_S:
            raise ValueError(
                "NAVIGATION_CRUISE_SPEED_CM_S must be in (0, 100] cm/s"
            )
        if not 0.0 < NAVIGATION_REVERSE_SPEED_CM_S <= _MAX_NAVIGATION_CRUISE_SPEED_CM_S:
            raise ValueError(
                "NAVIGATION_REVERSE_SPEED_CM_S must be in (0, 100] cm/s"
            )
        cruise_speed_mm_s = NAVIGATION_CRUISE_SPEED_CM_S * 10.0
        reverse_speed_mm_s = NAVIGATION_REVERSE_SPEED_CM_S * 10.0
        highest_command_speed_mm_s = max(cruise_speed_mm_s, reverse_speed_mm_s)
        max_wheel_speed_mm_s = max(
            300.0,
            highest_command_speed_mm_s * _WHEEL_SPEED_HEADROOM,
        )
        pursuit_config = PurePursuitConfig(
            cruise_speed_mm_s=cruise_speed_mm_s,
            max_speed_mm_s=max(150.0, highest_command_speed_mm_s),
            approach_speed_mm_s=min(50.0, highest_command_speed_mm_s),
            reverse_speed_mm_s=reverse_speed_mm_s,
        )
        self.navigation = Navigation(
            config=NavigationConfig(allow_reverse=config.allow_reverse),
            controller=PurePursuitController(config=pursuit_config),
            max_wheel_speed_mm_s=max_wheel_speed_mm_s,
            on_state_changed=self._on_navigation_state,
        )
        LOG.info(
            "navigation configured forward=%.1fcm/s reverse=%.1fcm/s max_wheel=%.1fcm/s allow_reverse=%s",
            NAVIGATION_CRUISE_SPEED_CM_S,
            NAVIGATION_REVERSE_SPEED_CM_S,
            max_wheel_speed_mm_s / 10.0,
            config.allow_reverse,
        )
        self.protocol = None
        self.link = None
        if hmac_key is not None:
            self.protocol = GroundNavigationProtocol(
                key=hmac_key,
                on_goal=self._on_goal_command,
                on_stop=self._on_stop_command,
            )
            self.link = SerialCommunicationDriver(
                port=config.link_port,
                on_bytes=self._on_link_frame,
                on_connected=lambda: LOG.info("HC-14 connected on %s", config.link_port),
                on_disconnected=lambda error: LOG.warning("HC-14 disconnected: %s", error),
                on_callback_error=lambda error: LOG.error("HC-14 callback failed: %s", error),
            )

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready

    def run(self) -> None:
        """Calibrate while stationary, then start navigation and command link."""

        LOG.info(
            "application starting pid=%d python=%s; vehicle must remain stationary "
            "during D500 calibration",
            os.getpid(),
            sys.version.split()[0],
        )
        self.radar.start()
        if not self.radar.serial.wait_connected(min(3.0, self.config.calibration_timeout_s)):
            raise RuntimeError(
                f"D500 UART {self.config.radar_port} could not be opened; "
                "verify rk3588-uart6-m1 overlay, Pin 21 RX wiring and dialout permission"
            )
        fitted_calibration, scans = self._wait_for_rectangle_calibration()
        calibration = rebase_calibration_to_start_pose(fitted_calibration)
        LOG.debug(
            "rectangle fitted scans=%d points=%d lines=%d edge_ccw_deg=%.3f "
            "original_bounds=(%.2f,%.2f,%.2f,%.2f) rebased_corners=%s",
            len(scans),
            sum(len(scan.points) for scan in scans),
            fitted_calibration.fitted_lines,
            fitted_calibration.selected_edge_ccw_from_car_deg,
            fitted_calibration.min_x_cm,
            fitted_calibration.max_x_cm,
            fitted_calibration.min_y_cm,
            fitted_calibration.max_y_cm,
            " ".join(
                f"({x_cm:.2f},{y_cm:.2f})"
                for x_cm, y_cm in calibration.field_polygon_cm
            ),
        )

        # Stop the reader before changing the odometry origin and alignment.
        self.radar.close()
        self.radar.assembler.reset()
        self.radar.odometry.reset(Pose2D())
        self.radar.global_map.clear()
        self.radar.alignment = calibration.local_to_global
        self.radar.enable_wall_fusion(
            calibration.wall_reference,
            fusion_config=WallFusionConfig(),
        )

        startup_points: list[tuple[float, float]] = []
        for scan in scans:
            startup_points.extend(
                scan_points_in_drone_global(
                    scan,
                    Pose2D(),
                    self.config.radar_mount,
                    calibration.local_to_global,
                )
            )
        self.radar.global_map.add_points(startup_points)
        trusted_startup_points = self._filter_vehicle_footprint_points(
            startup_points,
            Pose2D(),
        )
        self._trusted_map.clear()
        self._trusted_map.add_points(trusted_startup_points)
        grid = self._build_grid(trusted_startup_points, calibration)
        self.navigation.update_pose(self._navigation_pose(0.0, 0.0, 0.0))
        self.navigation.set_map(grid)
        self.navigation.start()

        with self._lock:
            self._calibration = calibration
            self._grid = grid
            self._ready = True
            self._last_map_update = time.monotonic()
            self._last_trusted_pose = Pose2D()
            self._last_trusted_pose_time = self._last_map_update
            self._last_trusted_rejection = None

        LOG.info(
            "map complete: startup rear axle=(0,0)cm, startup heading=0deg, "
            "nearest field edge=%.2fdeg CCW, bounds x=[%.1f,%.1f] y=[%.1f,%.1f]cm",
            calibration.selected_edge_ccw_from_car_deg,
            calibration.min_x_cm,
            calibration.max_x_cm,
            calibration.min_y_cm,
            calibration.max_y_cm,
        )
        self.radar.start()
        if self.link is not None:
            self.link.start()
            LOG.info("HC-14 authenticated NAVIGATE_TO input enabled")
        self._print_map_ready(calibration)
        self._start_console_if_available()
        self._stop_event.wait()

    def request_stop(self) -> None:
        LOG.info("application stop requested")
        self._stop_event.set()

    def close(self) -> None:
        LOG.info("application closing")
        with self._lock:
            self._ready = False
        try:
            if self.link is not None:
                self.link.close()
        finally:
            try:
                self.navigation.close()
            finally:
                self.radar.close()
        LOG.info("application closed; hardware outputs are safe")

    def _wait_for_rectangle_calibration(
        self,
    ) -> tuple[RectangleFieldCalibration, tuple[RadarScan, ...]]:
        deadline = time.monotonic() + self.config.calibration_timeout_s
        last_error = (
            f"D500 UART {self.config.radar_port} is open but no complete scan arrived; "
            "verify D500 TX -> Pin 21, 230400 baud, power and common ground"
        )
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            self._scan_event.wait(0.5)
            self._scan_event.clear()
            with self._lock:
                scans = tuple(self._startup_scans[-self.config.startup_scan_count :])
            if len(scans) < self.config.startup_scan_count:
                continue
            try:
                return self.calibrator.calibrate(scans), scans
            except (ValueError, RuntimeError) as exc:
                message = str(exc)
                if message != last_error:
                    LOG.warning("rectangle calibration retry: %s", message)
                    last_error = message
        raise RuntimeError(f"rectangle field calibration timed out: {last_error}")

    def _on_radar_update(self, update: RadarLocalizationUpdate) -> None:
        with self._lock:
            ready = self._ready
        self._log_radar_update(update, phase="navigation" if ready else "calibration")
        with self._lock:
            if not ready:
                self._startup_scans.append(update.scan)
                limit = max(self.config.startup_scan_count * 2, self.config.startup_scan_count)
                del self._startup_scans[:-limit]
                LOG.debug(
                    "calibration scan buffered count=%d required=%d",
                    len(self._startup_scans),
                    self.config.startup_scan_count,
                )
                self._scan_event.set()
                return

        now = time.monotonic()
        rejection = self._trusted_localization_rejection(update)
        if rejection is not None:
            with self._lock:
                self._last_trusted_rejection = rejection
            LOG.warning(
                "radar trusted localization rejected reason=%s; navigation pose/map not refreshed",
                rejection,
            )
            return

        navigation_accepted = self.navigation.update_from_radar(update)
        LOG.debug("radar trusted navigation pose accepted=%s", navigation_accepted)
        if not navigation_accepted or update.global_pose is None:
            return

        with self._lock:
            self._last_trusted_pose = update.global_pose
            self._last_trusted_pose_time = now
            self._last_trusted_rejection = None

        wall = update.wall_fusion
        wall_rejected = wall is not None and wall.attempted and not wall.accepted
        filtered_points = self._filter_vehicle_footprint_points(
            update.global_points_cm,
            update.global_pose,
        )
        if wall_rejected:
            LOG.warning(
                "trusted map scan skipped because wall correction was rejected reason=%r",
                wall.reason,
            )
        else:
            self._trusted_map.add_points(filtered_points)
        LOG.debug(
            "trusted map scan accepted raw_points=%d retained_points=%d wall_rejected=%s",
            len(update.global_points_cm),
            len(filtered_points),
            wall_rejected,
        )
        self._refresh_trusted_grid(now=now)

    def _trusted_localization_rejection(
        self,
        update: RadarLocalizationUpdate,
    ) -> str | None:
        """Return why a radar pose cannot safely update Navigation, or None."""

        pose = update.global_pose
        if pose is None:
            return "global alignment unavailable"
        if not update.odometry.accepted:
            return f"odometry rejected: {update.odometry.rejection_reason or 'unknown'}"
        if not all(math.isfinite(value) for value in (pose.x_cm, pose.y_cm, pose.yaw_cw_deg)):
            return "non-finite global pose"
        icp = update.odometry.icp
        if icp is not None and (
            not math.isfinite(icp.mean_error_cm)
            or icp.mean_error_cm > self.config.trusted_max_icp_error_cm
        ):
            return f"ICP error {icp.mean_error_cm:.2f}cm exceeds trusted gate"

        with self._lock:
            calibration = self._calibration
            previous = self._last_trusted_pose
        if calibration is None:
            return "field calibration unavailable"
        outside_corners = [
            corner
            for corner in self._vehicle_footprint_corners(pose)
            if not calibration.contains_point(*corner)
        ]
        if outside_corners:
            return f"vehicle footprint outside fitted field at {outside_corners[0]}"
        if previous is not None:
            step_cm = math.hypot(pose.x_cm - previous.x_cm, pose.y_cm - previous.y_cm)
            if step_cm > self.config.trusted_max_pose_step_cm:
                return (
                    f"pose translation jump {step_cm:.2f}cm exceeds "
                    f"{self.config.trusted_max_pose_step_cm:.2f}cm"
                )
            yaw_step = abs(
                (pose.yaw_cw_deg - previous.yaw_cw_deg + 180.0) % 360.0 - 180.0
            )
            if yaw_step > self.config.trusted_max_yaw_step_deg:
                return (
                    f"pose yaw jump {yaw_step:.2f}deg exceeds "
                    f"{self.config.trusted_max_yaw_step_deg:.2f}deg"
                )
        return None

    def _vehicle_footprint_corners(self, pose: Pose2D) -> tuple[tuple[float, float], ...]:
        geometry = self.navigation.geometry
        clearance = self.config.footprint_clearance_cm
        centre_x_body = geometry.rear_axle_to_body_center_cm
        half_length = geometry.body_length_cm / 2.0 + clearance
        half_width = geometry.body_width_cm / 2.0 + clearance
        yaw = math.radians(pose.yaw_cw_deg)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        corners: list[tuple[float, float]] = []
        for body_x, body_y in (
            (centre_x_body + half_length, half_width),
            (centre_x_body + half_length, -half_width),
            (centre_x_body - half_length, half_width),
            (centre_x_body - half_length, -half_width),
        ):
            corners.append(
                (
                    pose.x_cm + cosine * body_x + sine * body_y,
                    pose.y_cm - sine * body_x + cosine * body_y,
                )
            )
        return tuple(corners)

    def _filter_vehicle_footprint_points(
        self,
        points: list[tuple[float, float]] | tuple[tuple[float, float], ...],
        pose: Pose2D,
    ) -> list[tuple[float, float]]:
        """Remove radar self-returns and stale hits under the physical car."""

        geometry = self.navigation.geometry
        clearance = self.config.footprint_clearance_cm
        half_length = geometry.body_length_cm / 2.0 + clearance
        half_width = geometry.body_width_cm / 2.0 + clearance
        yaw = math.radians(pose.yaw_cw_deg)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        retained: list[tuple[float, float]] = []
        for point_x, point_y in points:
            dx, dy = point_x - pose.x_cm, point_y - pose.y_cm
            body_x = cosine * dx - sine * dy
            body_y = sine * dx + cosine * dy
            inside = (
                abs(body_x - geometry.rear_axle_to_body_center_cm) <= half_length
                and abs(body_y) <= half_width
            )
            if not inside:
                retained.append((point_x, point_y))
        return retained

    def _refresh_trusted_grid(self, *, now: float | None = None, force: bool = False) -> bool:
        timestamp = time.monotonic() if now is None else now
        with self._lock:
            calibration = self._calibration
            pose = self._last_trusted_pose
            elapsed = timestamp - self._last_map_update
        if calibration is None or pose is None:
            return False
        if not force and elapsed < self.config.map_update_interval_s:
            return False
        cells = self._trusted_map.cells(min_hits=self.config.map_min_hits)
        points = self._filter_vehicle_footprint_points(
            [(cell.x_cm, cell.y_cm) for cell in cells],
            pose,
        )
        grid = self._build_grid(points, calibration)
        changed = self.navigation.set_map(grid)
        with self._lock:
            self._grid = grid
            self._last_map_update = timestamp
        LOG.debug(
            "trusted map refresh source_cells=%d retained_after_current_footprint=%d "
            "grid_changed=%s force=%s",
            len(cells),
            len(points),
            changed,
            force,
        )
        return changed

    @staticmethod
    def _log_radar_update(update: RadarLocalizationUpdate, *, phase: str) -> None:
        odometry = update.odometry
        local_pose = odometry.pose
        global_pose = update.global_pose
        icp = odometry.icp
        wall = update.wall_fusion
        if icp is None:
            icp_values = (0, math.nan, 0, math.nan, math.nan, math.nan)
        else:
            delta = icp.transform_current_to_reference
            icp_values = (
                icp.matched_points,
                icp.mean_error_cm,
                icp.iterations,
                delta.x_cm,
                delta.y_cm,
                delta.yaw_cw_deg,
            )
        LOG.debug(
            "radar phase=%s scan_ts_ms=%d points=%d rotation_deg_s=%d "
            "odometry_accepted=%s initialized=%s rejection=%r "
            "local_pose=(%.3f,%.3f,%.3f) global_pose=%s "
            "icp=(matched=%d,error_cm=%.4f,iterations=%d,delta=%.3f,%.3f,%.3f) "
            "wall=(attempted=%s,accepted=%s,reason=%r) global_points=%d",
            phase,
            update.scan.timestamp_ms,
            len(update.scan.points),
            update.scan.rotation_speed_deg_s,
            odometry.accepted,
            odometry.initialized,
            odometry.rejection_reason,
            local_pose.x_cm,
            local_pose.y_cm,
            local_pose.yaw_cw_deg,
            "none"
            if global_pose is None
            else f"({global_pose.x_cm:.3f},{global_pose.y_cm:.3f},"
            f"{global_pose.yaw_cw_deg:.3f})",
            *icp_values,
            False if wall is None else wall.attempted,
            False if wall is None else wall.accepted,
            None if wall is None else wall.reason,
            len(update.global_points_cm),
        )

    def _build_grid(
        self,
        obstacle_points: list[tuple[float, float]],
        calibration: RectangleFieldCalibration,
    ) -> OccupancyGrid:
        resolution = self.config.map_resolution_cm
        margin = self.config.map_margin_cm
        origin_x = math.floor((calibration.min_x_cm - margin) / resolution) * resolution
        origin_y = math.floor((calibration.min_y_cm - margin) / resolution) * resolution
        max_x = math.ceil((calibration.max_x_cm + margin) / resolution) * resolution
        max_y = math.ceil((calibration.max_y_cm + margin) / resolution) * resolution
        width = max(1, round((max_x - origin_x) / resolution))
        height = max(1, round((max_y - origin_y) / resolution))
        grid = OccupancyGrid.from_obstacle_points(
            obstacle_points,
            resolution_cm=resolution,
            origin_x_cm=origin_x,
            origin_y_cm=origin_y,
            width=width,
            height=height,
        )
        # The fitted rectangle is the only area declared as known free space.
        # Keep the margin cells for footprint collision checks, but mark every
        # cell outside the field as occupied so a path can never route around a
        # sparse/missed wall return.
        cells = list(grid.cells)
        for iy in range(height):
            for ix in range(width):
                x_cm, y_cm = grid.cell_center(ix, iy)
                if not calibration.contains_point(x_cm, y_cm):
                    cells[iy * width + ix] = 100
        result = OccupancyGrid(
            grid.resolution_cm,
            grid.origin_x_cm,
            grid.origin_y_cm,
            grid.width,
            grid.height,
            tuple(cells),
            grid.occupied_threshold,
            grid.unknown_is_occupied,
        )
        LOG.debug(
            "grid built obstacle_points=%d dimensions=%dx%d resolution_cm=%.2f "
            "origin=(%.2f,%.2f) occupied_cells=%d",
            len(obstacle_points),
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            sum(value >= result.occupied_threshold for value in result.cells),
        )
        return result

    @staticmethod
    def _navigation_pose(
        x_cm: float,
        y_cm: float,
        heading_deg: float,
    ) -> NavigationPose:
        return NavigationPose(x_cm, y_cm, heading_deg, time.monotonic())

    def _on_link_frame(self, frame: bytes) -> None:
        if self.protocol is None:
            return
        LOG.debug("HC-14 frame received bytes=%d", len(frame))
        with self._lock:
            self._handling_link_frame = True
        try:
            try:
                replies = self.protocol.handle_frame(frame)
            except NavigationProtocolError as exc:
                LOG.warning("rejected unauthenticated/malformed ground frame: %s", exc)
                replies = ()
            for reply in replies:
                self._send_frame(reply)
            LOG.debug("HC-14 frame handled immediate_replies=%d", len(replies))
        finally:
            with self._lock:
                self._handling_link_frame = False
                post_acks, self._post_command_acks = self._post_command_acks, []
        for reply in post_acks:
            self._send_frame(reply)

    def _on_goal_command(
        self,
        goal: NavigationGoal,
        receipt: NavigationCommandReceipt,
    ) -> None:
        with self._lock:
            if not self._ready or self._calibration is None:
                raise NavigationCommandRejected(RejectReason.TASK_BUSY, "startup calibration incomplete")
            if self._active_receipt is not None or self._console_mission_active:
                raise NavigationCommandRejected(RejectReason.TASK_BUSY, "navigation already active")
            calibration = self._calibration
        if not calibration.contains_point(goal.x_cm, goal.y_cm):
            raise NavigationCommandRejected(RejectReason.BAD_PAYLOAD, "goal lies outside field")
        self._refresh_trusted_grid(force=True)
        with self._lock:
            self._active_receipt = receipt
        try:
            self.navigation.set_goal(goal)
            self.navigation.start_navigation()
        except BaseException:
            with self._lock:
                if self._active_receipt == receipt:
                    self._active_receipt = None
            raise
        LOG.info(
            "accepted goal x=%.1f y=%.1f heading=%s",
            goal.x_cm,
            goal.y_cm,
            "none" if goal.final_heading_deg is None else f"{goal.final_heading_deg:.2f}",
        )

    def _on_stop_command(self, receipt: NavigationCommandReceipt) -> None:
        self.navigation.cancel()
        with self._lock:
            previous = self._active_receipt
            self._active_receipt = None
            if previous is not None:
                self._post_command_acks.append(
                    self.protocol.build_status_ack(previous, AckStatus.FAILED, RejectReason.NONE)
                )
            self._post_command_acks.append(
                self.protocol.build_status_ack(receipt, AckStatus.COMPLETED, RejectReason.NONE)
            )
        LOG.info("navigation stopped by remote command")

    def _on_navigation_state(self, state: NavigationState, reason: str) -> None:
        LOG.info("navigation state=%s reason=%s", state.value, reason)
        if state is NavigationState.BLOCKED:
            self._log_navigation_blocked(reason)
        if state not in (NavigationState.ARRIVED, NavigationState.FAILED, NavigationState.BLOCKED):
            return
        with self._lock:
            receipt = self._active_receipt
            self._active_receipt = None
            console_mission = self._console_mission_active
            self._console_mission_active = False
        ready_reason = "ready for next goal; startup map and origin retained"
        if console_mission:
            if state is NavigationState.ARRIVED:
                self._console_print("已到达目标，位置与可选车头方向均满足容差。")
            else:
                self._console_print(f"任务失败：{state.value}，{reason}")
            self.navigation.cancel(reason=ready_reason)
            self._console_print("可继续输入下一目标；启动原点和地图坐标系保持不变。")
            LOG.info("terminal mission reset for next SSH goal; startup origin retained")
            return
        if receipt is not None:
            if state is NavigationState.ARRIVED:
                reply = self.protocol.build_status_ack(
                    receipt,
                    AckStatus.COMPLETED,
                    RejectReason.NONE,
                )
            else:
                reply = self.protocol.build_status_ack(
                    receipt,
                    AckStatus.FAILED,
                    RejectReason.BAD_PAYLOAD,
                )
            self._send_or_queue_status(reply)
        self.navigation.cancel(reason=ready_reason)
        LOG.info("terminal mission reset for next remote goal; startup origin retained")

    def _log_navigation_blocked(self, reason: str) -> None:
        """Record evidence that distinguishes a real obstacle from map drift."""

        pose = self.navigation.pose
        with self._lock:
            grid = self._grid
            calibration = self._calibration
            trusted_pose = self._last_trusted_pose
            trusted_time = self._last_trusted_pose_time
            trusted_rejection = self._last_trusted_rejection
        if pose is None or grid is None or calibration is None:
            LOG.error(
                "navigation blocked diagnostics unavailable reason=%r pose=%s grid=%s calibration=%s",
                reason,
                pose is not None,
                grid is not None,
                calibration is not None,
            )
            return

        radar_pose = Pose2D(pose.x_cm, pose.y_cm, (-pose.heading_deg) % 360.0)
        corners = self._vehicle_footprint_corners(radar_pose)
        checker = VehicleCollisionChecker(
            grid,
            self.navigation.geometry,
            safety_margin_cm=self.navigation.planner.config.safety_margin_cm,
        )
        min_x, max_x = min(x for x, _ in corners), max(x for x, _ in corners)
        min_y, max_y = min(y for _, y in corners), max(y for _, y in corners)
        min_ix, min_iy = grid.world_to_cell(min_x, min_y)
        max_ix, max_iy = grid.world_to_cell(max_x, max_y)
        occupied: list[tuple[int, int, float, float, int]] = []
        for iy in range(max(0, min_iy), min(grid.height - 1, max_iy) + 1):
            for ix in range(max(0, min_ix), min(grid.width - 1, max_ix) + 1):
                if not grid.is_occupied(ix, iy):
                    continue
                x_cm, y_cm = grid.cell_center(ix, iy)
                occupied.append((ix, iy, x_cm, y_cm, grid.cells[iy * grid.width + ix]))
                if len(occupied) >= 16:
                    break
            if len(occupied) >= 16:
                break
        LOG.error(
            "navigation blocked diagnostics reason=%r pose=(%.2f,%.2f,%.2f) "
            "pose_free=%s rear_axle_inside=%s footprint_inside=%s corners=%s "
            "nearby_occupied=%s map_revision=%d occupied_cells=%d "
            "trusted_pose=%s trusted_age_s=%.3f last_trusted_rejection=%r "
            "trusted_map_cells=%d raw_map_cells=%d",
            reason,
            pose.x_cm,
            pose.y_cm,
            pose.heading_deg,
            checker.is_pose_free(pose),
            calibration.contains_point(pose.x_cm, pose.y_cm),
            all(calibration.contains_point(*corner) for corner in corners),
            tuple((round(x, 2), round(y, 2)) for x, y in corners),
            occupied,
            self.navigation.map_revision,
            sum(value >= grid.occupied_threshold for value in grid.cells),
            "none"
            if trusted_pose is None
            else f"({trusted_pose.x_cm:.2f},{trusted_pose.y_cm:.2f},{trusted_pose.yaw_cw_deg:.2f})",
            math.inf if trusted_time <= 0 else max(0.0, time.monotonic() - trusted_time),
            trusted_rejection,
            len(self._trusted_map.cells(min_hits=self.config.map_min_hits)),
            len(self.radar.global_map.cells(min_hits=self.config.map_min_hits)),
        )

    def _send_or_queue_status(self, frame: bytes) -> None:
        with self._lock:
            if self._handling_link_frame:
                self._post_command_acks.append(frame)
                return
        self._send_frame(frame)

    def _send_frame(self, frame: bytes) -> None:
        if self.link is None:
            return
        try:
            self.link.write(frame)
            LOG.debug("HC-14 reply sent bytes=%d", len(frame))
        except Exception as exc:
            LOG.warning("could not send ground reply: %s", exc)

    def _print_map_ready(self, calibration: RectangleFieldCalibration) -> None:
        corners = " ".join(
            f"({x_cm:.1f},{y_cm:.1f})" for x_cm, y_cm in calibration.field_polygon_cm
        )
        self._console_print("")
        self._console_print("=== 建图完成，Navigation 已就绪 ===")
        self._console_print("启动位姿：x=0 cm, y=0 cm, heading=0°（车头方向）")
        self._console_print(f"场地边界：{corners}")
        self._console_print("输入：x_cm y_cm [heading_deg]，角度可选且必须为 0～359 整数")
        self._console_print("命令：status 查看状态，stop 停车取消，help 帮助，quit 安全退出")

    def _start_console_if_available(self) -> None:
        if not self.config.console_enabled:
            return
        if not sys.stdin.isatty():
            LOG.warning("SSH console disabled because stdin is not a TTY; use ssh -t")
            return
        self._console_thread = threading.Thread(
            target=self._console_loop,
            name="car-ssh-console",
            daemon=True,
        )
        self._console_thread.start()

    def _console_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                line = input("car-nav> ")
            except EOFError:
                self._console_print("SSH 输入已关闭，正在安全停车并退出。")
                self.request_stop()
                return
            try:
                LOG.debug("SSH console input=%r", line)
                command = parse_console_command(line)
                self._handle_console_command(command)
            except (ValueError, NavigationCommandRejected, NavigationError) as exc:
                self._console_print(f"输入被拒绝：{exc}")
            except Exception as exc:
                LOG.exception("SSH console command failed")
                self._console_print(f"命令失败：{exc}")

    def _handle_console_command(self, command: ConsoleCommand) -> None:
        LOG.debug(
            "SSH command action=%s goal=%s",
            command.action,
            "none"
            if command.goal is None
            else f"({command.goal.x_cm:.2f},{command.goal.y_cm:.2f},"
            f"{command.goal.final_heading_deg})",
        )
        if command.action == "empty":
            return
        if command.action == "help":
            self._console_print("示例：200 50 或 200 50 90；单位 cm，角度逆时针为正。")
            self._console_print("同一时间只执行一个目标；新目标前可输入 stop。")
            return
        if command.action == "status":
            pose = self.navigation.pose
            pose_text = "暂无有效定位"
            if pose is not None:
                pose_text = (
                    f"x={pose.x_cm:.1f}cm y={pose.y_cm:.1f}cm "
                    f"heading={pose.heading_deg:.1f}°"
                )
            self._console_print(
                f"状态={self.navigation.state.value}，{pose_text}，"
                f"原因={self.navigation.state_reason or '-'}"
            )
            tracker = self.navigation.last_tracker_command
            if tracker is not None:
                self._console_print(
                    "跟踪反馈："
                    f"横向误差={tracker.signed_cross_track_error_cm:+.1f}cm，"
                    f"航向误差={tracker.heading_error_deg:+.1f}°，"
                    f"目标舵角={tracker.steering_angle_rad:+.3f}rad，"
                    f"目标速度={tracker.speed_mm_s:.1f}mm/s"
                )
            plan = self.navigation.last_motion_plan
            if plan is not None:
                self._console_print(
                    "最近驱动："
                    f"实际舵角={plan.steering.angle_rad:+.3f}rad/"
                    f"{plan.steering.pulse_us}us，"
                    f"后轮=({plan.rear.requested.left_mm_s:.1f},"
                    f"{plan.rear.requested.right_mm_s:.1f})mm/s，"
                    f"C10B Vx/Vz=({plan.rear.linear_mm_s},"
                    f"{plan.rear.angular_mrad_s})"
                )
            return
        if command.action == "stop":
            self._cancel_from_console()
            self._console_print("已停车并取消当前任务。")
            return
        if command.action == "quit":
            self._cancel_from_console()
            self._console_print("正在安全退出 main。")
            self.request_stop()
            return
        if command.action != "navigate" or command.goal is None:
            raise ValueError("未知命令")
        self._submit_console_goal(command.goal)

    def _submit_console_goal(self, goal: NavigationGoal) -> None:
        with self._lock:
            calibration = self._calibration
            if not self._ready or calibration is None:
                raise NavigationCommandRejected(RejectReason.TASK_BUSY, "建图尚未完成")
            if self._console_mission_active or self._active_receipt is not None:
                raise NavigationCommandRejected(RejectReason.TASK_BUSY, "已有任务，先输入 stop")
            if not calibration.contains_point(goal.x_cm, goal.y_cm):
                raise NavigationCommandRejected(RejectReason.BAD_PAYLOAD, "目标位于拟合场地外")
            self._console_mission_active = True
        try:
            self._refresh_trusted_grid(force=True)
            self.navigation.set_goal(goal)
            self.navigation.start_navigation()
        except BaseException:
            with self._lock:
                self._console_mission_active = False
            raise
        heading = "不限定" if goal.final_heading_deg is None else f"{goal.final_heading_deg:.0f}°"
        self._console_print(
            f"已接受目标：x={goal.x_cm:.1f}cm y={goal.y_cm:.1f}cm heading={heading}；"
            "正在自主规划并使用雷达持续纠偏。"
        )
        LOG.info(
            "accepted SSH goal x=%.2f y=%.2f heading=%s",
            goal.x_cm,
            goal.y_cm,
            "none" if goal.final_heading_deg is None else f"{goal.final_heading_deg:.2f}",
        )

    def _cancel_from_console(self) -> None:
        LOG.info("SSH requested navigation cancellation")
        with self._lock:
            receipt = self._active_receipt
            self._active_receipt = None
            self._console_mission_active = False
        self.navigation.cancel()
        if receipt is not None and self.protocol is not None:
            self._send_frame(
                self.protocol.build_status_ack(receipt, AckStatus.FAILED, RejectReason.NONE)
            )

    @staticmethod
    def _console_print(message: str) -> None:
        print(message, flush=True)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radar-port", default=DEFAULT_D500_PORT)
    parser.add_argument("--link-port", default=DEFAULT_HC14_PORT)
    parser.add_argument("--radar-x-cm", type=float, default=0.0)
    parser.add_argument("--radar-y-cm", type=float, default=0.0)
    parser.add_argument("--radar-yaw-cw-deg", type=float, default=0.0)
    parser.add_argument("--startup-scans", type=int, default=3)
    parser.add_argument("--calibration-timeout", type=float, default=30.0)
    reverse_group = parser.add_mutually_exclusive_group()
    reverse_group.add_argument(
        "--allow-reverse",
        dest="allow_reverse",
        action="store_true",
        help="allow reversing (default follows NAVIGATION_ALLOW_REVERSE)",
    )
    reverse_group.add_argument(
        "--no-reverse",
        dest="allow_reverse",
        action="store_false",
        help="temporarily disable reversing",
    )
    parser.set_defaults(allow_reverse=NAVIGATION_ALLOW_REVERSE)
    parser.add_argument("--no-console", action="store_true", help="disable SSH terminal input")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO")
    parser.add_argument(
        "--log-dir",
        default=None,
        help="detailed log directory (default: logs beside main.py; CAR_LOG_DIR also supported)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    requested_log_dir = default_log_dir() if args.log_dir is None else Path(args.log_dir)
    try:
        configure_logging(requested_log_dir, args.log_level)
    except OSError as exc:
        print(f"cannot create detailed log in {requested_log_dir}: {exc}", file=sys.stderr)
        return 2
    app: CarMainApplication | None = None
    try:
        key = None
        if os.environ.get("GROUND_STATION_HMAC_KEY_HEX", "").strip():
            try:
                key = load_navigation_hmac_key()
            except NavigationProtocolError as exc:
                LOG.error("invalid ground-station HMAC key: %s", exc)
                return 2
        else:
            LOG.warning(
                "GROUND_STATION_HMAC_KEY_HEX is not set; HC-14 command input is disabled, "
                "SSH console remains available"
            )
        config = MainConfig(
            radar_port=args.radar_port,
            link_port=args.link_port,
            radar_mount=RadarMount(
                args.radar_x_cm,
                args.radar_y_cm,
                args.radar_yaw_cw_deg,
            ),
            startup_scan_count=args.startup_scans,
            calibration_timeout_s=args.calibration_timeout,
            allow_reverse=args.allow_reverse,
            console_enabled=not args.no_console,
        )
        app = CarMainApplication(config, hmac_key=key)

        def stop_handler(signum, frame) -> None:
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
        try:
            if app is not None:
                app.close()
        finally:
            shutdown_logging()


if __name__ == "__main__":
    raise SystemExit(main())
