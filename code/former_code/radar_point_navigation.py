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
    CoordinateGoalRejected,
    CoordinateGoalRejectReason,
    CoordinateNavigation,
    CoordinateNavigationConfig,
    DroneGlobalAlignment,
    GroundNavigationProtocol,
    NavigationCommandReceipt,
    NavigationCommandRejected,
    NavigationError,
    NavigationGoal,
    NavigationProtocolError,
    NavigationState,
    Pose2D,
    RadarMount,
    RectangleFieldCalibration,
    RejectReason,
    SerialCommunicationDriver,
    VehicleCollisionChecker,
    load_navigation_hmac_key,
)
from components.fleet_car_node import FleetCarNode
from components.fleet_models import (
    AckReason as FleetAckReason,
    AckStatus as FleetAckStatus,
    CarFleetState,
    CarNavigateCommand,
    CommandResult as FleetCommandResult,
    CoordinateFrameCommand,
    DisasterRescueCommand,
    NodeFlags as FleetNodeFlags,
    TerrainCode,
)
from components.grid_rescue_mission import (
    AdjacentGridNavigator,
    AdjacentGridRescuePlanner,
    GridLayout,
    GridRescueMissionController,
    InPlaceDifferentialTurn,
    overlay_blocked_terrain,
)
from components.sound_light_alarm import (
    AlarmGPIOError,
    SoundLightAlarm,
    alarm_off,
    alarm_on,
)


# 自主导航巡航速度，单位 cm/s；定位调试阶段保持 10 cm/s = 0.1 m/s。
# 允许范围为 0～100 cm/s。主程序会自动为阿克曼弯道外侧轮预留 20% 速度余量，
# 以后只需修改这一处即可调整正常行驶速度。
# Production forward cruise speed.  Curves, localization degradation and the
# final 60 cm use lower safety-controlled speeds.
NAVIGATION_CRUISE_SPEED_CM_S = 30.0
# Reverse cruise speed, in cm/s.  Keep this independent from the forward
# cruise setting so it can be tuned safely at the top of this file.
NAVIGATION_REVERSE_SPEED_CM_S = 15.0
# 自主导航倒车开关；True 允许规划倒车和前进/倒车换挡，False 只允许前进。
NAVIGATION_ALLOW_REVERSE = True
# ==================== 比赛现场只需编辑以下三个列表 ====================
# 模拟取水地块：可临时改成 field 等，不要求一定是真实 lake/river。
WATER_PICKUP_TERRAINS = ["lake", "river"]
# 模拟灭火目标地块：通常保持 wildfire。
WILDFIRE_TARGET_TERRAINS = ["wildfire"]
# 禁止进入地块：当前无要求所以留空；如需避开居民地可填 ["settlements"]。
FORBIDDEN_TERRAINS = []
# =====================================================================

TERRAIN_NAME_TO_CODE = {
    "snow_mountain": int(TerrainCode.SNOW_MOUNTAIN),
    "field": int(TerrainCode.FIELD),
    "river": int(TerrainCode.RIVER),
    "settlements": int(TerrainCode.SETTLEMENTS),
    "lake": int(TerrainCode.LAKE),
    "debris_flow": int(TerrainCode.DEBRIS_FLOW),
    "wildfire": int(TerrainCode.WILDFIRE),
}
RESCUE_NAVIGATION_STEP_TIMEOUT_S = 90.0
# The ground car uses wall lines only for conservative drift correction over
# continuous ICP; quality, residual and per-update jump gates remain mandatory.
RADAR_ABSOLUTE_WALL_LOW_PASS_RATIO = 0.20
_MAX_NAVIGATION_CRUISE_SPEED_CM_S = 100.0
_WHEEL_SPEED_HEADROOM = 1.20

LOG = logging.getLogger("car-main")
LOG_FILENAME = "car-main.log"
LOG_MAX_BYTES = 20 * 1024 * 1024
LOG_BACKUP_COUNT = 10
_LOG_LISTENER: QueueListener | None = None


def terrain_codes(names) -> tuple[int, ...]:
    normalized = tuple(str(name).strip().lower() for name in names)
    unknown = sorted(set(normalized) - set(TERRAIN_NAME_TO_CODE))
    if unknown:
        raise ValueError("unknown terrain names: {}".format(", ".join(unknown)))
    return tuple(TERRAIN_NAME_TO_CODE[name] for name in normalized)


def default_log_dir() -> Path:
    """Return ``logs`` beside this main program unless explicitly overridden."""

    configured = os.environ.get("CAR_LOG_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path(__file__).resolve().parent / "logs"


def configure_logging(log_dir: str | os.PathLike[str], console_level: str) -> Path:
    """Install detailed file logging and optional, explicitly enabled console logs.

    The SSH console is also the operator command input. Keeping normal runtime
    logging off that stream prevents asynchronous radar/control diagnostics from
    overwriting a partially typed coordinate command.
    """

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
    queued = QueueHandler(log_queue)
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.addHandler(queued)
    handlers: list[logging.Handler] = [detailed_file]
    if console_level != "OFF":
        console = logging.StreamHandler()
        console.setLevel(getattr(logging, console_level))
        console.setFormatter(formatter)
        handlers.insert(0, console)
    _LOG_LISTENER = QueueListener(log_queue, *handlers, respect_handler_level=True)
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


@dataclass(frozen=True, slots=True)
class MainConfig:
    radar_port: str = DEFAULT_D500_PORT
    link_port: str = DEFAULT_HC14_PORT
    radar_mount: RadarMount = RadarMount()
    startup_scan_count: int = 20
    calibration_timeout_s: float = 30.0
    allow_reverse: bool = NAVIGATION_ALLOW_REVERSE
    allow_in_place_rotation: bool = False
    map_resolution_cm: float = 5.0
    map_margin_cm: float = 15.0
    map_update_interval_s: float = 0.5
    map_min_hits: int = 2
    trusted_max_pose_step_cm: float = 25.0
    trusted_max_yaw_step_deg: float = 15.0
    trusted_max_icp_error_cm: float = 10.0
    footprint_clearance_cm: float = 2.0
    wall_rotation_adaptation: bool = True
    wall_low_pass_ratio: float = RADAR_ABSOLUTE_WALL_LOW_PASS_RATIO
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
        if not 0.0 < self.wall_low_pass_ratio <= 1.0:
            raise ValueError("wall_low_pass_ratio must be in (0, 1]")


class CarMainApplication:
    """Own all long-lived components and their safe startup/shutdown order."""

    def __init__(
        self,
        config: MainConfig,
        *,
        hmac_key: bytes | None,
        fleet_bus: bool = False,
    ) -> None:
        if fleet_bus and hmac_key is not None:
            raise ValueError("FleetBus and legacy HMAC mode cannot run together")
        self.config = config
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._ready = False
        self._active_receipt: NavigationCommandReceipt | None = None
        self._post_command_acks: list[bytes] = []
        self._handling_link_frame = False
        self._console_mission_active = False
        self._fleet_mission_active = False
        self._fleet_mapping_active = False
        self._fleet_mapping_thread: threading.Thread | None = None
        self._console_thread: threading.Thread | None = None
        self._started_at = time.monotonic()
        self._fleet_alignment: DroneGlobalAlignment | None = None
        self._rescue_condition = threading.Condition()
        self._rescue_terminal_generation = 0
        self._rescue_terminal_state: NavigationState | None = None
        self._rescue_mission_active = False
        self._rescue_request_seq: int | None = None
        self._semantic_step = None
        self._semantic_lock = threading.Lock()
        self.rescue_controller: GridRescueMissionController | None = None
        self._alarm: SoundLightAlarm | None = None

        LOG.debug(
            "application config radar_port=%s link_port=%s radar_mount=(%.2f,%.2f,%.2f) "
            "startup_scans=%d calibration_timeout_s=%.1f allow_reverse=%s "
            "map_resolution_cm=%.1f map_margin_cm=%.1f map_update_interval_s=%.1f "
            "map_min_hits=%d trusted_gates=(%.1fcm,%.1fdeg,%.1fcm_icp) "
            "footprint_clearance_cm=%.1f wall_rotation_adaptation=%s "
            "wall_low_pass_ratio=%.2f console_enabled=%s hmac_enabled=%s",
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
            config.wall_rotation_adaptation,
            config.wall_low_pass_ratio,
            config.console_enabled,
            hmac_key is not None,
        )
        self.coordinate_navigation = CoordinateNavigation(
            CoordinateNavigationConfig(
                radar_port=config.radar_port,
                radar_mount=config.radar_mount,
                startup_scan_count=config.startup_scan_count,
                calibration_timeout_s=config.calibration_timeout_s,
                allow_reverse=config.allow_reverse,
                allow_in_place_rotation=(
                    config.allow_in_place_rotation or fleet_bus
                ),
                cruise_speed_cm_s=NAVIGATION_CRUISE_SPEED_CM_S,
                reverse_speed_cm_s=NAVIGATION_REVERSE_SPEED_CM_S,
                max_cruise_speed_cm_s=_MAX_NAVIGATION_CRUISE_SPEED_CM_S,
                wheel_speed_headroom=_WHEEL_SPEED_HEADROOM,
                map_resolution_cm=config.map_resolution_cm,
                map_margin_cm=config.map_margin_cm,
                map_update_interval_s=config.map_update_interval_s,
                map_min_hits=config.map_min_hits,
                trusted_max_pose_step_cm=config.trusted_max_pose_step_cm,
                trusted_max_yaw_step_deg=config.trusted_max_yaw_step_deg,
                trusted_max_icp_error_cm=config.trusted_max_icp_error_cm,
                footprint_clearance_cm=config.footprint_clearance_cm,
                wall_rotation_adaptation=config.wall_rotation_adaptation,
                wall_low_pass_ratio=config.wall_low_pass_ratio,
            ),
            on_state_changed=self._on_navigation_state,
        )
        # Stable public aliases retained for status/reporting and task-specific
        # map overlays.  Navigation work itself is owned by the component.
        self.calibrator = self.coordinate_navigation.calibrator
        self.radar = self.coordinate_navigation.radar
        self.trusted_map = self.coordinate_navigation.trusted_map
        self.protocol = None
        self.link = None
        self.fleet_node = None
        if fleet_bus:
            self.link = SerialCommunicationDriver(
                port=config.link_port,
                on_bytes=self._on_fleet_frame,
                on_connected=lambda: LOG.info(
                    "FleetBus HC-14 connected on %s", config.link_port
                ),
                on_disconnected=lambda error: LOG.warning(
                    "FleetBus HC-14 disconnected: %s", error
                ),
                on_callback_error=lambda error: LOG.error(
                    "FleetBus HC-14 callback failed: %s", error
                ),
            )
            self.fleet_node = FleetCarNode(
                writer=self.link.write,
                state_provider=self._fleet_state,
                on_set_coordinate_frame=self._fleet_set_coordinate_frame,
                on_navigate=self._fleet_navigate,
                on_stop=self._fleet_stop,
                on_start_mapping=self._fleet_start_mapping,
                on_set_alarm=self._fleet_set_alarm,
            )
            self._initialize_grid_rescue()
        elif hmac_key is not None:
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

    @property
    def navigation(self):
        return self.coordinate_navigation.navigation

    @navigation.setter
    def navigation(self, value) -> None:
        self.coordinate_navigation.navigation = value

    def run(self) -> None:
        """Calibrate while stationary, then start navigation and command link."""

        LOG.info(
            "application starting pid=%d python=%s; vehicle must remain stationary "
            "during D500 calibration",
            os.getpid(),
            sys.version.split()[0],
        )
        if self.fleet_node is not None:
            self.fleet_node.start()
            assert self.link is not None
            self.link.start()
            LOG.info(
                "FleetBus command input enabled; waiting for CAR_START_MAPPING "
                "while the vehicle remains stationary"
            )
        else:
            calibration = self.coordinate_navigation.start()
            with self._lock:
                self._ready = True
            if self.link is not None:
                self.link.start()
                LOG.info("HC-14 authenticated legacy command input enabled")
            self._print_map_ready(calibration)
        self._start_console_if_available()
        self._stop_event.wait()

    def request_stop(self) -> None:
        LOG.info("application stop requested")
        self.coordinate_navigation.request_stop()
        self._stop_event.set()

    def close(self) -> None:
        LOG.info("application closing")
        with self._lock:
            self._ready = False
        if self.rescue_controller is not None:
            self.rescue_controller.stop()
        with self._rescue_condition:
            self._rescue_condition.notify_all()
        try:
            self.coordinate_navigation.request_stop()
            if self._alarm is not None:
                try:
                    self._alarm.off()
                except AlarmGPIOError as exc:
                    LOG.warning("could not silence alarm during shutdown: %s", exc)
            if self.fleet_node is not None:
                self.fleet_node.close()
            if self.link is not None:
                self.link.close()
            mapping_thread = self._fleet_mapping_thread
            if mapping_thread is not None:
                mapping_thread.join(timeout=2.0)
            if self.rescue_controller is not None:
                self.rescue_controller.wait(timeout=2.0)
        finally:
            self.coordinate_navigation.close()
        LOG.info("application closed; hardware outputs are safe")

    def _refresh_trusted_grid(self, *, now: float | None = None, force: bool = False) -> bool:
        return self.coordinate_navigation.refresh_map(now=now, force=force)

    def _install_navigation_grid(self, grid) -> bool:
        """Task-layer hook for conservative, obstacle-only map overlays."""
        with self._semantic_lock:
            step = self._semantic_step
        if step is not None:
            _current, _target, blocked = step
            grid = overlay_blocked_terrain(grid, blocked)
        return self.navigation.set_map(grid)

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
            if (
                self._active_receipt is not None
                or self._console_mission_active
                or self._fleet_mission_active
            ):
                raise NavigationCommandRejected(RejectReason.TASK_BUSY, "navigation already active")
            self._active_receipt = receipt
        try:
            self.coordinate_navigation.navigate(goal)
        except CoordinateGoalRejected as exc:
            with self._lock:
                if self._active_receipt == receipt:
                    self._active_receipt = None
            reason = (
                RejectReason.TASK_BUSY
                if exc.reason in (
                    CoordinateGoalRejectReason.NOT_READY,
                    CoordinateGoalRejectReason.BUSY,
                )
                else RejectReason.BAD_PAYLOAD
            )
            raise NavigationCommandRejected(reason, str(exc)) from exc
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
            fleet_mission = self._fleet_mission_active
            self._fleet_mission_active = False
            receipt = self._active_receipt
            self._active_receipt = None
            console_mission = self._console_mission_active
            self._console_mission_active = False
        ready_reason = "ready for next goal; startup map and origin retained"
        if fleet_mission:
            if self.fleet_node is not None:
                self.fleet_node.set_active_command_result(
                    FleetCommandResult(
                        FleetAckStatus.COMPLETED
                        if state is NavigationState.ARRIVED
                        else FleetAckStatus.FAILED,
                        FleetAckReason.NONE
                        if state is NavigationState.ARRIVED
                        else FleetAckReason.INTERNAL_ERROR,
                        "" if state is NavigationState.ARRIVED else reason,
                    )
                )
            self.navigation.cancel(reason=ready_reason)
            LOG.info("terminal FleetBus mission published; startup origin retained")
            self._notify_rescue_terminal(state)
            return
        if console_mission:
            if state is NavigationState.ARRIVED:
                self._console_print("已到达目标，位置与可选车头方向均满足容差。")
            else:
                self._console_print(f"任务失败：{state.value}，{reason}")
            self.navigation.cancel(reason=ready_reason)
            self._console_print("可继续输入下一目标；启动原点和地图坐标系保持不变。")
            LOG.info("terminal mission reset for next SSH goal; startup origin retained")
            self._notify_rescue_terminal(state)
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
        self._notify_rescue_terminal(state)

    def _notify_rescue_terminal(self, state: NavigationState) -> None:
        with self._rescue_condition:
            self._rescue_terminal_state = state
            self._rescue_terminal_generation += 1
            self._rescue_condition.notify_all()

    def _log_navigation_blocked(self, reason: str) -> None:
        """Record evidence that distinguishes a real obstacle from map drift."""

        pose = self.navigation.pose
        with self._lock:
            grid = self.trusted_map.grid
            calibration = self.trusted_map.calibration
            trusted_pose = self.trusted_map.last_pose
            trusted_time = self.trusted_map.last_pose_time
            trusted_rejection = self.trusted_map.last_rejection
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
        corners = self.trusted_map.vehicle_footprint_corners(radar_pose)
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
            self.trusted_map.trusted_cell_count(),
            len(self.radar.global_map.cells(min_hits=self.config.map_min_hits)),
        )

    def _send_or_queue_status(self, frame: bytes) -> None:
        with self._lock:
            if self._handling_link_frame:
                self._post_command_acks.append(frame)
                return
        self._send_frame(frame)

    def _on_fleet_frame(self, frame: bytes) -> None:
        if self.fleet_node is not None:
            self.fleet_node.feed_frame(frame)

    def _fleet_state(self) -> CarFleetState:
        with self._lock:
            alignment = self._fleet_alignment
            calibration = self.trusted_map.calibration
            ready = self._ready
        pose = self.navigation.pose
        pose_valid = (
            pose is not None
            and ready
            and time.monotonic() - pose.timestamp_s
            <= self.navigation.config.localization_timeout_s
        )
        x_cm = y_cm = heading_cdeg = 0
        if pose is not None:
            radar_pose = Pose2D(
                pose.x_cm, pose.y_cm, (-pose.heading_deg) % 360.0
            )
            if alignment is not None:
                radar_pose = alignment.pose_to_global(radar_pose)
            x_cm = round(radar_pose.x_cm)
            y_cm = round(radar_pose.y_cm)
            heading_cdeg = round(((-radar_pose.yaw_cw_deg) % 360.0) * 100) % 36000
        flags = 0
        if pose_valid:
            flags |= int(FleetNodeFlags.POSE_VALID)
        if ready:
            flags |= int(FleetNodeFlags.READY | FleetNodeFlags.MAP_READY)
        if alignment is not None:
            flags |= int(FleetNodeFlags.COORDINATE_FRAME_SYNCED)
        if self.navigation.state not in (
            NavigationState.IDLE,
            NavigationState.ARRIVED,
            NavigationState.FAILED,
            NavigationState.BLOCKED,
        ):
            flags |= int(FleetNodeFlags.BUSY | FleetNodeFlags.ARMED_OR_MOTOR_ACTIVE)
        corners = ()
        if calibration is not None:
            corners = tuple(
                (round(x_cm), round(y_cm))
                if alignment is None
                else tuple(round(value) for value in alignment.point_to_global((x_cm, y_cm)))
                for x_cm, y_cm in calibration.field_polygon_cm
            )
        path = self.navigation.path
        path_points = ()
        if path is not None:
            local_points = tuple(
                (round(point.x_cm), round(point.y_cm)) for point in path.points
            )
            path_points = (
                local_points
                if alignment is None
                else tuple(
                    tuple(round(value) for value in alignment.point_to_global(point))
                    for point in local_points
                )
            )
        states = list(NavigationState)
        return CarFleetState(
            flags,
            round((time.monotonic() - self._started_at) * 1000) & 0xFFFFFFFF,
            x_cm,
            y_cm,
            heading_cdeg,
            operation_state=states.index(self.navigation.state),
            pose_quality=3 if pose_valid else 0,
            map_revision=self.navigation.map_revision,
            field_corners=corners,
            path_revision=self.navigation.map_revision,
            path_points=path_points,
        )

    def _fleet_start_mapping(self, request_seq: int) -> FleetCommandResult:
        with self._lock:
            if self._ready:
                return FleetCommandResult(FleetAckStatus.COMPLETED)
            if self._fleet_mapping_active:
                return FleetCommandResult(
                    FleetAckStatus.REJECTED, FleetAckReason.BUSY
                )
            self._fleet_mapping_active = True
            self._fleet_mapping_thread = threading.Thread(
                target=self._run_fleet_mapping,
                args=(request_seq,),
                name="fleet-car-mapping",
                daemon=True,
            )
            self._fleet_mapping_thread.start()
        LOG.info(
            "CAR_START_MAPPING accepted seq=%d; starting stationary D500 calibration",
            request_seq,
        )
        return FleetCommandResult(FleetAckStatus.ACCEPTED)

    def _fleet_set_alarm(self, active: bool) -> FleetCommandResult:
        try:
            self._alarm = alarm_on() if active else alarm_off()
        except AlarmGPIOError as exc:
            LOG.error("could not set sound/light alarm active=%s: %s", active, exc)
            return FleetCommandResult(
                FleetAckStatus.FAILED,
                FleetAckReason.INTERNAL_ERROR,
                str(exc),
            )
        LOG.info("sound/light alarm %s by FleetBus", "on" if active else "off")
        return FleetCommandResult(FleetAckStatus.COMPLETED)

    def _initialize_grid_rescue(self) -> None:
        if self.fleet_node is None:
            return
        layout = GridLayout()
        planner = AdjacentGridRescuePlanner(
            water_terrain_codes=terrain_codes(WATER_PICKUP_TERRAINS),
            wildfire_terrain_codes=terrain_codes(WILDFIRE_TARGET_TERRAINS),
            forbidden_terrain_codes=terrain_codes(FORBIDDEN_TERRAINS),
            layout=layout,
        )
        self.coordinate_navigation.map_installer = self._install_navigation_grid
        pivot = InPlaceDifferentialTurn(
            self.navigation.drive,
            pose_provider=lambda: self.navigation.pose,
            on_motion_changed=self.radar.set_motion_hint,
            stop_requested=lambda: (
                self._stop_event.is_set()
                or (
                    self.rescue_controller is not None
                    and self.rescue_controller.stop_requested
                )
            ),
        )
        grid_navigator = AdjacentGridNavigator(
            pivot,
            navigate_to=self._navigate_grid_pose,
            layout=layout,
        )
        self.rescue_controller = GridRescueMissionController(
            planner,
            navigate=self._navigate_rescue_cell,
            move_adjacent=grid_navigator.move,
            set_step_overlay=self._set_semantic_step,
            clear_overlay=self._clear_semantic_overlay,
            on_result=self._on_rescue_result,
            hold_seconds=3.0,
        )
        self.fleet_node.set_disaster_handler(self._fleet_disaster_rescue)
        LOG.info(
            "adjacent-grid rescue enabled water=%s wildfire=%s forbidden=%s",
            WATER_PICKUP_TERRAINS,
            WILDFIRE_TARGET_TERRAINS,
            FORBIDDEN_TERRAINS,
        )

    def _fleet_disaster_rescue(
        self, command: DisasterRescueCommand
    ) -> FleetCommandResult:
        controller = self.rescue_controller
        with self._lock:
            if (
                controller is None
                or not self._ready
                or self._fleet_alignment is None
                or self.trusted_map.calibration is None
            ):
                return FleetCommandResult(
                    FleetAckStatus.REJECTED, FleetAckReason.NOT_READY
                )
            if (
                self._rescue_mission_active
                or self._fleet_mission_active
                or self._console_mission_active
                or self._active_receipt is not None
            ):
                return FleetCommandResult(
                    FleetAckStatus.REJECTED, FleetAckReason.BUSY
                )
            self._rescue_mission_active = True
            self._rescue_request_seq = (
                None
                if self.fleet_node is None
                else self.fleet_node.active_command_seq
            )
        result = controller.submit(command)
        if result.status != FleetAckStatus.ACCEPTED:
            with self._lock:
                self._rescue_mission_active = False
                self._rescue_request_seq = None
        return result

    def _on_rescue_result(self, result: FleetCommandResult) -> None:
        with self._lock:
            self._rescue_mission_active = False
            request_seq = self._rescue_request_seq
            self._rescue_request_seq = None
        if self.fleet_node is not None:
            self.fleet_node.set_active_command_result(result, request_seq)

    def _set_semantic_step(self, current, target, blocked) -> None:
        with self._semantic_lock:
            self._semantic_step = (current, target, blocked)
        self._refresh_trusted_grid(force=True)
        LOG.info(
            "semantic rescue corridor current=%s target=%s blocked=%s",
            current,
            target,
            sorted(blocked),
        )

    def _clear_semantic_overlay(self) -> None:
        with self._semantic_lock:
            self._semantic_step = None
        self._refresh_trusted_grid(force=True)

    def _navigate_rescue_cell(self, cell) -> bool:
        layout = GridLayout()
        if cell is None:
            x_cm, y_cm = layout.start_point_cm
        else:
            x_cm, y_cm = layout.centre(cell)
        return self._navigate_grid_pose(x_cm, y_cm, None)

    def _navigate_grid_pose(
        self, x_cm: float, y_cm: float, heading: float | None
    ) -> bool:
        controller = self.rescue_controller
        if (
            controller is None
            or controller.stop_requested
            or self._stop_event.is_set()
        ):
            return False
        with self._rescue_condition:
            generation = self._rescue_terminal_generation
        try:
            self._submit_console_goal(NavigationGoal(x_cm, y_cm, heading))
        except (
            CoordinateGoalRejected,
            NavigationCommandRejected,
            NavigationError,
            ValueError,
        ) as exc:
            LOG.error(
                "rescue waypoint rejected pose=(%.1f,%.1f,%s): %s",
                x_cm,
                y_cm,
                "none" if heading is None else f"{heading:.1f}",
                exc,
            )
            return False
        deadline = time.monotonic() + RESCUE_NAVIGATION_STEP_TIMEOUT_S
        with self._rescue_condition:
            while self._rescue_terminal_generation == generation:
                if controller.stop_requested or self._stop_event.is_set():
                    self._cancel_from_console()
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._cancel_from_console()
                    LOG.error(
                        "rescue waypoint timeout pose=(%.1f,%.1f,%s)",
                        x_cm,
                        y_cm,
                        heading,
                    )
                    return False
                self._rescue_condition.wait(min(0.2, remaining))
            return self._rescue_terminal_state is NavigationState.ARRIVED

    def _run_fleet_mapping(self, request_seq: int) -> None:
        try:
            calibration = self.coordinate_navigation.start()
            with self._lock:
                self._ready = True
            self._print_map_ready(calibration)
            result = FleetCommandResult(FleetAckStatus.COMPLETED)
        except Exception as exc:
            LOG.exception("FleetBus mapping startup failed: %s", exc)
            result = FleetCommandResult(
                FleetAckStatus.FAILED,
                FleetAckReason.INTERNAL_ERROR,
                str(exc),
            )
        finally:
            with self._lock:
                self._fleet_mapping_active = False
        if self.fleet_node is not None:
            self.fleet_node.set_active_command_result(result, request_seq)

    def _fleet_set_coordinate_frame(
        self, command: CoordinateFrameCommand
    ) -> FleetCommandResult:
        with self._lock:
            if not self._ready or self.trusted_map.calibration is None:
                return FleetCommandResult(
                    FleetAckStatus.REJECTED, FleetAckReason.NOT_READY
                )
            if self._fleet_alignment is not None:
                return FleetCommandResult(
                    FleetAckStatus.REJECTED,
                    FleetAckReason.ALREADY_SYNCHRONIZED,
                )
            if self.navigation.state not in (
                NavigationState.IDLE,
                NavigationState.ARRIVED,
                NavigationState.FAILED,
                NavigationState.BLOCKED,
            ):
                return FleetCommandResult(
                    FleetAckStatus.REJECTED, FleetAckReason.BUSY
                )
            self._fleet_alignment = DroneGlobalAlignment(
                float(command.origin_x_cm),
                float(command.origin_y_cm),
                (-command.startup_x_heading_cdeg / 100.0) % 360.0,
            )
        LOG.info(
            "FleetBus world frame synchronized origin=(%d,%d) heading_ccw=%.2f",
            command.origin_x_cm,
            command.origin_y_cm,
            command.startup_x_heading_cdeg / 100.0,
        )
        return FleetCommandResult(FleetAckStatus.COMPLETED)

    def _fleet_navigate(
        self, command: CarNavigateCommand
    ) -> FleetCommandResult:
        with self._lock:
            alignment = self._fleet_alignment
        if alignment is None:
            return FleetCommandResult(
                FleetAckStatus.REJECTED, FleetAckReason.NOT_READY
            )
        local_x, local_y = alignment.point_to_local(
            (float(command.x_cm), float(command.y_cm))
        )
        local_heading = None
        if command.heading_cdeg is not None:
            world_yaw_cw = (-command.heading_cdeg / 100.0) % 360.0
            local_yaw_cw = (
                world_yaw_cw - alignment.yaw_offset_cw_deg
            ) % 360.0
            local_heading = (-local_yaw_cw) % 360.0
        with self._lock:
            if (
                self._rescue_mission_active
                or self._fleet_mission_active
                or self._console_mission_active
                or self._active_receipt is not None
            ):
                return FleetCommandResult(
                    FleetAckStatus.REJECTED, FleetAckReason.BUSY
                )
            self._fleet_mission_active = True
        try:
            self.coordinate_navigation.navigate(
                NavigationGoal(local_x, local_y, local_heading)
            )
        except CoordinateGoalRejected as exc:
            with self._lock:
                self._fleet_mission_active = False
            reason = (
                FleetAckReason.BUSY
                if exc.reason in (
                    CoordinateGoalRejectReason.NOT_READY,
                    CoordinateGoalRejectReason.BUSY,
                )
                else FleetAckReason.OUTSIDE_FIELD
            )
            return FleetCommandResult(
                FleetAckStatus.REJECTED, reason, str(exc)
            )
        except (NavigationError, ValueError) as exc:
            with self._lock:
                self._fleet_mission_active = False
            return FleetCommandResult(
                FleetAckStatus.REJECTED,
                FleetAckReason.LOCALIZATION_INVALID,
                str(exc),
            )
        except BaseException:
            with self._lock:
                self._fleet_mission_active = False
            raise
        return FleetCommandResult(FleetAckStatus.ACCEPTED)

    def _fleet_stop(self) -> FleetCommandResult:
        if self.rescue_controller is not None:
            self.rescue_controller.stop()
        with self._lock:
            self._fleet_mission_active = False
            self._rescue_mission_active = False
            mapping_active = self._fleet_mapping_active
        if mapping_active:
            self.coordinate_navigation.request_stop()
        self._cancel_from_console()
        with self._rescue_condition:
            self._rescue_condition.notify_all()
        return FleetCommandResult(FleetAckStatus.COMPLETED)

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
        returning_to_start_without_heading = (
            goal.final_heading_deg is None
            and math.hypot(goal.x_cm, goal.y_cm) <= 1.0
        )
        with self._lock:
            if (
                self._console_mission_active
                or self._active_receipt is not None
                or self._fleet_mission_active
            ):
                raise NavigationCommandRejected(RejectReason.TASK_BUSY, "已有任务，先输入 stop")
            self._console_mission_active = True
        try:
            self.coordinate_navigation.navigate(goal)
        except CoordinateGoalRejected as exc:
            with self._lock:
                self._console_mission_active = False
            reason = (
                RejectReason.TASK_BUSY
                if exc.reason in (
                    CoordinateGoalRejectReason.NOT_READY,
                    CoordinateGoalRejectReason.BUSY,
                )
                else RejectReason.BAD_PAYLOAD
            )
            raise NavigationCommandRejected(reason, str(exc)) from exc
        except BaseException:
            with self._lock:
                self._console_mission_active = False
            raise
        heading = "不限定" if goal.final_heading_deg is None else f"{goal.final_heading_deg:.0f}°"
        self._console_print(
            f"已接受目标：x={goal.x_cm:.1f}cm y={goal.y_cm:.1f}cm heading={heading}；"
            "正在自主规划并使用雷达持续纠偏。"
        )
        if returning_to_start_without_heading:
            self._console_print(
                "注意：0 0 不约束最终车头方向；若要回到启动位置和启动朝向，请输入 0 0 0。"
            )
            LOG.warning(
                "return-to-origin goal has no final heading; planner may arrive "
                "with any feasible vehicle orientation"
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
    parser.add_argument(
        "--fleet-bus",
        action="store_true",
        help="use FleetBus V1 instead of legacy authenticated HC-14 commands",
    )
    parser.add_argument("--radar-x-cm", type=float, default=0.0)
    parser.add_argument("--radar-y-cm", type=float, default=0.0)
    parser.add_argument("--radar-yaw-cw-deg", type=float, default=0.0)
    parser.add_argument("--startup-scans", type=int, default=20)
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
    parser.add_argument(
        "--log-level",
        choices=("OFF", "DEBUG", "INFO", "WARNING", "ERROR"),
        default="OFF",
        help=(
            "optional SSH terminal logging; default OFF keeps coordinate input clean "
            "(the rotating file log always records DEBUG)"
        ),
    )
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
        if args.fleet_bus:
            LOG.info("FleetBus mode selected; legacy HMAC protocol is disabled")
        elif os.environ.get("GROUND_STATION_HMAC_KEY_HEX", "").strip():
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
        app = CarMainApplication(config, hmac_key=key, fleet_bus=args.fleet_bus)

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
