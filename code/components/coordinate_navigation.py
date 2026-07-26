"""Coordinate-level D500 navigation runtime for the production car.

The public operation is :meth:`CoordinateNavigation.navigate_to`: coordinates
are centimetres in the startup vehicle frame (``+X`` forward, ``+Y`` left) and
heading is counter-clockwise from the startup vehicle heading.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import logging
import math
import threading
import time
from typing import Callable

from .navigation import (
    Navigation,
    NavigationConfig,
    NavigationError,
    NavigationGoal,
    NavigationPose,
    NavigationState,
    OccupancyGrid,
    PurePursuitConfig,
    PurePursuitController,
)
from .radar_driver import (
    DEFAULT_D500_PORT,
    D500RadarComponent,
    Pose2D,
    RadarLocalizationUpdate,
    RadarMount,
    RadarScan,
    RectangleFieldCalibration,
    RectangleFieldCalibrator,
    WallFusionConfig,
    WallLineConfig,
    rebase_calibration_to_start_pose,
    scan_points_in_drone_global,
)
from .trusted_navigation_map import (
    TrustedNavigationMap,
    TrustedNavigationMapConfig,
)


LOG = logging.getLogger(__name__)


class CoordinateGoalRejectReason(Enum):
    NOT_READY = "not_ready"
    BUSY = "busy"
    OUTSIDE_FIELD = "outside_field"
    UNSAFE_FOOTPRINT = "unsafe_footprint"


class CoordinateGoalRejected(NavigationError):
    """A coordinate command was safely rejected before vehicle motion."""

    def __init__(self, reason: CoordinateGoalRejectReason, message: str) -> None:
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True, slots=True)
class CoordinateNavigationConfig:
    radar_port: str = DEFAULT_D500_PORT
    radar_mount: RadarMount = RadarMount()
    startup_scan_count: int = 20
    calibration_timeout_s: float = 30.0
    allow_reverse: bool = True
    allow_in_place_rotation: bool = False
    cruise_speed_cm_s: float = 30.0
    reverse_speed_cm_s: float = 15.0
    max_cruise_speed_cm_s: float = 100.0
    wheel_speed_headroom: float = 1.20
    map_resolution_cm: float = 5.0
    map_margin_cm: float = 15.0
    map_update_interval_s: float = 0.5
    map_min_hits: int = 2
    trusted_max_pose_step_cm: float = 25.0
    trusted_max_yaw_step_deg: float = 15.0
    trusted_max_icp_error_cm: float = 10.0
    footprint_clearance_cm: float = 2.0
    wall_rotation_adaptation: bool = True
    wall_low_pass_ratio: float = 0.20

    def __post_init__(self) -> None:
        if not self.radar_port:
            raise ValueError("radar_port cannot be empty")
        if self.startup_scan_count <= 0 or self.calibration_timeout_s <= 0:
            raise ValueError("startup scan count and timeout must be positive")
        if not 0.0 < self.cruise_speed_cm_s <= self.max_cruise_speed_cm_s:
            raise ValueError("cruise_speed_cm_s is outside the configured range")
        if not 0.0 < self.reverse_speed_cm_s <= self.max_cruise_speed_cm_s:
            raise ValueError("reverse_speed_cm_s is outside the configured range")
        if self.wheel_speed_headroom < 1.0:
            raise ValueError("wheel_speed_headroom must be at least 1")
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


class CoordinateNavigation:
    """Own radar localization, trusted mapping, planning and vehicle control."""

    def __init__(
        self,
        config: CoordinateNavigationConfig = CoordinateNavigationConfig(),
        *,
        on_state_changed: Callable[[NavigationState, str], None] | None = None,
        map_installer: Callable[[OccupancyGrid], bool] | None = None,
    ) -> None:
        self.config = config
        self.on_state_changed = on_state_changed
        self.map_installer = map_installer
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._scan_event = threading.Event()
        self._startup_scans: list[RadarScan] = []
        self._ready = False

        if (
            abs(config.radar_mount.x_forward_cm) < 1e-9
            and abs(config.radar_mount.y_left_cm) < 1e-9
            and abs(config.radar_mount.yaw_cw_deg) < 1e-9
        ):
            LOG.warning(
                "radar mount is (0,0,0); this is valid only when the D500 "
                "origin is at the rear axle and its zero angle faces forward"
            )

        self.calibrator = RectangleFieldCalibrator(mount=config.radar_mount)
        self.radar = D500RadarComponent(
            port=config.radar_port,
            mount=config.radar_mount,
            on_update=self._on_radar_update,
            on_connected=lambda: LOG.info("D500 connected on %s", config.radar_port),
            on_disconnected=lambda error: LOG.warning("D500 disconnected: %s", error),
        )
        self.radar.set_motion_hint(False)

        forward_mm_s = config.cruise_speed_cm_s * 10.0
        reverse_mm_s = config.reverse_speed_cm_s * 10.0
        highest_mm_s = max(forward_mm_s, reverse_mm_s)
        max_wheel_mm_s = max(
            300.0,
            highest_mm_s * config.wheel_speed_headroom,
        )
        pursuit_config = PurePursuitConfig(
            cruise_speed_mm_s=forward_mm_s,
            max_speed_mm_s=max(150.0, highest_mm_s),
            approach_speed_mm_s=min(80.0, highest_mm_s),
            reverse_speed_mm_s=reverse_mm_s,
            min_lookahead_cm=20.0,
            max_lookahead_cm=50.0,
            slowdown_distance_cm=60.0,
        )
        self.navigation = Navigation(
            config=NavigationConfig(allow_reverse=config.allow_reverse),
            controller=PurePursuitController(config=pursuit_config),
            max_wheel_speed_mm_s=max_wheel_mm_s,
            allow_in_place_rotation=config.allow_in_place_rotation,
            on_state_changed=self._on_navigation_state,
            on_motion_changed=self.radar.set_motion_hint,
        )
        safety_margin_cm = self.navigation.planner.config.safety_margin_cm
        self.trusted_map = TrustedNavigationMap(
            TrustedNavigationMapConfig(
                resolution_cm=config.map_resolution_cm,
                margin_cm=config.map_margin_cm,
                update_interval_s=config.map_update_interval_s,
                min_hits=config.map_min_hits,
                max_pose_step_cm=config.trusted_max_pose_step_cm,
                max_yaw_step_deg=config.trusted_max_yaw_step_deg,
                max_icp_error_cm=config.trusted_max_icp_error_cm,
                footprint_clearance_cm=config.footprint_clearance_cm,
                navigation_safety_margin_cm=safety_margin_cm,
            ),
            self.navigation.geometry,
        )
        LOG.info(
            "coordinate navigation configured forward=%.1fcm/s reverse=%.1fcm/s "
            "max_wheel=%.1fcm/s allow_reverse=%s",
            config.cruise_speed_cm_s,
            config.reverse_speed_cm_s,
            max_wheel_mm_s / 10.0,
            config.allow_reverse,
        )

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready

    @property
    def calibration(self) -> RectangleFieldCalibration | None:
        return self.trusted_map.calibration

    @property
    def pose(self) -> NavigationPose | None:
        return self.navigation.pose

    @property
    def state(self) -> NavigationState:
        return self.navigation.state

    def start(self) -> RectangleFieldCalibration:
        """Calibrate while stationary and start the navigation control loop."""

        with self._lock:
            if self._ready:
                calibration = self.trusted_map.calibration
                if calibration is None:
                    raise RuntimeError("coordinate navigation has inconsistent state")
                return calibration
            self._stop_event.clear()
            self._startup_scans.clear()

        self.radar.start()
        if not self.radar.serial.wait_connected(
            min(3.0, self.config.calibration_timeout_s)
        ):
            raise RuntimeError(
                f"D500 UART {self.config.radar_port} could not be opened; verify "
                "UART6-M1, Pin 21 RX wiring and dialout permission"
            )
        fitted, scans = self._wait_for_rectangle_calibration()
        calibration = rebase_calibration_to_start_pose(fitted)

        # Do not let scans race with the odometry/alignment origin change.
        self.radar.close()
        self.radar.assembler.reset()
        self.radar.odometry.reset(Pose2D())
        self.radar.global_map.clear()
        self.radar.alignment = calibration.local_to_global
        fusion_config = WallFusionConfig.car_slow_drift(
            position_gain=self.config.wall_low_pass_ratio,
        )
        self.radar.enable_wall_fusion(
            calibration.wall_reference,
            line_config=WallLineConfig(
                rotation_adaptation=self.config.wall_rotation_adaptation,
            ),
            fusion_config=fusion_config,
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
        grid = self.trusted_map.initialize(
            calibration,
            startup_points,
            pose=Pose2D(),
        )
        self.navigation.update_pose(NavigationPose(0.0, 0.0, 0.0))
        self._install_navigation_grid(grid)
        self.navigation.start()
        with self._lock:
            self._ready = True
        self.radar.start()
        LOG.info(
            "coordinate navigation ready startup=(0,0,0deg) bounds="
            "x=[%.1f,%.1f] y=[%.1f,%.1f]cm",
            calibration.min_x_cm,
            calibration.max_x_cm,
            calibration.min_y_cm,
            calibration.max_y_cm,
        )
        return calibration

    def navigate_to(
        self,
        x_cm: float,
        y_cm: float,
        final_heading_deg: float | None = None,
        *,
        position_tolerance_cm: float = 5.0,
        heading_tolerance_deg: float = 8.0,
    ) -> NavigationGoal:
        """Plan and drive to a pose relative to startup position and heading."""

        goal = NavigationGoal(
            x_cm,
            y_cm,
            final_heading_deg,
            position_tolerance_cm,
            heading_tolerance_deg,
        )
        with self._lock:
            calibration = self.trusted_map.calibration
            if not self._ready or calibration is None:
                raise CoordinateGoalRejected(
                    CoordinateGoalRejectReason.NOT_READY,
                    "startup calibration is incomplete",
                )
            if getattr(self.navigation, "active", False):
                raise CoordinateGoalRejected(
                    CoordinateGoalRejectReason.BUSY,
                    "navigation already active",
                )
        if not calibration.contains_point(goal.x_cm, goal.y_cm):
            raise CoordinateGoalRejected(
                CoordinateGoalRejectReason.OUTSIDE_FIELD,
                "goal lies outside the startup field",
            )
        self.refresh_map(force=True)
        if not self.trusted_map.goal_has_safe_vehicle_footprint(goal):
            raise CoordinateGoalRejected(
                CoordinateGoalRejectReason.UNSAFE_FOOTPRINT,
                "goal vehicle footprint intersects an obstacle or field boundary",
            )
        coordinate_entry = getattr(self.navigation, "navigate_to", None)
        if coordinate_entry is None:
            # Small test/task adapters written for the former two-call API can
            # still be injected without weakening the production entry point.
            self.navigation.set_goal(goal)
            self.navigation.start_navigation()
        else:
            coordinate_entry(
                goal.x_cm,
                goal.y_cm,
                goal.final_heading_deg,
                position_tolerance_cm=goal.position_tolerance_cm,
                heading_tolerance_deg=goal.heading_tolerance_deg,
            )
        LOG.info(
            "coordinate goal accepted x=%.2f y=%.2f heading=%s",
            goal.x_cm,
            goal.y_cm,
            "none" if goal.final_heading_deg is None else f"{goal.final_heading_deg:.2f}",
        )
        return goal

    def navigate(self, goal: NavigationGoal) -> NavigationGoal:
        """NavigationGoal variant of :meth:`navigate_to`."""

        if not isinstance(goal, NavigationGoal):
            raise TypeError("goal must be a NavigationGoal")
        return self.navigate_to(
            goal.x_cm,
            goal.y_cm,
            goal.final_heading_deg,
            position_tolerance_cm=goal.position_tolerance_cm,
            heading_tolerance_deg=goal.heading_tolerance_deg,
        )

    def cancel(self, *, reason: str = "navigation cancelled") -> None:
        self.navigation.cancel(reason=reason)

    def request_stop(self) -> None:
        """Interrupt startup calibration without touching another input layer."""

        self._stop_event.set()
        self._scan_event.set()

    def close(self) -> None:
        self.request_stop()
        with self._lock:
            self._ready = False
        try:
            self.navigation.close()
        finally:
            self.radar.close()

    def refresh_map(self, *, now: float | None = None, force: bool = False) -> bool:
        grid = self.trusted_map.refresh_grid(now=now, force=force)
        if grid is None:
            return False
        return self._install_navigation_grid(grid)

    def _install_navigation_grid(self, grid: OccupancyGrid) -> bool:
        installer = self.map_installer
        return self.navigation.set_map(grid) if installer is None else installer(grid)

    def _wait_for_rectangle_calibration(
        self,
    ) -> tuple[RectangleFieldCalibration, tuple[RadarScan, ...]]:
        deadline = time.monotonic() + self.config.calibration_timeout_s
        last_error = (
            f"D500 UART {self.config.radar_port} is open but no complete scan arrived"
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
                return self.calibrator.calibrate(scans), scans
            except (ValueError, RuntimeError) as exc:
                if str(exc) != last_error:
                    LOG.warning("rectangle calibration retry: %s", exc)
                    last_error = str(exc)
        raise RuntimeError(f"rectangle field calibration timed out: {last_error}")

    def _on_radar_update(self, update: RadarLocalizationUpdate) -> None:
        with self._lock:
            ready = self._ready
            if not ready:
                self._startup_scans.append(update.scan)
                limit = max(
                    self.config.startup_scan_count * 2,
                    self.config.startup_scan_count,
                )
                del self._startup_scans[:-limit]
                self._scan_event.set()
        self._log_radar_update(update, phase="navigation" if ready else "calibration")
        if not ready:
            return

        now = time.monotonic()
        rejection = self.trusted_map.rejection_reason(update)
        if rejection is not None:
            self.trusted_map.record_rejection(rejection)
            LOG.warning(
                "trusted radar localization rejected; pose/map retained: %s",
                rejection,
            )
            if (
                rejection.startswith("vehicle footprint outside fitted field")
                and self.navigation.active
            ):
                self.navigation.fail_safe_stop(
                    f"immediate field-boundary safety stop: {rejection}"
                )
            return

        if not self.navigation.update_from_radar(update) or update.global_pose is None:
            return
        result = self.trusted_map.ingest(update, now=now)
        if result.wall_hard_rejected:
            LOG.warning(
                "trusted map skipped wall-rejected scan reason=%r",
                result.wall_reason,
            )
        LOG.debug(
            "trusted map scan raw=%d retained=%d purged=%d hard_rejected=%s",
            result.raw_points,
            result.retained_points,
            result.purged_self_cells,
            result.wall_hard_rejected,
        )
        if result.refreshed_grid is not None:
            self._install_navigation_grid(result.refreshed_grid)

    def _on_navigation_state(self, state: NavigationState, reason: str) -> None:
        callback = self.on_state_changed
        if callback is not None:
            callback(state, reason)

    @staticmethod
    def _log_radar_update(
        update: RadarLocalizationUpdate,
        *,
        phase: str,
    ) -> None:
        odometry = update.odometry
        pose = update.global_pose
        icp = odometry.icp
        wall = update.wall_fusion
        LOG.debug(
            "radar phase=%s scan_ts_ms=%d points=%d accepted=%s initialized=%s "
            "rejection=%r local_pose=(%.3f,%.3f,%.3f) global_pose=%s "
            "icp_error_cm=%s icp_matches=%s wall_status=%s wall_reason=%r "
            "wall_correction=(%.3f,%.3f,%.3f) "
            "wall_residual=(%.3f,%.3f,%.3f) "
            "global_points=%d",
            phase,
            update.scan.timestamp_ms,
            len(update.scan.points),
            odometry.accepted,
            odometry.initialized,
            odometry.rejection_reason,
            odometry.pose.x_cm,
            odometry.pose.y_cm,
            odometry.pose.yaw_cw_deg,
            "none"
            if pose is None
            else f"({pose.x_cm:.3f},{pose.y_cm:.3f},{pose.yaw_cw_deg:.3f})",
            "none" if icp is None else f"{icp.mean_error_cm:.4f}",
            "none" if icp is None else icp.matched_points,
            "none" if wall is None else wall.status.value,
            None if wall is None else wall.reason,
            0.0 if wall is None else wall.correction_x_cm,
            0.0 if wall is None else wall.correction_y_cm,
            0.0 if wall is None else wall.correction_yaw_deg,
            0.0 if wall is None else wall.residual_x_cm,
            0.0 if wall is None else wall.residual_y_cm,
            0.0 if wall is None else wall.residual_yaw_deg,
            len(update.global_points_cm),
        )
