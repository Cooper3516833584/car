"""Trusted radar localization and occupancy-map policy for Navigation."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import Iterable

from .navigation import (
    NavigationGoal,
    NavigationPose,
    OccupancyGrid,
    VehicleCollisionChecker,
    VehicleGeometry,
)
from .radar_driver import (
    DroneGlobalPointMap,
    Pose2D,
    RadarLocalizationUpdate,
    RectangleFieldCalibration,
    WallFusionStatus,
)


LOG = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class TrustedNavigationMapConfig:
    """Safety gates and map geometry used for accepted D500 observations."""

    resolution_cm: float = 5.0
    margin_cm: float = 15.0
    update_interval_s: float = 0.5
    min_hits: int = 2
    max_pose_step_cm: float = 25.0
    max_yaw_step_deg: float = 15.0
    max_icp_error_cm: float = 10.0
    footprint_clearance_cm: float = 2.0
    navigation_safety_margin_cm: float = 2.0

    def __post_init__(self) -> None:
        if self.resolution_cm <= 0 or self.margin_cm < 0:
            raise ValueError("invalid map geometry")
        if self.update_interval_s <= 0 or self.min_hits <= 0:
            raise ValueError("invalid map update configuration")
        if min(
            self.max_pose_step_cm,
            self.max_yaw_step_deg,
            self.max_icp_error_cm,
        ) <= 0:
            raise ValueError("trusted localization gates must be positive")
        if min(
            self.footprint_clearance_cm,
            self.navigation_safety_margin_cm,
        ) < 0:
            raise ValueError("footprint clearances cannot be negative")


@dataclass(frozen=True, slots=True)
class TrustedMapIngestResult:
    """Result of adding one navigation-accepted radar observation."""

    refreshed_grid: OccupancyGrid | None
    raw_points: int
    retained_points: int
    purged_self_cells: int
    wall_hard_rejected: bool
    wall_reason: str | None


class TrustedNavigationMap:
    """Own the trusted pose gates, self-return filtering and bounded field map.

    The component deliberately does not drive the vehicle or call ``Navigation``.
    The application coordinator first asks it to validate a radar update, then
    lets Navigation accept the pose, and finally ingests the scan here.
    """

    def __init__(
        self,
        config: TrustedNavigationMapConfig,
        vehicle_geometry: VehicleGeometry,
    ) -> None:
        self.config = config
        self.vehicle_geometry = vehicle_geometry
        self.point_map = DroneGlobalPointMap(resolution_cm=config.resolution_cm)
        self._lock = threading.RLock()
        self._calibration: RectangleFieldCalibration | None = None
        self._grid: OccupancyGrid | None = None
        self._last_pose: Pose2D | None = None
        self._last_pose_time = 0.0
        self._last_rejection: str | None = None
        self._last_map_update = 0.0
        self._self_return_clearance_cm = max(
            config.footprint_clearance_cm,
            config.navigation_safety_margin_cm
            + config.resolution_cm / math.sqrt(2.0),
        )

    @property
    def calibration(self) -> RectangleFieldCalibration | None:
        with self._lock:
            return self._calibration

    @property
    def grid(self) -> OccupancyGrid | None:
        with self._lock:
            return self._grid

    @property
    def last_pose(self) -> Pose2D | None:
        with self._lock:
            return self._last_pose

    @property
    def last_pose_time(self) -> float:
        with self._lock:
            return self._last_pose_time

    @property
    def last_rejection(self) -> str | None:
        with self._lock:
            return self._last_rejection

    def initialize(
        self,
        calibration: RectangleFieldCalibration,
        startup_points: Iterable[tuple[float, float]],
        *,
        pose: Pose2D = Pose2D(),
        now: float | None = None,
    ) -> OccupancyGrid:
        """Start a new trusted map in the fitted field coordinate frame."""

        timestamp = time.monotonic() if now is None else now
        retained = self.filter_vehicle_footprint_points(startup_points, pose)
        self.point_map.clear()
        self.point_map.add_points(retained)
        grid = self.build_grid(retained, calibration)
        with self._lock:
            self._calibration = calibration
            self._grid = grid
            self._last_pose = pose
            self._last_pose_time = timestamp
            self._last_rejection = None
            self._last_map_update = timestamp
        return grid

    def rejection_reason(self, update: RadarLocalizationUpdate) -> str | None:
        """Return why an update cannot safely reach Navigation, or ``None``."""

        pose = update.global_pose
        if pose is None:
            return "global alignment unavailable"
        if not update.odometry.accepted:
            return f"odometry rejected: {update.odometry.rejection_reason or 'unknown'}"
        if not all(
            math.isfinite(value)
            for value in (pose.x_cm, pose.y_cm, pose.yaw_cw_deg)
        ):
            return "non-finite global pose"
        icp = update.odometry.icp
        if icp is not None and (
            not math.isfinite(icp.mean_error_cm)
            or icp.mean_error_cm > self.config.max_icp_error_cm
        ):
            return f"ICP error {icp.mean_error_cm:.2f}cm exceeds trusted gate"

        with self._lock:
            calibration = self._calibration
            previous = self._last_pose
        if calibration is None:
            return "field calibration unavailable"
        outside_corners = [
            corner
            for corner in self.vehicle_footprint_corners(pose)
            if not calibration.contains_point(*corner)
        ]
        if outside_corners:
            return f"vehicle footprint outside fitted field at {outside_corners[0]}"
        if previous is not None:
            step_cm = math.hypot(
                pose.x_cm - previous.x_cm,
                pose.y_cm - previous.y_cm,
            )
            if step_cm > self.config.max_pose_step_cm:
                return (
                    f"pose translation jump {step_cm:.2f}cm exceeds "
                    f"{self.config.max_pose_step_cm:.2f}cm"
                )
            yaw_step = abs(
                (pose.yaw_cw_deg - previous.yaw_cw_deg + 180.0) % 360.0 - 180.0
            )
            if yaw_step > self.config.max_yaw_step_deg:
                return (
                    f"pose yaw jump {yaw_step:.2f}deg exceeds "
                    f"{self.config.max_yaw_step_deg:.2f}deg"
                )
        return None

    def record_rejection(self, reason: str) -> None:
        with self._lock:
            self._last_rejection = reason

    def ingest(
        self,
        update: RadarLocalizationUpdate,
        *,
        now: float | None = None,
    ) -> TrustedMapIngestResult:
        """Record a pose already accepted by Navigation and ingest its scan."""

        pose = update.global_pose
        if pose is None:
            raise ValueError("cannot ingest a radar update without a global pose")
        timestamp = time.monotonic() if now is None else now
        with self._lock:
            self._last_pose = pose
            self._last_pose_time = timestamp
            self._last_rejection = None

        wall = update.wall_fusion
        wall_hard_rejected = (
            wall is not None
            and wall.attempted
            and not wall.accepted
            and (
                wall.status is WallFusionStatus.HARD_REJECTED
                or (
                    wall.status is WallFusionStatus.NOT_ATTEMPTED
                    and wall.reason != "no valid wall axes"
                )
            )
        )
        filtered_points = self.filter_vehicle_footprint_points(
            update.global_points_cm,
            pose,
        )
        purged_cells = self.purge_vehicle_footprint(pose)
        if not wall_hard_rejected:
            self.point_map.add_points(filtered_points)
        return TrustedMapIngestResult(
            self.refresh_grid(now=timestamp),
            len(update.global_points_cm),
            len(filtered_points),
            purged_cells,
            wall_hard_rejected,
            None if wall is None else wall.reason,
        )

    def vehicle_footprint_corners(
        self,
        pose: Pose2D,
    ) -> tuple[tuple[float, float], ...]:
        geometry = self.vehicle_geometry
        clearance = self.config.footprint_clearance_cm
        centre_x_body = geometry.rear_axle_to_body_center_cm
        half_length = geometry.body_length_cm / 2.0 + clearance
        half_width = geometry.body_width_cm / 2.0 + clearance
        yaw = math.radians(pose.yaw_cw_deg)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return tuple(
            (
                pose.x_cm + cosine * body_x + sine * body_y,
                pose.y_cm - sine * body_x + cosine * body_y,
            )
            for body_x, body_y in (
                (centre_x_body + half_length, half_width),
                (centre_x_body + half_length, -half_width),
                (centre_x_body - half_length, half_width),
                (centre_x_body - half_length, -half_width),
            )
        )

    def filter_vehicle_footprint_points(
        self,
        points: Iterable[tuple[float, float]],
        pose: Pose2D,
    ) -> list[tuple[float, float]]:
        """Remove radar self-returns and stale hits under the physical car."""

        return [
            (point_x, point_y)
            for point_x, point_y in points
            if not self.point_inside_vehicle_clearance(point_x, point_y, pose)
        ]

    def point_inside_vehicle_clearance(
        self,
        point_x: float,
        point_y: float,
        pose: Pose2D,
    ) -> bool:
        geometry = self.vehicle_geometry
        clearance = self._self_return_clearance_cm
        half_length = geometry.body_length_cm / 2.0 + clearance
        half_width = geometry.body_width_cm / 2.0 + clearance
        yaw = math.radians(pose.yaw_cw_deg)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        dx, dy = point_x - pose.x_cm, point_y - pose.y_cm
        body_x = cosine * dx - sine * dy
        body_y = sine * dx + cosine * dy
        return (
            abs(body_x - geometry.rear_axle_to_body_center_cm) <= half_length
            and abs(body_y) <= half_width
        )

    def purge_vehicle_footprint(self, pose: Pose2D) -> int:
        """Erase historical self-return cells while the car occupies them."""

        return self.point_map.remove_cells(
            lambda point_x, point_y: self.point_inside_vehicle_clearance(
                point_x,
                point_y,
                pose,
            )
        )

    def refresh_grid(
        self,
        *,
        now: float | None = None,
        force: bool = False,
    ) -> OccupancyGrid | None:
        """Build and store a new grid when due; return ``None`` when unchanged."""

        timestamp = time.monotonic() if now is None else now
        with self._lock:
            calibration = self._calibration
            pose = self._last_pose
            elapsed = timestamp - self._last_map_update
        if calibration is None or pose is None:
            return None
        if not force and elapsed < self.config.update_interval_s:
            return None
        cells = self.point_map.cells(min_hits=self.config.min_hits)
        points = self.filter_vehicle_footprint_points(
            ((cell.x_cm, cell.y_cm) for cell in cells),
            pose,
        )
        grid = self.build_grid(points, calibration)
        with self._lock:
            self._grid = grid
            self._last_map_update = timestamp
        LOG.debug(
            "trusted map refresh source_cells=%d retained_after_current_footprint=%d "
            "force=%s",
            len(cells),
            len(points),
            force,
        )
        return grid

    def trusted_cell_count(self) -> int:
        return len(self.point_map.cells(min_hits=self.config.min_hits))

    def goal_has_safe_vehicle_footprint(self, goal: NavigationGoal) -> bool:
        """Return whether at least one complete vehicle goal pose is safe.

        A heading-constrained goal checks that exact orientation.  A
        position-only goal checks the same 10-degree heading lattice used by
        the production coordinator before allowing the planner to run.
        """

        if not isinstance(goal, NavigationGoal):
            raise TypeError("goal must be a NavigationGoal")
        with self._lock:
            calibration = self._calibration
            grid = self._grid
        if calibration is None or grid is None:
            return False
        if not calibration.contains_point(goal.x_cm, goal.y_cm):
            return False
        checker = VehicleCollisionChecker(
            grid,
            self.vehicle_geometry,
            safety_margin_cm=self.config.navigation_safety_margin_cm,
        )
        headings = (
            (goal.final_heading_deg,)
            if goal.final_heading_deg is not None
            else tuple(float(value) for value in range(0, 360, 10))
        )
        for heading_deg in headings:
            navigation_pose = NavigationPose(
                goal.x_cm,
                goal.y_cm,
                heading_deg,
                0.0,
            )
            radar_pose = Pose2D(
                goal.x_cm,
                goal.y_cm,
                (-heading_deg) % 360.0,
            )
            if all(
                calibration.contains_point(*corner)
                for corner in self.vehicle_footprint_corners(radar_pose)
            ) and checker.is_pose_free(navigation_pose):
                return True
        return False

    def build_grid(
        self,
        obstacle_points: Iterable[tuple[float, float]],
        calibration: RectangleFieldCalibration,
    ) -> OccupancyGrid:
        """Build a grid whose fitted rectangle is the only known-free area."""

        points = list(obstacle_points)
        resolution = self.config.resolution_cm
        margin = self.config.margin_cm
        origin_x = math.floor(
            (calibration.min_x_cm - margin) / resolution
        ) * resolution
        origin_y = math.floor(
            (calibration.min_y_cm - margin) / resolution
        ) * resolution
        max_x = math.ceil(
            (calibration.max_x_cm + margin) / resolution
        ) * resolution
        max_y = math.ceil(
            (calibration.max_y_cm + margin) / resolution
        ) * resolution
        width = max(1, round((max_x - origin_x) / resolution))
        height = max(1, round((max_y - origin_y) / resolution))
        grid = OccupancyGrid.from_obstacle_points(
            points,
            resolution_cm=resolution,
            origin_x_cm=origin_x,
            origin_y_cm=origin_y,
            width=width,
            height=height,
        )
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
            len(points),
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            sum(value >= result.occupied_threshold for value in result.cells),
        )
        return result
