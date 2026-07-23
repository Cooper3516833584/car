#!/usr/bin/env python3
"""Autonomous point/pose navigation for the Ackermann car.

Navigation coordinates deliberately use a conventional top-down map frame:
``+X`` is the drone's zero-degree direction, ``+Y`` is left of ``+X``, and
headings increase counter-clockwise in ``[0, 360)``.  Radar poses use the same
XY axes but clockwise-positive yaw, so the boundary conversion is ``heading =
(-yaw_cw) % 360``.

The navigation pose is the midpoint of the rear axle.  Collision checking uses
the measured 230 x 145 mm body rectangle whose centre is 71.25 mm ahead of the
rear axle (equal front/rear body overhang is assumed).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import heapq
import itertools
import logging
import math
import threading
import time
from typing import Callable, Final, Iterable, Sequence

from .ackermann_drive import AckermannDrive, AckermannMotionPlan
from .rear_motor import MotorDirection
from .radar_driver import RadarLocalizationUpdate
from .steering_servo import STEERING_LEFT_MAX_RAD, STEERING_RIGHT_MAX_RAD


DEFAULT_WHEEL_THICKNESS_MM: Final[float] = 26.4
DEFAULT_OUTER_WHEEL_WIDTH_MM: Final[float] = 143.5
DEFAULT_TRACK_WIDTH_MM: Final[float] = 143.5 - 26.4
DEFAULT_WHEELBASE_MM: Final[float] = 142.5
DEFAULT_BODY_LENGTH_MM: Final[float] = 230.0
DEFAULT_BODY_WIDTH_MM: Final[float] = 145.0
DEFAULT_MIN_TURN_RADIUS_MM: Final[float] = 350.0
LOG = logging.getLogger("car-navigation")


class NavigationError(RuntimeError):
    """Base class for navigation failures."""


class PathNotFoundError(NavigationError):
    """No collision-free Ackermann path was found within configured limits."""


class PlanningCancelledError(NavigationError):
    """Planning was superseded by stop, pause, a new goal, or shutdown."""


class NavigationState(Enum):
    IDLE = "idle"
    WAITING_FOR_POSE = "waiting_for_pose"
    PLANNING = "planning"
    FOLLOWING = "following"
    FINAL_APPROACH = "final_approach"
    GEAR_CHANGE = "gear_change"
    ARRIVED = "arrived"
    PAUSED = "paused"
    BLOCKED = "blocked"
    LOCALIZATION_LOST = "localization_lost"
    FAILED = "failed"
    CLOSED = "closed"


def normalize_heading_deg(angle_deg: float) -> float:
    value = float(angle_deg)
    if not math.isfinite(value):
        raise ValueError("heading must be finite")
    normalized = value % 360.0
    return 0.0 if abs(normalized) < 1e-12 else normalized


def signed_heading_error_deg(target_deg: float, current_deg: float) -> float:
    """Shortest target-current error in ``[-180, 180)`` (CCW positive)."""

    return (normalize_heading_deg(target_deg) - normalize_heading_deg(current_deg) + 180.0) % 360.0 - 180.0


def radar_yaw_to_navigation_heading(yaw_cw_deg: float) -> float:
    return normalize_heading_deg(-float(yaw_cw_deg))


def navigation_heading_to_radar_yaw(heading_ccw_deg: float) -> float:
    """Return clockwise-positive yaw normalized to ``[-180, 180)``."""

    yaw = (-normalize_heading_deg(heading_ccw_deg) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(yaw) < 1e-12 else yaw


@dataclass(frozen=True, slots=True)
class NavigationPose:
    x_cm: float
    y_cm: float
    heading_deg: float
    timestamp_s: float = field(default_factory=time.monotonic)

    def __post_init__(self) -> None:
        values = (self.x_cm, self.y_cm, self.timestamp_s)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("pose coordinates and timestamp must be finite")
        object.__setattr__(self, "heading_deg", normalize_heading_deg(self.heading_deg))


@dataclass(frozen=True, slots=True)
class NavigationGoal:
    x_cm: float
    y_cm: float
    final_heading_deg: float | None = None
    position_tolerance_cm: float = 5.0
    heading_tolerance_deg: float = 3.0

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.x_cm)) or not math.isfinite(float(self.y_cm)):
            raise ValueError("goal coordinates must be finite")
        if self.position_tolerance_cm <= 0 or self.heading_tolerance_deg <= 0:
            raise ValueError("goal tolerances must be positive")
        if self.final_heading_deg is not None:
            object.__setattr__(
                self,
                "final_heading_deg",
                normalize_heading_deg(self.final_heading_deg),
            )


@dataclass(frozen=True, slots=True)
class VehicleGeometry:
    wheelbase_cm: float = DEFAULT_WHEELBASE_MM / 10.0
    track_width_cm: float = DEFAULT_TRACK_WIDTH_MM / 10.0
    body_length_cm: float = DEFAULT_BODY_LENGTH_MM / 10.0
    body_width_cm: float = DEFAULT_BODY_WIDTH_MM / 10.0
    min_turn_radius_cm: float = DEFAULT_MIN_TURN_RADIUS_MM / 10.0
    rear_axle_to_body_center_cm: float = DEFAULT_WHEELBASE_MM / 20.0

    def __post_init__(self) -> None:
        if min(
            self.wheelbase_cm,
            self.track_width_cm,
            self.body_length_cm,
            self.body_width_cm,
            self.min_turn_radius_cm,
        ) <= 0:
            raise ValueError("vehicle geometry dimensions must be positive")
        if abs(self.rear_axle_to_body_center_cm) > self.body_length_cm / 2:
            raise ValueError("body centre offset lies outside the vehicle body")

    @property
    def left_min_turn_radius_cm(self) -> float:
        servo_radius = (
            self.wheelbase_cm / math.tan(STEERING_LEFT_MAX_RAD)
            - self.track_width_cm / 2.0
        )
        return max(self.min_turn_radius_cm, servo_radius)

    @property
    def right_min_turn_radius_cm(self) -> float:
        servo_radius = abs(
            self.wheelbase_cm / math.tan(STEERING_RIGHT_MAX_RAD)
            - self.track_width_cm / 2.0
        )
        return max(self.min_turn_radius_cm, servo_radius)

    @property
    def max_left_steering_rad(self) -> float:
        """Largest left command that respects servo and vehicle radius limits."""

        radius = self.left_min_turn_radius_cm
        geometry_limit = math.atan(
            self.wheelbase_cm / (radius + self.track_width_cm / 2.0)
        )
        return min(STEERING_LEFT_MAX_RAD, geometry_limit)

    @property
    def min_right_steering_rad(self) -> float:
        """Largest-magnitude right command that respects the radius limit."""

        radius = self.right_min_turn_radius_cm
        geometry_limit = math.atan(
            self.wheelbase_cm / (-radius + self.track_width_cm / 2.0)
        )
        return max(STEERING_RIGHT_MAX_RAD, geometry_limit)


@dataclass(frozen=True, slots=True)
class OccupancyGrid:
    """Immutable occupancy grid in navigation/drone global centimetres.

    Cell values are ``0`` free, ``100`` occupied and ``-1`` unknown.  Any value
    greater than or equal to ``occupied_threshold`` is treated as occupied.
    """

    resolution_cm: float
    origin_x_cm: float
    origin_y_cm: float
    width: int
    height: int
    cells: tuple[int, ...]
    occupied_threshold: int = 50
    unknown_is_occupied: bool = True

    def __post_init__(self) -> None:
        if not math.isfinite(self.resolution_cm) or self.resolution_cm <= 0:
            raise ValueError("grid resolution must be finite and positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid dimensions must be positive")
        if len(self.cells) != self.width * self.height:
            raise ValueError("cell count does not match grid dimensions")

    @classmethod
    def empty(
        cls,
        *,
        resolution_cm: float,
        origin_x_cm: float,
        origin_y_cm: float,
        width: int,
        height: int,
        unknown_is_occupied: bool = True,
    ) -> "OccupancyGrid":
        return cls(
            resolution_cm,
            origin_x_cm,
            origin_y_cm,
            width,
            height,
            (0,) * (width * height),
            unknown_is_occupied=unknown_is_occupied,
        )

    @classmethod
    def from_obstacle_points(
        cls,
        points_cm: Iterable[tuple[float, float]],
        *,
        resolution_cm: float,
        origin_x_cm: float,
        origin_y_cm: float,
        width: int,
        height: int,
    ) -> "OccupancyGrid":
        """Build a map where supplied hit cells are occupied and all others free.

        This is suitable only when the caller deliberately defines the whole
        rectangle as known space.  A radar hit-only map cannot prove that all
        non-hit cells are free, so that policy must be selected by the caller.
        """

        cells = [0] * (width * height)
        for x_cm, y_cm in points_cm:
            ix = math.floor((x_cm - origin_x_cm) / resolution_cm)
            iy = math.floor((y_cm - origin_y_cm) / resolution_cm)
            if 0 <= ix < width and 0 <= iy < height:
                cells[iy * width + ix] = 100
        return cls(
            resolution_cm,
            origin_x_cm,
            origin_y_cm,
            width,
            height,
            tuple(cells),
            unknown_is_occupied=False,
        )

    def world_to_cell(self, x_cm: float, y_cm: float) -> tuple[int, int]:
        return (
            math.floor((x_cm - self.origin_x_cm) / self.resolution_cm),
            math.floor((y_cm - self.origin_y_cm) / self.resolution_cm),
        )

    def cell_center(self, ix: int, iy: int) -> tuple[float, float]:
        return (
            self.origin_x_cm + (ix + 0.5) * self.resolution_cm,
            self.origin_y_cm + (iy + 0.5) * self.resolution_cm,
        )

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def is_occupied(self, ix: int, iy: int) -> bool:
        if not self.in_bounds(ix, iy):
            return True
        value = self.cells[iy * self.width + ix]
        return value < 0 and self.unknown_is_occupied or value >= self.occupied_threshold


class VehicleCollisionChecker:
    """Collision checks using the full oriented rectangular body footprint."""

    def __init__(
        self,
        grid: OccupancyGrid,
        geometry: VehicleGeometry,
        *,
        safety_margin_cm: float = 2.0,
    ) -> None:
        if safety_margin_cm < 0:
            raise ValueError("safety margin cannot be negative")
        self.grid = grid
        self.geometry = geometry
        self.safety_margin_cm = safety_margin_cm

    def is_pose_free(self, pose: NavigationPose) -> bool:
        heading = math.radians(pose.heading_deg)
        cosine, sine = math.cos(heading), math.sin(heading)
        centre_x = pose.x_cm + cosine * self.geometry.rear_axle_to_body_center_cm
        centre_y = pose.y_cm + sine * self.geometry.rear_axle_to_body_center_cm
        half_length = self.geometry.body_length_cm / 2 + self.safety_margin_cm
        half_width = self.geometry.body_width_cm / 2 + self.safety_margin_cm

        corners = []
        for body_x, body_y in (
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        ):
            corners.append(
                (
                    centre_x + cosine * body_x - sine * body_y,
                    centre_y + sine * body_x + cosine * body_y,
                )
            )
        corner_cells = [self.grid.world_to_cell(x, y) for x, y in corners]
        if any(not self.grid.in_bounds(ix, iy) for ix, iy in corner_cells):
            return False

        min_x, max_x = min(x for x, _ in corners), max(x for x, _ in corners)
        min_y, max_y = min(y for _, y in corners), max(y for _, y in corners)
        min_ix, min_iy = self.grid.world_to_cell(min_x, min_y)
        max_ix, max_iy = self.grid.world_to_cell(max_x, max_y)
        cell_padding = self.grid.resolution_cm / math.sqrt(2.0)
        for iy in range(min_iy, max_iy + 1):
            for ix in range(min_ix, max_ix + 1):
                if not self.grid.is_occupied(ix, iy):
                    continue
                cell_x, cell_y = self.grid.cell_center(ix, iy)
                dx, dy = cell_x - centre_x, cell_y - centre_y
                body_x = cosine * dx + sine * dy
                body_y = -sine * dx + cosine * dy
                if (
                    abs(body_x) <= half_length + cell_padding
                    and abs(body_y) <= half_width + cell_padding
                ):
                    return False
        return True


@dataclass(frozen=True, slots=True)
class PathPoint:
    x_cm: float
    y_cm: float
    heading_deg: float
    direction: MotorDirection = MotorDirection.FORWARD


@dataclass(frozen=True, slots=True)
class NavigationPath:
    points: tuple[PathPoint, ...]
    goal: NavigationGoal
    map_revision: int = 0


@dataclass(frozen=True, slots=True)
class HybridAStarConfig:
    heading_bins: int = 72
    primitive_length_cm: float = 10.0
    collision_sample_cm: float = 2.5
    max_expansions: int = 60000
    max_planning_time_s: float = 5.0
    analytic_expansion_interval: int = 25
    reverse_cost_multiplier: float = 1.35
    gear_change_cost_cm: float = 25.0
    steering_cost_cm: float = 1.0
    safety_margin_cm: float = 2.0

    def __post_init__(self) -> None:
        if self.heading_bins < 8 or self.primitive_length_cm <= 0:
            raise ValueError("invalid Hybrid A* discretization")
        if (
            self.collision_sample_cm <= 0
            or self.max_expansions <= 0
            or self.max_planning_time_s <= 0
            or self.analytic_expansion_interval <= 0
        ):
            raise ValueError("invalid Hybrid A* limits")


@dataclass(slots=True)
class _SearchNode:
    x_cm: float
    y_cm: float
    heading_deg: float
    direction: MotorDirection
    curvature: float
    cost: float
    parent: "_SearchNode | None"


def _integrate_bicycle(
    x_cm: float,
    y_cm: float,
    heading_deg: float,
    signed_distance_cm: float,
    curvature_per_cm: float,
) -> tuple[float, float, float]:
    heading = math.radians(heading_deg)
    if abs(curvature_per_cm) < 1e-12:
        return (
            x_cm + signed_distance_cm * math.cos(heading),
            y_cm + signed_distance_cm * math.sin(heading),
            normalize_heading_deg(heading_deg),
        )
    new_heading = heading + signed_distance_cm * curvature_per_cm
    return (
        x_cm + (math.sin(new_heading) - math.sin(heading)) / curvature_per_cm,
        y_cm + (-math.cos(new_heading) + math.cos(heading)) / curvature_per_cm,
        normalize_heading_deg(math.degrees(new_heading)),
    )


class HybridAStarPlanner:
    """Collision-aware non-holonomic planner with an optional reverse switch."""

    def __init__(
        self,
        geometry: VehicleGeometry = VehicleGeometry(),
        config: HybridAStarConfig = HybridAStarConfig(),
    ) -> None:
        self.geometry = geometry
        self.config = config

    def plan(
        self,
        start: NavigationPose,
        goal: NavigationGoal,
        grid: OccupancyGrid,
        *,
        allow_reverse: bool = False,
        map_revision: int = 0,
        should_cancel: Callable[[], bool] | None = None,
    ) -> NavigationPath:
        started = time.monotonic()
        checker = VehicleCollisionChecker(
            grid,
            self.geometry,
            safety_margin_cm=self.config.safety_margin_cm,
        )
        if not checker.is_pose_free(start):
            raise PathNotFoundError("start vehicle footprint is occupied or outside the map")

        start_direction = MotorDirection.FORWARD
        start_key = self._state_key(start.x_cm, start.y_cm, start.heading_deg, start_direction, grid)
        start_node = _SearchNode(
            start.x_cm,
            start.y_cm,
            start.heading_deg,
            start_direction,
            0.0,
            0.0,
            None,
        )
        best_cost: dict[tuple[int, int, int, int], float] = {start_key: 0.0}
        counter = itertools.count()
        frontier: list[tuple[float, int, tuple[int, int, int, int], _SearchNode]] = [
            (
                self._heuristic(start.x_cm, start.y_cm, start.heading_deg, goal),
                next(counter),
                start_key,
                start_node,
            )
        ]

        directions = [MotorDirection.FORWARD]
        if allow_reverse:
            directions.append(MotorDirection.REVERSE)
        left_radius = self.geometry.left_min_turn_radius_cm
        right_radius = self.geometry.right_min_turn_radius_cm
        curvatures = (
            -1.0 / right_radius,
            -0.5 / right_radius,
            0.0,
            0.5 / left_radius,
            1.0 / left_radius,
        )

        expansions = 0
        while frontier and expansions < self.config.max_expansions:
            if should_cancel is not None and should_cancel():
                raise PlanningCancelledError("planning cancelled")
            elapsed = time.monotonic() - started
            if elapsed >= self.config.max_planning_time_s:
                raise PathNotFoundError(
                    f"planning timed out after {self.config.max_planning_time_s:.1f}s"
                )
            _, _, key, node = heapq.heappop(frontier)
            if node.cost > best_cost.get(key, math.inf) + 1e-9:
                continue
            expansions += 1
            if self._goal_reached(node, goal):
                LOG.debug(
                    "Hybrid A* reached goal expansions=%d elapsed_s=%.3f",
                    expansions,
                    time.monotonic() - started,
                )
                return self._reconstruct(node, goal, map_revision)
            if (
                goal.final_heading_deg is not None
                and (expansions == 1 or expansions % self.config.analytic_expansion_interval == 0)
            ):
                analytic_tail = self._dubins_tail(node, goal, checker)
                if analytic_tail is not None:
                    LOG.debug(
                        "Dubins analytic expansion accepted expansions=%d elapsed_s=%.3f "
                        "tail_points=%d",
                        expansions,
                        time.monotonic() - started,
                        len(analytic_tail),
                    )
                    return self._reconstruct_with_tail(
                        node,
                        analytic_tail,
                        goal,
                        map_revision,
                    )

            for direction in directions:
                signed_length = self.config.primitive_length_cm * direction.value
                for curvature in curvatures:
                    samples = self._primitive_samples(node, signed_length, curvature)
                    if any(not checker.is_pose_free(sample) for sample in samples):
                        continue
                    end = samples[-1]
                    new_key = self._state_key(
                        end.x_cm,
                        end.y_cm,
                        end.heading_deg,
                        direction,
                        grid,
                    )
                    movement_cost = self.config.primitive_length_cm
                    if direction is MotorDirection.REVERSE:
                        movement_cost *= self.config.reverse_cost_multiplier
                    if direction is not node.direction:
                        movement_cost += self.config.gear_change_cost_cm
                    normalized_steering = (
                        abs(curvature) * (left_radius if curvature >= 0 else right_radius)
                    )
                    movement_cost += self.config.steering_cost_cm * normalized_steering
                    new_cost = node.cost + movement_cost
                    if new_cost >= best_cost.get(new_key, math.inf) - 1e-9:
                        continue
                    best_cost[new_key] = new_cost
                    new_node = _SearchNode(
                        end.x_cm,
                        end.y_cm,
                        end.heading_deg,
                        direction,
                        curvature,
                        new_cost,
                        node,
                    )
                    priority = new_cost + self._heuristic(
                        end.x_cm, end.y_cm, end.heading_deg, goal
                    )
                    heapq.heappush(frontier, (priority, next(counter), new_key, new_node))

        reason = "search expansion limit reached" if expansions >= self.config.max_expansions else "search exhausted"
        raise PathNotFoundError(reason)

    def _dubins_tail(
        self,
        node: _SearchNode,
        goal: NavigationGoal,
        checker: VehicleCollisionChecker,
    ) -> tuple[PathPoint, ...] | None:
        """Return a collision-checked forward-only Dubins connection.

        The larger of the left/right minimum radii is used for both turn
        directions.  This is conservative for the mechanically asymmetric
        steering and therefore every generated arc is realizable.
        """

        if goal.final_heading_deg is None:
            return None
        radius_cm = max(
            self.geometry.left_min_turn_radius_cm,
            self.geometry.right_min_turn_radius_cm,
        )
        word = self._shortest_dubins_word(
            node.x_cm,
            node.y_cm,
            math.radians(node.heading_deg),
            goal.x_cm,
            goal.y_cm,
            math.radians(goal.final_heading_deg),
            radius_cm,
        )
        if word is None:
            return None

        x_cm = node.x_cm
        y_cm = node.y_cm
        heading_deg = node.heading_deg
        points: list[PathPoint] = []
        for mode, normalized_length in word:
            segment_length_cm = normalized_length * radius_cm
            if segment_length_cm <= 1e-9:
                continue
            curvature = 0.0
            if mode == "L":
                curvature = 1.0 / radius_cm
            elif mode == "R":
                curvature = -1.0 / radius_cm
            sample_count = max(
                1,
                math.ceil(segment_length_cm / self.config.collision_sample_cm),
            )
            step_cm = segment_length_cm / sample_count
            for _ in range(sample_count):
                x_cm, y_cm, heading_deg = _integrate_bicycle(
                    x_cm,
                    y_cm,
                    heading_deg,
                    step_cm,
                    curvature,
                )
                pose = NavigationPose(x_cm, y_cm, heading_deg, 0.0)
                if not checker.is_pose_free(pose):
                    return None
                points.append(
                    PathPoint(x_cm, y_cm, heading_deg, MotorDirection.FORWARD)
                )
        if not points:
            return None
        final = points[-1]
        if (
            math.hypot(final.x_cm - goal.x_cm, final.y_cm - goal.y_cm) > 1e-3
            or abs(
                signed_heading_error_deg(goal.final_heading_deg, final.heading_deg)
            )
            > 1e-3
        ):
            return None
        return tuple(points)

    @staticmethod
    def _shortest_dubins_word(
        start_x_cm: float,
        start_y_cm: float,
        start_heading_rad: float,
        goal_x_cm: float,
        goal_y_cm: float,
        goal_heading_rad: float,
        radius_cm: float,
    ) -> tuple[tuple[str, float], ...] | None:
        """Compute the shortest normalized Dubins word among all six types."""

        dx = (goal_x_cm - start_x_cm) / radius_cm
        dy = (goal_y_cm - start_y_cm) / radius_cm
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        alpha = (start_heading_rad - bearing) % (2.0 * math.pi)
        beta = (goal_heading_rad - bearing) % (2.0 * math.pi)
        sin_a, sin_b = math.sin(alpha), math.sin(beta)
        cos_a, cos_b = math.cos(alpha), math.cos(beta)
        cos_ab = math.cos(alpha - beta)
        candidates: list[tuple[float, tuple[tuple[str, float], ...]]] = []

        def add(modes: str, values: tuple[float, float, float] | None) -> None:
            if values is None:
                return
            word = tuple(zip(modes, values))
            candidates.append((sum(values), word))

        def sqrt_nonnegative(value: float) -> float | None:
            if value < -1e-10:
                return None
            return math.sqrt(max(0.0, value))

        p = sqrt_nonnegative(
            2.0 + distance * distance - 2.0 * cos_ab
            + 2.0 * distance * (sin_a - sin_b)
        )
        if p is not None:
            angle = math.atan2(cos_b - cos_a, distance + sin_a - sin_b)
            add("LSL", ((-alpha + angle) % (2 * math.pi), p, (beta - angle) % (2 * math.pi)))

        p = sqrt_nonnegative(
            2.0 + distance * distance - 2.0 * cos_ab
            + 2.0 * distance * (-sin_a + sin_b)
        )
        if p is not None:
            angle = math.atan2(cos_a - cos_b, distance - sin_a + sin_b)
            add("RSR", ((alpha - angle) % (2 * math.pi), p, (-beta + angle) % (2 * math.pi)))

        p = sqrt_nonnegative(
            -2.0 + distance * distance + 2.0 * cos_ab
            + 2.0 * distance * (sin_a + sin_b)
        )
        if p is not None:
            angle = (
                math.atan2(-cos_a - cos_b, distance + sin_a + sin_b)
                - math.atan2(-2.0, p)
            )
            add("LSR", ((-alpha + angle) % (2 * math.pi), p, (-beta + angle) % (2 * math.pi)))

        p = sqrt_nonnegative(
            distance * distance - 2.0 + 2.0 * cos_ab
            - 2.0 * distance * (sin_a + sin_b)
        )
        if p is not None:
            angle = (
                math.atan2(cos_a + cos_b, distance - sin_a - sin_b)
                - math.atan2(2.0, p)
            )
            add("RSL", ((alpha - angle) % (2 * math.pi), p, (beta - angle) % (2 * math.pi)))

        value = (
            6.0 - distance * distance + 2.0 * cos_ab
            + 2.0 * distance * (sin_a - sin_b)
        ) / 8.0
        if -1.0 <= value <= 1.0:
            p = (2.0 * math.pi - math.acos(value)) % (2.0 * math.pi)
            angle = math.atan2(cos_a - cos_b, distance - sin_a + sin_b)
            t = (alpha - angle + p / 2.0) % (2.0 * math.pi)
            add("RLR", (t, p, (alpha - beta - t + p) % (2.0 * math.pi)))

        value = (
            6.0 - distance * distance + 2.0 * cos_ab
            + 2.0 * distance * (-sin_a + sin_b)
        ) / 8.0
        if -1.0 <= value <= 1.0:
            p = (2.0 * math.pi - math.acos(value)) % (2.0 * math.pi)
            angle = math.atan2(cos_a - cos_b, distance + sin_a - sin_b)
            t = (-alpha - angle + p / 2.0) % (2.0 * math.pi)
            add("LRL", (t, p, (beta - alpha - t + p) % (2.0 * math.pi)))

        if not candidates:
            return None
        return min(candidates, key=lambda candidate: candidate[0])[1]

    def _primitive_samples(
        self,
        node: _SearchNode,
        signed_length_cm: float,
        curvature: float,
    ) -> list[NavigationPose]:
        count = max(1, math.ceil(abs(signed_length_cm) / self.config.collision_sample_cm))
        samples = []
        for index in range(1, count + 1):
            distance = signed_length_cm * index / count
            x_cm, y_cm, heading = _integrate_bicycle(
                node.x_cm, node.y_cm, node.heading_deg, distance, curvature
            )
            samples.append(NavigationPose(x_cm, y_cm, heading, 0.0))
        return samples

    def _state_key(
        self,
        x_cm: float,
        y_cm: float,
        heading_deg: float,
        direction: MotorDirection,
        grid: OccupancyGrid,
    ) -> tuple[int, int, int, int]:
        ix, iy = grid.world_to_cell(x_cm, y_cm)
        heading_bin = round(normalize_heading_deg(heading_deg) * self.config.heading_bins / 360.0)
        heading_bin %= self.config.heading_bins
        return ix, iy, heading_bin, direction.value

    @staticmethod
    def _goal_reached(node: _SearchNode, goal: NavigationGoal) -> bool:
        if math.hypot(node.x_cm - goal.x_cm, node.y_cm - goal.y_cm) > goal.position_tolerance_cm:
            return False
        return goal.final_heading_deg is None or abs(
            signed_heading_error_deg(goal.final_heading_deg, node.heading_deg)
        ) <= goal.heading_tolerance_deg

    @staticmethod
    def _heuristic(x_cm: float, y_cm: float, heading_deg: float, goal: NavigationGoal) -> float:
        cost = math.hypot(goal.x_cm - x_cm, goal.y_cm - y_cm)
        if goal.final_heading_deg is not None:
            cost += 0.12 * abs(signed_heading_error_deg(goal.final_heading_deg, heading_deg))
        return cost

    @staticmethod
    def _reconstruct(
        node: _SearchNode,
        goal: NavigationGoal,
        map_revision: int,
    ) -> NavigationPath:
        reverse_points: list[PathPoint] = []
        while node is not None:
            reverse_points.append(
                PathPoint(node.x_cm, node.y_cm, node.heading_deg, node.direction)
            )
            node = node.parent
        reverse_points.reverse()
        return NavigationPath(tuple(reverse_points), goal, map_revision)

    @staticmethod
    def _reconstruct_with_tail(
        node: _SearchNode,
        tail: tuple[PathPoint, ...],
        goal: NavigationGoal,
        map_revision: int,
    ) -> NavigationPath:
        prefix: list[PathPoint] = []
        current: _SearchNode | None = node
        while current is not None:
            prefix.append(
                PathPoint(
                    current.x_cm,
                    current.y_cm,
                    current.heading_deg,
                    current.direction,
                )
            )
            current = current.parent
        prefix.reverse()
        prefix.extend(tail)
        return NavigationPath(tuple(prefix), goal, map_revision)


@dataclass(frozen=True, slots=True)
class TrackerCommand:
    speed_mm_s: float
    steering_angle_rad: float
    direction: MotorDirection
    nearest_path_index: int
    cross_track_error_cm: float
    distance_to_goal_cm: float
    signed_cross_track_error_cm: float = 0.0
    heading_error_deg: float = 0.0


@dataclass(frozen=True, slots=True)
class PurePursuitConfig:
    cruise_speed_mm_s: float = 100.0
    max_speed_mm_s: float = 150.0
    approach_speed_mm_s: float = 50.0
    reverse_speed_mm_s: float = 60.0
    min_lookahead_cm: float = 30.0
    max_lookahead_cm: float = 80.0
    slowdown_distance_cm: float = 80.0
    max_path_deviation_cm: float = 20.0
    cross_track_gain: float = 0.35
    heading_gain: float = 0.65
    feedback_softening_speed_cm_s: float = 5.0
    cross_track_slowdown_cm: float = 20.0
    heading_slowdown_deg: float = 35.0
    minimum_tracking_speed_scale: float = 0.40
    nearest_search_ahead_points: int = 30

    def __post_init__(self) -> None:
        speeds = (
            self.cruise_speed_mm_s,
            self.max_speed_mm_s,
            self.approach_speed_mm_s,
            self.reverse_speed_mm_s,
        )
        if min(speeds) <= 0 or self.cruise_speed_mm_s > self.max_speed_mm_s:
            raise ValueError("invalid pursuit speeds")
        if not 0 < self.min_lookahead_cm <= self.max_lookahead_cm:
            raise ValueError("invalid lookahead distances")
        if self.cross_track_gain < 0 or self.heading_gain < 0:
            raise ValueError("tracking feedback gains cannot be negative")
        if self.feedback_softening_speed_cm_s <= 0:
            raise ValueError("feedback softening speed must be positive")
        if self.cross_track_slowdown_cm <= 0 or self.heading_slowdown_deg <= 0:
            raise ValueError("tracking slowdown thresholds must be positive")
        if not 0 < self.minimum_tracking_speed_scale <= 1:
            raise ValueError("minimum tracking speed scale must be in (0, 1]")
        if self.nearest_search_ahead_points < 2:
            raise ValueError("nearest path search window is too small")


class PurePursuitController:
    def __init__(
        self,
        geometry: VehicleGeometry = VehicleGeometry(),
        config: PurePursuitConfig = PurePursuitConfig(),
    ) -> None:
        self.geometry = geometry
        self.config = config

    def compute(
        self,
        pose: NavigationPose,
        path: NavigationPath,
        *,
        min_path_index: int = 0,
    ) -> TrackerCommand:
        """Compute feed-forward curvature plus radar-pose feedback.

        Pure Pursuit supplies the path curvature.  Signed lateral error and
        vehicle-heading error add an explicit pose feedback term, so every new
        radar revolution changes both steering and speed instead of merely
        refreshing an open-loop speed target.
        """

        if len(path.points) < 2:
            raise NavigationError("navigation path has fewer than two points")
        nearest_index, projection, segment_ratio, signed_cross_track = self._nearest_segment(
            pose,
            path,
            min_path_index,
        )
        cross_track = abs(signed_cross_track)
        distance_to_goal = math.hypot(path.goal.x_cm - pose.x_cm, path.goal.y_cm - pose.y_cm)

        speed_for_lookahead = self.config.cruise_speed_mm_s
        lookahead = self.config.min_lookahead_cm + (
            self.config.max_lookahead_cm - self.config.min_lookahead_cm
        ) * min(1.0, speed_for_lookahead / self.config.max_speed_mm_s)
        direction = path.points[min(nearest_index + 1, len(path.points) - 1)].direction
        target_x, target_y = self._lookahead_target(
            path,
            nearest_index,
            projection,
            direction,
            lookahead,
        )

        heading = math.radians(pose.heading_deg)
        dx, dy = target_x - pose.x_cm, target_y - pose.y_cm
        body_y = -math.sin(heading) * dx + math.cos(heading) * dy
        chord_sq = max(dx * dx + dy * dy, 1e-6)
        curvature = 2.0 * body_y / chord_sq
        curvature = max(
            -1.0 / self.geometry.right_min_turn_radius_cm,
            min(1.0 / self.geometry.left_min_turn_radius_cm, curvature),
        )
        if abs(curvature) < 1e-9:
            steering = 0.0
        else:
            radius = 1.0 / curvature
            steering = math.atan(
                self.geometry.wheelbase_cm / (radius + self.geometry.track_width_cm / 2.0)
            )
        path_heading = self._interpolated_heading(
            path.points[nearest_index].heading_deg,
            path.points[nearest_index + 1].heading_deg,
            segment_ratio,
        )
        heading_error = signed_heading_error_deg(path_heading, pose.heading_deg)
        direction_sign = float(direction.value)
        speed_cm_s = self.config.cruise_speed_mm_s / 10.0
        cross_track_feedback = -direction_sign * math.atan2(
            self.config.cross_track_gain * signed_cross_track,
            speed_cm_s + self.config.feedback_softening_speed_cm_s,
        )
        heading_feedback = direction_sign * self.config.heading_gain * math.radians(
            heading_error
        )
        steering += cross_track_feedback + heading_feedback
        # Feedback is added after the Pure Pursuit curvature calculation, so
        # the final command needs the vehicle-radius guard as well as the
        # servo's mechanical guard.  In particular, the asymmetric right
        # travel would otherwise request a radius below the C10B 350 mm limit.
        steering = max(
            self.geometry.min_right_steering_rad,
            min(self.geometry.max_left_steering_rad, steering),
        )

        curve_ratio = min(1.0, abs(curvature) * self.geometry.min_turn_radius_cm)
        speed = self.config.cruise_speed_mm_s * (1.0 - 0.45 * curve_ratio)
        if distance_to_goal < self.config.slowdown_distance_cm:
            approach_ratio = max(0.0, distance_to_goal / self.config.slowdown_distance_cm)
            speed = min(
                speed,
                self.config.approach_speed_mm_s
                + (self.config.cruise_speed_mm_s - self.config.approach_speed_mm_s) * approach_ratio,
            )
        if direction is MotorDirection.REVERSE:
            speed = min(speed, self.config.reverse_speed_mm_s)
        cross_track_scale = self._tracking_speed_scale(
            cross_track,
            self.config.cross_track_slowdown_cm,
        )
        heading_scale = self._tracking_speed_scale(
            abs(heading_error),
            self.config.heading_slowdown_deg,
        )
        speed *= min(cross_track_scale, heading_scale)
        # A configured reverse cap must remain effective even when it is below
        # the normal approach-speed floor.  Otherwise a low reverse setting in
        # main.py would be silently raised back to 50 mm/s.
        speed_cap = (
            self.config.reverse_speed_mm_s
            if direction is MotorDirection.REVERSE
            else self.config.max_speed_mm_s
        )
        speed_floor = min(self.config.approach_speed_mm_s, speed_cap)
        speed = max(speed_floor, min(speed, speed_cap))
        return TrackerCommand(
            speed,
            steering,
            direction,
            nearest_index,
            cross_track,
            distance_to_goal,
            signed_cross_track,
            heading_error,
        )

    def _tracking_speed_scale(self, error: float, full_slowdown_error: float) -> float:
        ratio = min(1.0, abs(error) / full_slowdown_error)
        return 1.0 - (1.0 - self.config.minimum_tracking_speed_scale) * ratio

    def _nearest_segment(
        self,
        pose: NavigationPose,
        path: NavigationPath,
        min_path_index: int,
    ) -> tuple[int, tuple[float, float], float, float]:
        last_segment = len(path.points) - 2
        start = max(0, min(int(min_path_index), last_segment))
        stop = min(last_segment, start + self.config.nearest_search_ahead_points - 1)
        best: tuple[float, int, float, float, float, float] | None = None
        for index in range(start, stop + 1):
            first, second = path.points[index], path.points[index + 1]
            segment_x = second.x_cm - first.x_cm
            segment_y = second.y_cm - first.y_cm
            length_sq = segment_x * segment_x + segment_y * segment_y
            if length_sq <= 1e-9:
                continue
            ratio = max(
                0.0,
                min(
                    1.0,
                    ((pose.x_cm - first.x_cm) * segment_x + (pose.y_cm - first.y_cm) * segment_y)
                    / length_sq,
                ),
            )
            projected_x = first.x_cm + ratio * segment_x
            projected_y = first.y_cm + ratio * segment_y
            offset_x = pose.x_cm - projected_x
            offset_y = pose.y_cm - projected_y
            distance_sq = offset_x * offset_x + offset_y * offset_y
            length = math.sqrt(length_sq)
            signed_error = (segment_x * offset_y - segment_y * offset_x) / length
            candidate = (distance_sq, index, ratio, projected_x, projected_y, signed_error)
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is None:
            raise NavigationError("navigation path contains no usable segment")
        _, index, ratio, projected_x, projected_y, signed_error = best
        return index, (projected_x, projected_y), ratio, signed_error

    @staticmethod
    def _interpolated_heading(start_deg: float, end_deg: float, ratio: float) -> float:
        return normalize_heading_deg(
            start_deg + signed_heading_error_deg(end_deg, start_deg) * ratio
        )

    @staticmethod
    def _lookahead_target(
        path: NavigationPath,
        nearest_index: int,
        projection: tuple[float, float],
        direction: MotorDirection,
        lookahead_cm: float,
    ) -> tuple[float, float]:
        remaining = lookahead_cm
        current_x, current_y = projection
        segment_start = path.points[nearest_index]
        segment_end = path.points[nearest_index + 1]
        if segment_end.direction is direction:
            last_dx = segment_end.x_cm - segment_start.x_cm
            last_dy = segment_end.y_cm - segment_start.y_cm
        else:
            last_dx = 0.0
            last_dy = 0.0
        for index in range(nearest_index + 1, len(path.points)):
            candidate = path.points[index]
            if candidate.direction is not direction:
                # Do not look through a forward/reverse gear change.
                return current_x, current_y
            segment_dx = candidate.x_cm - current_x
            segment_dy = candidate.y_cm - current_y
            segment_length = math.hypot(segment_dx, segment_dy)
            if segment_length >= remaining and segment_length > 1e-9:
                ratio = remaining / segment_length
                return (
                    current_x + ratio * segment_dx,
                    current_y + ratio * segment_dy,
                )
            remaining -= segment_length
            if segment_length > 1e-9:
                last_dx = segment_dx
                last_dy = segment_dy
            current_x, current_y = candidate.x_cm, candidate.y_cm

        # Hybrid A* intentionally stops as soon as a node enters the goal
        # tolerance.  Consequently, the stored path normally ends several
        # centimetres before the requested coordinate.  Returning that final
        # point as the pursuit target makes the chord collapse during the last
        # approach; a small radar heading error can then demand full steering
        # even on a straight path.  Extend the final path tangent only for the
        # lookahead calculation.  Goal arrival is still checked against the
        # real coordinate before every drive command, so this virtual segment
        # cannot make the vehicle drive past the configured goal tolerance.
        tangent_length = math.hypot(last_dx, last_dy)
        if remaining > 0.0 and tangent_length > 1e-9:
            return (
                current_x + remaining * last_dx / tangent_length,
                current_y + remaining * last_dy / tangent_length,
            )
        return current_x, current_y


@dataclass(frozen=True, slots=True)
class NavigationConfig:
    allow_reverse: bool = False
    control_hz: float = 20.0
    localization_timeout_s: float = 0.5
    replan_interval_s: float = 0.5
    gear_change_stop_s: float = 0.25
    max_steering_rate_rad_s: float = 1.2
    deviation_replan_samples: int = 2
    hard_path_deviation_cm: float = 30.0
    radar_error_slowdown_start_cm: float = 4.0
    radar_error_slowdown_stop_cm: float = 10.0
    minimum_radar_speed_scale: float = 0.40
    arrival_confirmation_samples: int = 3
    arrival_confirmation_s: float = 0.5
    correction_failure_samples: int = 3
    correction_failure_min_cross_track_cm: float = 12.0
    correction_failure_min_heading_error_deg: float = 20.0
    correction_failure_worsening_cm: float = 1.0
    steering_saturation_ratio: float = 0.90
    safety_prediction_horizon_s: float = 0.8
    safety_prediction_step_s: float = 0.1

    def __post_init__(self) -> None:
        if (
            self.control_hz <= 0
            or self.localization_timeout_s <= 0
            or self.replan_interval_s < 0
            or self.gear_change_stop_s < 0
            or self.max_steering_rate_rad_s <= 0
        ):
            raise ValueError("invalid navigation timing configuration")
        if self.deviation_replan_samples <= 0 or self.hard_path_deviation_cm <= 0:
            raise ValueError("invalid path-deviation configuration")
        if not 0 <= self.radar_error_slowdown_start_cm < self.radar_error_slowdown_stop_cm:
            raise ValueError("invalid radar ICP error slowdown thresholds")
        if not 0 < self.minimum_radar_speed_scale <= 1:
            raise ValueError("minimum radar speed scale must be in (0, 1]")
        if self.arrival_confirmation_samples <= 0 or self.arrival_confirmation_s < 0:
            raise ValueError("invalid arrival confirmation configuration")
        if (
            self.correction_failure_samples <= 0
            or self.correction_failure_min_cross_track_cm <= 0
            or self.correction_failure_min_heading_error_deg <= 0
            or self.correction_failure_worsening_cm < 0
            or not 0 < self.steering_saturation_ratio <= 1
        ):
            raise ValueError("invalid correction-failure configuration")
        if (
            self.safety_prediction_horizon_s <= 0
            or self.safety_prediction_step_s <= 0
            or self.safety_prediction_step_s > self.safety_prediction_horizon_s
        ):
            raise ValueError("invalid motion-sweep safety configuration")


class Navigation:
    """Threaded safety state machine connecting localization, planning and drive."""

    def __init__(
        self,
        drive: AckermannDrive | None = None,
        *,
        geometry: VehicleGeometry = VehicleGeometry(),
        config: NavigationConfig = NavigationConfig(),
        planner: HybridAStarPlanner | None = None,
        controller: PurePursuitController | None = None,
        max_wheel_speed_mm_s: float = 300.0,
        on_state_changed: Callable[[NavigationState, str], None] | None = None,
        on_path_updated: Callable[[NavigationPath], None] | None = None,
        on_goal_reached: Callable[[NavigationGoal, NavigationPose], None] | None = None,
    ) -> None:
        self.geometry = geometry
        self.config = config
        self._owns_drive = drive is None
        if isinstance(drive, AckermannDrive) and (
            not math.isclose(drive.wheelbase_mm, geometry.wheelbase_cm * 10.0, abs_tol=1e-6)
            or not math.isclose(drive.track_width_mm, geometry.track_width_cm * 10.0, abs_tol=1e-6)
        ):
            raise ValueError(
                "supplied AckermannDrive geometry does not match Navigation geometry"
            )
        self.drive = drive or AckermannDrive(
            wheelbase_mm=geometry.wheelbase_cm * 10.0,
            track_width_mm=geometry.track_width_cm * 10.0,
            max_wheel_speed_mm_s=max_wheel_speed_mm_s,
        )
        self.planner = planner or HybridAStarPlanner(geometry)
        self.controller = controller or PurePursuitController(geometry)
        self.on_state_changed = on_state_changed
        self.on_path_updated = on_path_updated
        self.on_goal_reached = on_goal_reached

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._control_wakeup = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = NavigationState.IDLE
        self._reason = ""
        self._pose: NavigationPose | None = None
        self._grid: OccupancyGrid | None = None
        self._map_revision = 0
        self._goal: NavigationGoal | None = None
        self._path: NavigationPath | None = None
        self._active = False
        self._paused = False
        self._last_plan_time = 0.0
        self._pose_revision = 0
        self._localization_speed_scale = 1.0
        self._path_progress_index = 0
        self._deviation_count = 0
        self._last_deviation_pose_revision = -1
        self._arrival_confirmation_count = 0
        self._last_arrival_pose_revision = -1
        self._arrival_confirmation_started_at: float | None = None
        self._correction_failure_count = 0
        self._last_correction_pose_revision = -1
        self._last_correction_cross_track_cm: float | None = None
        self._last_correction_goal_distance_cm: float | None = None
        self._last_direction: MotorDirection | None = None
        self._pending_direction: MotorDirection | None = None
        self._gear_change_ready_time = 0.0
        self._last_steering_angle_rad = 0.0
        self._last_motion_time: float | None = None
        self._last_tracker_command: TrackerCommand | None = None
        self._last_motion_plan: AckermannMotionPlan | None = None

    @property
    def state(self) -> NavigationState:
        with self._lock:
            return self._state

    @property
    def state_reason(self) -> str:
        with self._lock:
            return self._reason

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def path(self) -> NavigationPath | None:
        with self._lock:
            return self._path

    @property
    def pose(self) -> NavigationPose | None:
        """Latest accepted global/navigation pose, primarily for status output."""

        with self._lock:
            return self._pose

    @property
    def map_revision(self) -> int:
        """Current occupancy-map revision, primarily for diagnostics."""

        with self._lock:
            return self._map_revision

    @property
    def last_tracker_command(self) -> TrackerCommand | None:
        """Latest requested closed-loop correction, retained after a failure."""

        with self._lock:
            return self._last_tracker_command

    @property
    def last_motion_plan(self) -> AckermannMotionPlan | None:
        """Latest command successfully accepted by the unified drive."""

        with self._lock:
            return self._last_motion_plan

    def start(self) -> "Navigation":
        with self._lock:
            if self._thread and self._thread.is_alive():
                return self
            if not self.drive.is_running:
                self.drive.start()
            self._stop_event.clear()
            self._control_wakeup.clear()
            self._thread = threading.Thread(target=self._run, name="car-navigation", daemon=True)
            self._thread.start()
        return self

    def __enter__(self) -> "Navigation":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def close(self) -> None:
        self._stop_event.set()
        self._control_wakeup.set()
        self._safe_stop()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        if self._owns_drive:
            self.drive.close()
        self._set_state(NavigationState.CLOSED, "navigation closed")

    def set_map(self, grid: OccupancyGrid) -> bool:
        """Install a new map and preserve a still-collision-free active path.

        Radar hit maps are refreshed periodically.  A revision that only adds
        obstacles away from the remaining path must not cause a stop/replan
        cycle, otherwise continuous radar mapping produces jerky motion.
        """

        if not isinstance(grid, OccupancyGrid):
            raise TypeError("grid must be an OccupancyGrid")
        path_invalidated = False
        with self._lock:
            if grid == self._grid:
                return False
            previous_path = self._path
            self._grid = grid
            self._map_revision += 1
            if self._path is not None and self._remaining_path_is_free(
                self._path,
                grid,
                self._path_progress_index,
                self._pose,
            ):
                self._path = NavigationPath(
                    self._path.points,
                    self._path.goal,
                    self._map_revision,
                )
            else:
                self._path = None
                self._path_progress_index = 0
                self._reset_deviation_tracking()
                self._reset_correction_tracking()
            path_invalidated = previous_path is not None and self._path is None
        if path_invalidated:
            self._safe_stop()
        self._control_wakeup.set()
        return True

    def update_pose(self, pose: NavigationPose) -> None:
        if not isinstance(pose, NavigationPose):
            raise TypeError("pose must be a NavigationPose")
        with self._lock:
            self._pose = pose
            self._pose_revision += 1
            self._localization_speed_scale = 1.0
        self._control_wakeup.set()

    def update_from_radar(self, update: RadarLocalizationUpdate) -> bool:
        """Accept one valid, globally aligned radar localization update."""

        if update.global_pose is None or not update.odometry.accepted:
            return False
        radar_pose = update.global_pose
        pose = NavigationPose(
            radar_pose.x_cm,
            radar_pose.y_cm,
            radar_yaw_to_navigation_heading(radar_pose.yaw_cw_deg),
            time.monotonic(),
        )
        speed_scale = self._radar_speed_scale(update)
        with self._lock:
            self._pose = pose
            self._pose_revision += 1
            self._localization_speed_scale = speed_scale
        self._control_wakeup.set()
        return True

    def set_goal(self, goal: NavigationGoal) -> None:
        if not isinstance(goal, NavigationGoal):
            raise TypeError("goal must be a NavigationGoal")
        with self._lock:
            self._goal = goal
            self._path = None
            self._path_progress_index = 0
            self._reset_deviation_tracking()
            self._reset_arrival_tracking()
            self._reset_correction_tracking()
            self._active = False
            self._paused = False
            self._last_direction = None
            self._pending_direction = None
            self._last_steering_angle_rad = 0.0
            self._last_motion_time = None
            self._last_tracker_command = None
            self._last_motion_plan = None
        self._safe_stop()
        self._set_state(NavigationState.IDLE, "goal loaded; call start_navigation")

    def start_navigation(self) -> None:
        with self._lock:
            if self._goal is None:
                raise NavigationError("no navigation goal is set")
            if not self._thread or not self._thread.is_alive():
                raise NavigationError("navigation component is not started")
            self._active = True
            self._paused = False
            self._path = None
            self._last_direction = None
            self._pending_direction = None
            self._path_progress_index = 0
            self._reset_deviation_tracking()
            self._reset_arrival_tracking()
            self._reset_correction_tracking()
        self._control_wakeup.set()

    def pause(self) -> None:
        with self._lock:
            self._paused = True
        self._safe_stop()
        self._set_state(NavigationState.PAUSED, "paused by caller")

    def resume(self) -> None:
        with self._lock:
            if self._goal is None:
                raise NavigationError("no navigation goal is set")
            self._paused = False
            self._active = True
            self._path = None
            self._path_progress_index = 0
            self._reset_deviation_tracking()
            self._reset_arrival_tracking()
            self._reset_correction_tracking()
        self._control_wakeup.set()

    def cancel(self, *, reason: str = "navigation cancelled") -> None:
        """Clear only the active mission; retain the current pose and map."""

        if not isinstance(reason, str) or not reason:
            raise ValueError("cancel reason must be a non-empty string")
        with self._lock:
            self._active = False
            self._paused = False
            self._goal = None
            self._path = None
            self._path_progress_index = 0
            self._reset_deviation_tracking()
            self._reset_arrival_tracking()
            self._reset_correction_tracking()
            self._last_direction = None
            self._pending_direction = None
        self._safe_stop()
        self._set_state(NavigationState.IDLE, reason)

    def fail_safe_stop(self, reason: str) -> None:
        """Immediately stop and latch a terminal BLOCKED state.

        This is used for safety evidence that must not wait for the normal
        localization-staleness timeout, such as a fitted-field footprint
        violation or an unsafe predicted stopping sweep.
        """

        if not isinstance(reason, str) or not reason:
            raise ValueError("fail-safe reason must be a non-empty string")
        with self._lock:
            self._active = False
            self._paused = False
            self._path = None
            self._path_progress_index = 0
            self._reset_deviation_tracking()
            self._reset_arrival_tracking()
            self._reset_correction_tracking()
            self._last_direction = None
            self._pending_direction = None
        self._safe_stop()
        self._set_state(NavigationState.BLOCKED, reason)

    def _run(self) -> None:
        period = 1.0 / self.config.control_hz
        while not self._stop_event.is_set():
            self._control_wakeup.clear()
            started = time.monotonic()
            try:
                self._control_step(started)
            except BaseException as exc:
                if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                    raise
                LOG.exception("navigation control step failed")
                self._safe_stop()
                with self._lock:
                    self._active = False
                self._set_state(NavigationState.FAILED, str(exc))
            wait_time = max(0.0, period - (time.monotonic() - started))
            self._control_wakeup.wait(wait_time)

    def _control_step(self, now: float) -> None:
        with self._lock:
            active, paused = self._active, self._paused
            pose, grid, goal, path = self._pose, self._grid, self._goal, self._path
            revision = self._map_revision
            pose_revision = self._pose_revision
            localization_speed_scale = self._localization_speed_scale
            path_progress_index = self._path_progress_index
        if not active:
            return
        if paused:
            self._safe_stop()
            return
        if pose is None:
            self._safe_stop()
            self._set_state(NavigationState.WAITING_FOR_POSE, "no global pose")
            return
        if now - pose.timestamp_s > self.config.localization_timeout_s:
            self._safe_stop()
            self._set_state(NavigationState.LOCALIZATION_LOST, "global pose is stale")
            return
        if grid is None:
            self._safe_stop()
            self._set_state(NavigationState.BLOCKED, "no occupancy map")
            return
        if goal is None:
            self.cancel()
            return
        if self._goal_reached(pose, goal):
            self._safe_stop()
            if self._arrival_confirmed(pose_revision, now):
                self._finish_goal(pose, goal)
            else:
                with self._lock:
                    count = self._arrival_confirmation_count
                self._set_state(
                    NavigationState.FINAL_APPROACH,
                    f"confirming goal arrival {count}/"
                    f"{self.config.arrival_confirmation_samples}",
                )
            return
        with self._lock:
            self._reset_arrival_tracking()

        if path is None or path.map_revision != revision:
            if now - self._last_plan_time < self.config.replan_interval_s:
                self._safe_stop()
                return
            self._safe_stop()
            self._set_state(NavigationState.PLANNING, "planning Ackermann path")
            self._last_plan_time = now
            try:
                path = self.planner.plan(
                    pose,
                    goal,
                    grid,
                    allow_reverse=self.config.allow_reverse,
                    map_revision=revision,
                    should_cancel=lambda: self._planning_was_cancelled(goal),
                )
            except PlanningCancelledError:
                LOG.debug("path planning cancelled before completion")
                self._safe_stop()
                return
            except PathNotFoundError as exc:
                self._set_state(NavigationState.BLOCKED, str(exc))
                return
            with self._lock:
                if (
                    self._stop_event.is_set()
                    or not self._active
                    or self._paused
                    or self._goal is not goal
                    or self._map_revision != revision
                ):
                    return
                self._path = path
                self._path_progress_index = 0
                self._reset_deviation_tracking()
                self._reset_correction_tracking()
            LOG.debug(
                "path planned revision=%d points=%d start=(%.2f,%.2f,%.2f) "
                "end=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%s) reverse_points=%d",
                revision,
                len(path.points),
                path.points[0].x_cm,
                path.points[0].y_cm,
                path.points[0].heading_deg,
                path.points[-1].x_cm,
                path.points[-1].y_cm,
                path.points[-1].heading_deg,
                goal.x_cm,
                goal.y_cm,
                "none"
                if goal.final_heading_deg is None
                else f"{goal.final_heading_deg:.2f}",
                sum(
                    point.direction is MotorDirection.REVERSE
                    for point in path.points
                ),
            )
            if self.on_path_updated is not None:
                try:
                    self.on_path_updated(path)
                except BaseException:
                    pass

        with self._lock:
            latest_pose = self._pose
            pose_revision = self._pose_revision
            localization_speed_scale = self._localization_speed_scale
        if latest_pose is None or time.monotonic() - latest_pose.timestamp_s > self.config.localization_timeout_s:
            self._safe_stop()
            self._set_state(NavigationState.LOCALIZATION_LOST, "global pose became stale while planning")
            return
        pose = latest_pose

        command = self.controller.compute(
            pose,
            path,
            min_path_index=max(0, path_progress_index - 1),
        )
        with self._lock:
            self._last_tracker_command = command
            self._path_progress_index = max(
                self._path_progress_index,
                command.nearest_path_index,
            )
        if self._correction_is_failing(command, pose_revision):
            self.fail_safe_stop(
                "steering correction saturated while path/goal error worsened"
            )
            return
        if command.cross_track_error_cm > self.controller.config.max_path_deviation_cm:
            if pose_revision != self._last_deviation_pose_revision:
                self._last_deviation_pose_revision = pose_revision
                self._deviation_count += 1
            if (
                command.cross_track_error_cm >= self.config.hard_path_deviation_cm
                or self._deviation_count >= self.config.deviation_replan_samples
            ):
                self._safe_stop()
                with self._lock:
                    self._path = None
                    self._path_progress_index = 0
                    self._reset_deviation_tracking()
                self._set_state(NavigationState.PLANNING, "confirmed path deviation requires replanning")
                return
        elif pose_revision != self._last_deviation_pose_revision:
            self._last_deviation_pose_revision = pose_revision
            self._deviation_count = 0
        if self._last_direction is not None and command.direction is not self._last_direction:
            if self._pending_direction is not command.direction:
                self._pending_direction = command.direction
                self._gear_change_ready_time = now + self.config.gear_change_stop_s
            if now < self._gear_change_ready_time:
                self._safe_stop()
                self._set_state(NavigationState.GEAR_CHANGE, "stopped before direction change")
                return
            self._last_direction = command.direction
            self._pending_direction = None
        elif self._last_direction is None:
            self._last_direction = command.direction
            self._pending_direction = None
        steering = self._rate_limited_steering(command.steering_angle_rad, now)
        speed = command.speed_mm_s * localization_speed_scale
        with self._lock:
            path_is_current = (
                self._active
                and not self._paused
                and self._goal is goal
                and self._path is not None
                and self._path.points is path.points
            )
        if not path_is_current:
            self._safe_stop()
            return
        if not self._motion_sweep_is_free(
            pose,
            grid,
            speed,
            steering,
            command.direction,
        ):
            self.fail_safe_stop(
                "predicted stopping sweep intersects obstacle or field boundary"
            )
            return
        try:
            motion_plan = self.drive.set_motion(
                speed,
                steering,
                direction=command.direction,
                rear_differential_linked=True,
            )
        except BaseException as exc:
            LOG.error(
                "drive command rejected pose=(%.3f,%.3f,%.3f) "
                "error=(cross_signed=%.3fcm,heading=%.3fdeg) "
                "direction=%s speed_mm_s=%.3f tracker_steering=%.5f "
                "rate_limited_steering=%.5f error=%s",
                pose.x_cm,
                pose.y_cm,
                pose.heading_deg,
                command.signed_cross_track_error_cm,
                command.heading_error_deg,
                command.direction.name,
                speed,
                command.steering_angle_rad,
                steering,
                exc,
            )
            raise
        with self._lock:
            self._last_motion_plan = motion_plan
        if motion_plan is not None:
            LOG.debug(
                "control pose=(%.3f,%.3f,%.3f) path_index=%d/%d "
                "error=(cross_signed=%.3fcm,heading=%.3fdeg,goal=%.3fcm) "
                "direction=%s speed=(tracker=%.3f,localization_scale=%.3f,applied=%.3f) "
                "steering=(tracker=%.5f,rate_limited=%.5f,applied=%.5f,pwm_us=%d) "
                "radius_mm=%s rear_mm_s=(%.3f,%.3f) c10b=(vx=%d,vz_mrad_s=%d)",
                pose.x_cm,
                pose.y_cm,
                pose.heading_deg,
                command.nearest_path_index,
                len(path.points) - 1,
                command.signed_cross_track_error_cm,
                command.heading_error_deg,
                command.distance_to_goal_cm,
                command.direction.name,
                command.speed_mm_s,
                localization_speed_scale,
                speed,
                command.steering_angle_rad,
                steering,
                motion_plan.steering.angle_rad,
                motion_plan.steering.pulse_us,
                "straight"
                if motion_plan.turn_radius_mm is None
                else f"{motion_plan.turn_radius_mm:.3f}",
                motion_plan.rear.requested.left_mm_s,
                motion_plan.rear.requested.right_mm_s,
                motion_plan.rear.linear_mm_s,
                motion_plan.rear.angular_mrad_s,
            )
        if command.distance_to_goal_cm <= self.controller.config.slowdown_distance_cm:
            self._set_state(NavigationState.FINAL_APPROACH, "approaching goal")
        else:
            self._set_state(NavigationState.FOLLOWING, "following planned path")

    @staticmethod
    def _goal_reached(pose: NavigationPose, goal: NavigationGoal) -> bool:
        if math.hypot(goal.x_cm - pose.x_cm, goal.y_cm - pose.y_cm) > goal.position_tolerance_cm:
            return False
        return goal.final_heading_deg is None or abs(
            signed_heading_error_deg(goal.final_heading_deg, pose.heading_deg)
        ) <= goal.heading_tolerance_deg

    def _planning_was_cancelled(self, goal: NavigationGoal) -> bool:
        with self._lock:
            return (
                self._stop_event.is_set()
                or not self._active
                or self._paused
                or self._goal is not goal
            )

    def _finish_goal(self, pose: NavigationPose, goal: NavigationGoal) -> None:
        self._safe_stop()
        with self._lock:
            self._active = False
            self._path = None
            self._path_progress_index = 0
            self._reset_deviation_tracking()
            self._reset_arrival_tracking()
            self._reset_correction_tracking()
            self._last_direction = None
            self._pending_direction = None
        self._set_state(NavigationState.ARRIVED, "goal position and heading reached")
        if self.on_goal_reached is not None:
            try:
                self.on_goal_reached(goal, pose)
            except BaseException:
                pass

    def _safe_stop(self) -> None:
        with self._lock:
            self._last_steering_angle_rad = 0.0
            self._last_motion_time = None
        try:
            if self.drive.is_running:
                self.drive.stop(center_steering=True)
        except BaseException:
            pass

    def _arrival_confirmed(self, pose_revision: int, now: float) -> bool:
        with self._lock:
            if pose_revision != self._last_arrival_pose_revision:
                self._last_arrival_pose_revision = pose_revision
                self._arrival_confirmation_count += 1
                if self._arrival_confirmation_started_at is None:
                    self._arrival_confirmation_started_at = now
            started_at = self._arrival_confirmation_started_at
            return (
                self._arrival_confirmation_count
                >= self.config.arrival_confirmation_samples
                and started_at is not None
                and now - started_at >= self.config.arrival_confirmation_s
            )

    def _correction_is_failing(
        self,
        command: TrackerCommand,
        pose_revision: int,
    ) -> bool:
        """Detect repeated saturated corrections that move farther off course."""

        with self._lock:
            if pose_revision == self._last_correction_pose_revision:
                return False
            self._last_correction_pose_revision = pose_revision
            previous_cross_track = self._last_correction_cross_track_cm
            previous_goal_distance = self._last_correction_goal_distance_cm
            self._last_correction_cross_track_cm = command.cross_track_error_cm
            self._last_correction_goal_distance_cm = command.distance_to_goal_cm

            steering_limit = (
                self.geometry.max_left_steering_rad
                if command.steering_angle_rad >= 0.0
                else abs(self.geometry.min_right_steering_rad)
            )
            saturated = (
                steering_limit > 0.0
                and abs(command.steering_angle_rad)
                >= steering_limit * self.config.steering_saturation_ratio
            )
            significant_error = (
                command.cross_track_error_cm
                >= self.config.correction_failure_min_cross_track_cm
                or abs(command.heading_error_deg)
                >= self.config.correction_failure_min_heading_error_deg
            )
            worsening = (
                previous_cross_track is not None
                and previous_goal_distance is not None
                and command.cross_track_error_cm
                >= previous_cross_track + self.config.correction_failure_worsening_cm
                and command.distance_to_goal_cm
                >= previous_goal_distance - self.config.correction_failure_worsening_cm
            )
            if saturated and significant_error and worsening:
                self._correction_failure_count += 1
            else:
                self._correction_failure_count = 0
            return (
                self._correction_failure_count
                >= self.config.correction_failure_samples
            )

    def _motion_sweep_is_free(
        self,
        pose: NavigationPose,
        grid: OccupancyGrid,
        speed_mm_s: float,
        steering_angle_rad: float,
        direction: MotorDirection,
    ) -> bool:
        """Check the commanded near-future footprint before touching hardware."""

        checker = VehicleCollisionChecker(
            grid,
            self.geometry,
            safety_margin_cm=self.planner.config.safety_margin_cm,
        )
        if not checker.is_pose_free(pose):
            return False
        signed_speed_cm_s = speed_mm_s * direction.value / 10.0
        if abs(signed_speed_cm_s) < 1e-9:
            return True
        if abs(steering_angle_rad) < 1e-9:
            curvature = 0.0
        else:
            radius_cm = (
                self.geometry.wheelbase_cm / math.tan(steering_angle_rad)
                - self.geometry.track_width_cm / 2.0
            )
            curvature = 1.0 / radius_cm
        x_cm, y_cm, heading_deg = pose.x_cm, pose.y_cm, pose.heading_deg
        elapsed = 0.0
        while elapsed < self.config.safety_prediction_horizon_s - 1e-9:
            step_s = min(
                self.config.safety_prediction_step_s,
                self.config.safety_prediction_horizon_s - elapsed,
            )
            x_cm, y_cm, heading_deg = _integrate_bicycle(
                x_cm,
                y_cm,
                heading_deg,
                signed_speed_cm_s * step_s,
                curvature,
            )
            if not checker.is_pose_free(
                NavigationPose(x_cm, y_cm, heading_deg, pose.timestamp_s)
            ):
                return False
            elapsed += step_s
        return True

    def _rate_limited_steering(self, requested_rad: float, now: float) -> float:
        with self._lock:
            if self._last_motion_time is None:
                elapsed = 1.0 / self.config.control_hz
            else:
                elapsed = max(0.0, now - self._last_motion_time)
            max_delta = self.config.max_steering_rate_rad_s * elapsed
            limited = max(
                self._last_steering_angle_rad - max_delta,
                min(self._last_steering_angle_rad + max_delta, requested_rad),
            )
            self._last_steering_angle_rad = limited
            self._last_motion_time = now
            return limited

    def _radar_speed_scale(self, update: RadarLocalizationUpdate) -> float:
        result = update.odometry.icp
        if result is None or result.mean_error_cm <= self.config.radar_error_slowdown_start_cm:
            return 1.0
        span = (
            self.config.radar_error_slowdown_stop_cm
            - self.config.radar_error_slowdown_start_cm
        )
        ratio = min(
            1.0,
            (result.mean_error_cm - self.config.radar_error_slowdown_start_cm) / span,
        )
        return 1.0 - (1.0 - self.config.minimum_radar_speed_scale) * ratio

    def _remaining_path_is_free(
        self,
        path: NavigationPath,
        grid: OccupancyGrid,
        start_index: int,
        current_pose: NavigationPose | None,
    ) -> bool:
        checker = VehicleCollisionChecker(
            grid,
            self.geometry,
            safety_margin_cm=self.planner.config.safety_margin_cm,
        )
        sample_distance = self.planner.config.collision_sample_cm
        if current_pose is not None and not checker.is_pose_free(current_pose):
            return False
        start = max(0, min(start_index, len(path.points) - 1))
        if not checker.is_pose_free(
            NavigationPose(
                path.points[start].x_cm,
                path.points[start].y_cm,
                path.points[start].heading_deg,
                0.0,
            )
        ):
            return False
        for index in range(start + 1, len(path.points)):
            first, second = path.points[index - 1], path.points[index]
            distance = math.hypot(second.x_cm - first.x_cm, second.y_cm - first.y_cm)
            samples = max(1, math.ceil(distance / sample_distance))
            heading_delta = signed_heading_error_deg(second.heading_deg, first.heading_deg)
            for sample in range(1, samples + 1):
                ratio = sample / samples
                pose = NavigationPose(
                    first.x_cm + ratio * (second.x_cm - first.x_cm),
                    first.y_cm + ratio * (second.y_cm - first.y_cm),
                    first.heading_deg + ratio * heading_delta,
                    0.0,
                )
                if not checker.is_pose_free(pose):
                    return False
        return True

    def _reset_deviation_tracking(self) -> None:
        self._deviation_count = 0
        self._last_deviation_pose_revision = -1

    def _reset_arrival_tracking(self) -> None:
        self._arrival_confirmation_count = 0
        self._last_arrival_pose_revision = -1
        self._arrival_confirmation_started_at = None

    def _reset_correction_tracking(self) -> None:
        self._correction_failure_count = 0
        self._last_correction_pose_revision = -1
        self._last_correction_cross_track_cm = None
        self._last_correction_goal_distance_cm = None

    def _set_state(self, state: NavigationState, reason: str) -> None:
        callback = None
        with self._lock:
            if self._state is state and self._reason == reason:
                return
            self._state = state
            self._reason = reason
            callback = self.on_state_changed
        if callback is not None:
            try:
                callback(state, reason)
            except BaseException:
                pass
