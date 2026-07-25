"""Pure 3x5 rescue routing plus a small sequential mission controller."""

from collections import deque
from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import Callable, FrozenSet, Iterable, Optional, Tuple

from .fleet_models import (
    AckReason,
    AckStatus,
    CommandResult,
    DisasterRescueCommand,
    TerrainCode,
)
from .ackermann_drive import AckermannDrive
from .navigation import (
    NavigationPose,
    OccupancyGrid,
    normalize_heading_deg,
    signed_heading_error_deg,
)
from .rear_motor import UnsupportedWheelCommand


Cell = Tuple[int, int]
LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class GridLayout:
    x_centres_cm: Tuple[float, ...] = (0.0, 70.0, 140.0, 210.0, 280.0)
    y_centres_cm: Tuple[float, ...] = (0.0, 70.0, 140.0)
    start_point_cm: Tuple[float, float] = (0.0, 0.0)
    start_entry_cells: Tuple[Cell, ...] = ((0, 0),)
    terrain_half_size_cm: float = 35.0

    def __post_init__(self) -> None:
        if len(self.x_centres_cm) != 5 or len(self.y_centres_cm) != 3:
            raise ValueError("rescue layout must be exactly 3x5")
        if not self.start_entry_cells:
            raise ValueError("at least one start entry cell is required")
        for cell in self.start_entry_cells:
            self.validate_cell(cell)

    @staticmethod
    def validate_cell(cell: Cell) -> None:
        row, col = cell
        if not 0 <= row < 3 or not 0 <= col < 5:
            raise ValueError("cell is outside the 3x5 terrain grid")

    def centre(self, cell: Cell) -> Tuple[float, float]:
        self.validate_cell(cell)
        row, col = cell
        return self.x_centres_cm[col], self.y_centres_cm[row]

    def step_heading_deg(self, current: Cell, target: Cell) -> float:
        """Return the cardinal heading for one orthogonally adjacent step."""

        self.validate_cell(current)
        self.validate_cell(target)
        delta_row = target[0] - current[0]
        delta_col = target[1] - current[1]
        if abs(delta_row) + abs(delta_col) != 1:
            raise ValueError("grid move must target one orthogonally adjacent cell")
        if delta_col == 1:
            return 0.0
        if delta_row == 1:
            return 90.0
        if delta_col == -1:
            return 180.0
        return 270.0

    @staticmethod
    def all_cells() -> FrozenSet[Cell]:
        return frozenset((row, col) for row in range(3) for col in range(5))


@dataclass(frozen=True)
class RescueRoutePlan:
    water_cell: Cell
    wildfire_cell: Cell
    to_water: Tuple[Cell, ...]
    to_wildfire: Tuple[Cell, ...]
    to_start_entry: Tuple[Cell, ...]
    blocked_to_water: FrozenSet[Cell]
    blocked_after_water: FrozenSet[Cell]

    @property
    def driven_cells(self) -> Tuple[Cell, ...]:
        return self.to_water + self.to_wildfire + self.to_start_entry


class RescuePlanError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class InPlaceTurnConfig:
    wheel_speed_mm_s: float = 80.0
    heading_tolerance_deg: float = 4.0
    localization_timeout_s: float = 0.5
    timeout_s: float = 8.0
    refresh_interval_s: float = 0.05
    confirmation_samples: int = 2

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(float(value)) and float(value) > 0.0
            for value in (
                self.wheel_speed_mm_s,
                self.heading_tolerance_deg,
                self.localization_timeout_s,
                self.timeout_s,
                self.refresh_interval_s,
            )
        ):
            raise ValueError("in-place turn parameters must be finite and positive")
        if self.heading_tolerance_deg >= 45.0:
            raise ValueError("heading_tolerance_deg must be below 45 degrees")
        if self.confirmation_samples <= 0:
            raise ValueError("confirmation_samples must be positive")


class InPlaceDifferentialTurn:
    """Radar-heading closed-loop pivot with centred front wheels.

    Counter-clockwise turns command the left rear wheel backwards and the
    right rear wheel forwards.  The inverse is used for clockwise turns.  This
    manoeuvre is explicitly opt-in on ``RearMotorDriver`` and is not used by
    normal Ackermann Navigation.
    """

    def __init__(
        self,
        drive: AckermannDrive,
        *,
        pose_provider: Callable[[], NavigationPose | None],
        config: InPlaceTurnConfig = InPlaceTurnConfig(),
        on_motion_changed: Callable[[bool], None] = lambda _moving: None,
        stop_requested: Callable[[], bool] = lambda: False,
    ) -> None:
        self.drive = drive
        self.pose_provider = pose_provider
        self.config = config
        self.on_motion_changed = on_motion_changed
        self.stop_requested = stop_requested

    def turn_to(self, heading_deg: float) -> None:
        target = normalize_heading_deg(heading_deg)
        if not self.drive.rear_motors.allow_in_place_rotation:
            raise RuntimeError(
                "rear motor driver has not enabled the adjacent-grid "
                "in-place rotation mode"
            )

        # Stop first, then centre the physical front wheels before any rear
        # wheel is allowed to move.
        self.drive.stop(center_steering=True)
        deadline = time.monotonic() + self.config.timeout_s
        last_confirmation_timestamp: float | None = None
        confirmations = 0
        moving = False
        LOG.info(
            "adjacent-grid pivot started target_heading=%.1fdeg speed=%.1fmm/s",
            target,
            self.config.wheel_speed_mm_s,
        )
        try:
            while True:
                if self.stop_requested():
                    raise RuntimeError("in-place turn cancelled")
                now = time.monotonic()
                if now >= deadline:
                    raise RuntimeError(
                        f"in-place turn timed out before heading {target:.1f}deg"
                    )
                pose = self.pose_provider()
                if pose is None or now - pose.timestamp_s > self.config.localization_timeout_s:
                    raise RuntimeError("in-place turn requires a fresh radar pose")
                error = signed_heading_error_deg(target, pose.heading_deg)
                if abs(error) <= self.config.heading_tolerance_deg:
                    self.drive.rear_motors.stop()
                    if pose.timestamp_s != last_confirmation_timestamp:
                        confirmations += 1
                        last_confirmation_timestamp = pose.timestamp_s
                    if confirmations >= self.config.confirmation_samples:
                        LOG.info(
                            "adjacent-grid pivot complete heading=%.1fdeg error=%+.2fdeg",
                            pose.heading_deg,
                            error,
                        )
                        return
                else:
                    confirmations = 0
                    last_confirmation_timestamp = None
                    direction = 1.0 if error > 0.0 else -1.0
                    speed = self.config.wheel_speed_mm_s
                    try:
                        self.drive.rear_motors.set_wheels(
                            -direction * speed,
                            direction * speed,
                        )
                    except UnsupportedWheelCommand as exc:
                        raise RuntimeError(
                            "C10B rejected the configured in-place wheel command"
                        ) from exc
                    if not moving:
                        self.on_motion_changed(True)
                        moving = True
                    LOG.debug(
                        "adjacent-grid pivot feedback current=%.2f target=%.2f "
                        "error=%+.2f rear=(%+.1f,%+.1f)mm/s",
                        pose.heading_deg,
                        target,
                        error,
                        -direction * speed,
                        direction * speed,
                    )
                time.sleep(self.config.refresh_interval_s)
        finally:
            try:
                self.drive.rear_motors.stop()
            finally:
                try:
                    self.drive.steering.center()
                finally:
                    if moving:
                        self.on_motion_changed(False)


class AdjacentGridNavigator:
    """Move between centres of adjacent cells in the fixed 3x5/70 cm field."""

    def __init__(
        self,
        pivot_turn: InPlaceDifferentialTurn,
        *,
        navigate_to: Callable[[float, float, float | None], bool],
        layout: GridLayout = GridLayout(),
    ) -> None:
        self.pivot_turn = pivot_turn
        self.navigate_to = navigate_to
        self.layout = layout

    def move(self, current: Optional[Cell], target: Optional[Cell]) -> bool:
        """Pivot to the step direction, then drive to the target cell centre."""

        if target is None:
            if current not in self.layout.start_entry_cells:
                raise ValueError("only a start-entry cell can return to start point")
            self.pivot_turn.turn_to(0.0)
            x_cm, y_cm = self.layout.start_point_cm
            return bool(self.navigate_to(x_cm, y_cm, 0.0))
        self.layout.validate_cell(target)
        if current is None:
            if target not in self.layout.start_entry_cells:
                raise ValueError("the first grid move must enter a configured start cell")
            x_cm, y_cm = self.layout.centre(target)
            return bool(self.navigate_to(x_cm, y_cm, None))

        heading = self.layout.step_heading_deg(current, target)
        LOG.info(
            "adjacent-grid move current=%s target=%s centre=(%.1f,%.1f) heading=%.1f",
            current,
            target,
            *self.layout.centre(target),
            heading,
        )
        self.pivot_turn.turn_to(heading)
        x_cm, y_cm = self.layout.centre(target)
        return bool(self.navigate_to(x_cm, y_cm, heading))


class AdjacentGridRescuePlanner:
    """Select one water cell and route using orthogonal neighbours only."""

    def __init__(
        self,
        *,
        water_terrain_codes: Iterable[int],
        wildfire_terrain_codes: Iterable[int],
        forbidden_terrain_codes: Iterable[int],
        layout: GridLayout = GridLayout(),
    ) -> None:
        self.water_codes = frozenset(int(value) for value in water_terrain_codes)
        self.wildfire_codes = frozenset(int(value) for value in wildfire_terrain_codes)
        self.forbidden_codes = frozenset(int(value) for value in forbidden_terrain_codes)
        self.layout = layout
        if not self.water_codes:
            raise ValueError("water terrain list must not be empty")
        if not self.wildfire_codes:
            raise ValueError("wildfire terrain list must not be empty")

    def plan(self, command: DisasterRescueCommand) -> RescueRoutePlan:
        if len(command.terrain_codes) != 15:
            raise RescuePlanError("terrain grid must contain 15 cells")
        fire = (command.wildfire_row, command.wildfire_col)
        self.layout.validate_cell(fire)
        terrain = {
            (row, col): int(command.terrain_codes[row * 5 + col])
            for row in range(3)
            for col in range(5)
        }
        if terrain[fire] not in self.wildfire_codes:
            raise RescuePlanError("reported wildfire cell is not an enabled fire target")

        forbidden = {
            cell
            for cell, code in terrain.items()
            if code in self.forbidden_codes or code == int(TerrainCode.UNKNOWN)
        }
        pickup_cells = {cell for cell, code in terrain.items() if code in self.water_codes}
        actual_water_cells = {
            cell
            for cell, code in terrain.items()
            if code in (int(TerrainCode.RIVER), int(TerrainCode.LAKE))
        }
        if not pickup_cells:
            raise RescuePlanError("no enabled water pickup cell exists")
        if fire in forbidden or fire in actual_water_cells:
            raise RescuePlanError("wildfire target conflicts with water/forbidden terrain")

        candidates = []
        for water in sorted(pickup_cells):
            inbound_blocked = forbidden | (actual_water_cells - {water})
            to_water = self._from_start(water, inbound_blocked)
            if to_water is None:
                continue
            after_water_blocked = forbidden | actual_water_cells | {water}
            water_to_fire = self._shortest(water, {fire}, after_water_blocked)
            fire_to_entry = self._shortest(
                fire, set(self.layout.start_entry_cells), after_water_blocked
            )
            if water_to_fire is None or fire_to_entry is None:
                continue
            plan = RescueRoutePlan(
                water,
                fire,
                tuple(to_water),
                tuple(water_to_fire[1:]),
                tuple(fire_to_entry[1:]),
                frozenset(inbound_blocked),
                frozenset(after_water_blocked),
            )
            candidates.append((len(plan.driven_cells), water, plan))
        if not candidates:
            raise RescuePlanError(
                "no adjacent route can visit water exactly once, reach wildfire and return"
            )
        return min(candidates, key=lambda item: (item[0], item[1]))[2]

    def _from_start(self, goal: Cell, blocked: set) -> Optional[Tuple[Cell, ...]]:
        paths = []
        for entry in self.layout.start_entry_cells:
            path = self._shortest(entry, {goal}, blocked)
            if path is not None:
                paths.append(path)
        return min(paths, key=len) if paths else None

    @staticmethod
    def _neighbours(cell: Cell):
        row, col = cell
        for neighbour in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if 0 <= neighbour[0] < 3 and 0 <= neighbour[1] < 5:
                yield neighbour

    def _shortest(
        self, start: Cell, goals: set, blocked: set
    ) -> Optional[Tuple[Cell, ...]]:
        if start in goals:
            return (start,)
        queue = deque(((start,),))
        visited = {start}
        while queue:
            path = queue.popleft()
            for neighbour in self._neighbours(path[-1]):
                if neighbour in visited or neighbour in blocked:
                    continue
                next_path = path + (neighbour,)
                if neighbour in goals:
                    return next_path
                visited.add(neighbour)
                queue.append(next_path)
        return None


def overlay_blocked_terrain(
    grid: OccupancyGrid,
    blocked_cells: Iterable[Cell],
    layout: GridLayout = GridLayout(),
) -> OccupancyGrid:
    """Add semantic obstacles without ever freeing a radar-occupied cell."""

    cells = list(grid.cells)
    half = layout.terrain_half_size_cm
    for cell in blocked_cells:
        centre_x, centre_y = layout.centre(cell)
        min_ix, min_iy = grid.world_to_cell(centre_x - half, centre_y - half)
        max_ix, max_iy = grid.world_to_cell(centre_x + half, centre_y + half)
        for iy in range(max(0, min_iy), min(grid.height - 1, max_iy) + 1):
            for ix in range(max(0, min_ix), min(grid.width - 1, max_ix) + 1):
                cells[iy * grid.width + ix] = 100
    return OccupancyGrid(
        grid.resolution_cm,
        grid.origin_x_cm,
        grid.origin_y_cm,
        grid.width,
        grid.height,
        tuple(cells),
        grid.occupied_threshold,
        grid.unknown_is_occupied,
    )


class GridRescueMissionController:
    """Run one accepted plan outside the FleetBus receive worker."""

    def __init__(
        self,
        planner: AdjacentGridRescuePlanner,
        *,
        navigate: Callable[[Optional[Cell]], bool],
        move_adjacent: Callable[[Optional[Cell], Optional[Cell]], bool] | None = None,
        set_step_overlay: Callable[
            [Optional[Cell], Optional[Cell], FrozenSet[Cell]], None
        ],
        clear_overlay: Callable[[], None],
        indicator: Callable[[str, bool], None] = lambda _stage, _active: None,
        on_result: Callable[[CommandResult], None] = lambda _result: None,
        hold_seconds: float = 3.0,
    ) -> None:
        self.planner = planner
        self._navigate = navigate
        self._move_adjacent = move_adjacent
        self._set_step_overlay = set_step_overlay
        self._clear_overlay = clear_overlay
        self._indicator = indicator
        self._on_result = on_result
        self._hold_seconds = hold_seconds
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None  # type: Optional[threading.Thread]
        self._completed_event_ids = set()

    @property
    def stop_requested(self) -> bool:
        return self._stop.is_set()

    def submit(self, command: DisasterRescueCommand) -> CommandResult:
        with self._lock:
            if command.event_id in self._completed_event_ids:
                return CommandResult(AckStatus.COMPLETED)
            if self._thread is not None and self._thread.is_alive():
                return CommandResult(AckStatus.REJECTED, AckReason.BUSY)
            try:
                plan = self.planner.plan(command)
            except (RescuePlanError, ValueError) as exc:
                return CommandResult(AckStatus.REJECTED, AckReason.BAD_PAYLOAD, str(exc))
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run,
                args=(command.event_id, plan),
                name="grid-rescue-mission",
                daemon=True,
            )
            self._thread.start()
        return CommandResult(AckStatus.ACCEPTED)

    def stop(self) -> None:
        self._stop.set()

    def _run(self, event_id: int, plan: RescueRoutePlan) -> None:
        result = CommandResult(AckStatus.FAILED, AckReason.INTERNAL_ERROR)
        current = None  # type: Optional[Cell]
        try:
            for target in plan.to_water:
                self._move(current, target, plan.blocked_to_water)
                current = target
            self._hold("water")
            for target in plan.to_wildfire:
                self._move(current, target, plan.blocked_after_water)
                current = target
            self._hold("wildfire")
            for target in plan.to_start_entry:
                self._move(current, target, plan.blocked_after_water)
                current = target
            self._move(current, None, plan.blocked_after_water)
            with self._lock:
                self._completed_event_ids.add(event_id)
            result = CommandResult(AckStatus.COMPLETED)
        except RuntimeError as exc:
            result = CommandResult(AckStatus.FAILED, AckReason.INTERNAL_ERROR, str(exc))
        finally:
            self._indicator("water", False)
            self._indicator("wildfire", False)
            self._clear_overlay()
            self._on_result(result)

    def _move(
        self,
        current: Optional[Cell],
        target: Optional[Cell],
        blocked_cells: FrozenSet[Cell],
    ) -> None:
        if self._stop.is_set():
            raise RuntimeError("rescue mission stopped")
        step_blocked = frozenset(
            cell for cell in blocked_cells if cell not in (current, target)
        )
        self._set_step_overlay(current, target, step_blocked)
        reached = (
            self._navigate(target)
            if self._move_adjacent is None
            else self._move_adjacent(current, target)
        )
        if not reached:
            raise RuntimeError("navigation did not reach the next adjacent rescue cell")

    def _hold(self, stage: str) -> None:
        self._indicator(stage, True)
        if self._stop.wait(self._hold_seconds):
            raise RuntimeError("rescue mission stopped during hold")
        self._indicator(stage, False)
